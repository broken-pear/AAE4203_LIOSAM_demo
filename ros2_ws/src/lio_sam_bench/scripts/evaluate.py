#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
evaluate.py —— 离线评测：ATE / RPE / 终点误差 / 鲁棒性 / 输出频率
===============================================================================

输入是 traj_recorder.py 写出的 CSV（gt 行 + est 行），输出是
  * 一份 JSON（每个数字都能追溯）
  * 一张 Markdown 表（直接贴进报告/PPT）
  * 终端摘要

为什么评测要单独做成一个脚本
----------------------------
两条算法的"好不好"必须在**同一个评测器**里算，否则数字没有可比性。
所以本脚本只依赖 numpy，不依赖 ROS，也能在没装 ROS 的机器上复算别人的 CSV。

关联（association）规则 —— 这是最容易出错的地方
------------------------------------------------
est 行的时间戳来自算法自己：
  * LIO-SAM  精确到纳秒，与点云时间戳一致          -> mode = exact
  * A-LOAM   只剩整数秒（上游把 header.stamp.sec 当 double 用）-> mode = floor

于是关联分两种：
  exact : 每条 est 找时间最近的 gt，要求 |dt| <= 5 ms，且必须单调
  floor : 两条流按顺序配对，先搜索一个统一的滞后量 lag（暖机帧造成的偏移），
          再验证 floor(gt_t[k+lag]) == floor(est_t[k])；合格率要 > 0.95

脚本会把 match_fraction / lag / median_dt 一起输出。**如果合格率低，ATE 就不可信**，
先去看关联诊断，别急着比较算法。

指标定义（报告里要写清楚）
--------------------------
ATE  : 先用 SE(3) Umeyama（无尺度）把估计轨迹对齐到真值，再算逐帧平移误差
       rmse / mean / median / max / std。对齐是标准做法：两条算法各自从自己的
       原点出发，不对齐的话结果里混着常数平移，没有意义。
       同时给出对齐前的 raw rmse，用来说明"漂移有多大"。
RPE  : 相隔 delta 秒的两帧，相对位姿误差（KITTI 口径）
       trans drift = ||trans(E)|| / ||trans(Q_rel)|| * 100   [%/m]
       rot   drift = angle(E_rot) / ||trans(Q_rel)||         [°/m]
final: 对齐后最后一帧的位置误差 / 偏航误差 —— 对应"绕回起点时的接缝"
robust: 逐帧误差 > 0.5 m 的帧占比；最大误差 > 5 m 记为发散
rate : 输出频率（用 node_now 算，避开 A-LOAM 的粗时间戳）
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np


# ============================================================ 基础几何工具
def quat_to_R(q):
    """(x,y,z,w) -> 3x3 旋转矩阵。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_rotvec_deg(R):
    """旋转矩阵 -> 旋转角（度）。"""
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def yaw_from_R(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def umeyama_align(src, dst):
    """求 R,t 使 ||dst - (R·src + t)|| 最小（无尺度）。src/dst: (N,3)。"""
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    S = (dst - mu_d).T @ (src - mu_s) / n
    U, _D, Vt = np.linalg.svd(S)
    S_fix = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S_fix[-1, -1] = -1.0
    R = U @ S_fix @ Vt
    t = mu_d - R @ mu_s
    return R, t


def stats(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return {'rmse': None, 'mean': None, 'median': None, 'max': None, 'std': None}
    return {
        'rmse': float(np.sqrt(np.mean(x * x))),
        'mean': float(np.mean(x)),
        'median': float(np.median(x)),
        'max': float(np.max(x)),
        'std': float(np.std(x)),
    }


# ================================================================ 读 CSV
def load_traj(csv_path):
    gt_t, gt_p, gt_q = [], [], []
    es_t, es_p, es_q = [], [], []
    es_now = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            kind = row['kind']
            t = float(row['stamp'])
            p = [float(row['x']), float(row['y']), float(row['z'])]
            q = [float(row['qx']), float(row['qy']),
                 float(row['qz']), float(row['qw'])]
            if kind == 'gt':
                gt_t.append(t)
                gt_p.append(p)
                gt_q.append(q)
            elif kind == 'est':
                es_t.append(t)
                es_p.append(p)
                es_q.append(q)
                es_now.append(float(row['node_now']))
    out = {
        'gt': {'t': np.array(gt_t), 'p': np.array(gt_p), 'q': np.array(gt_q)},
        'est': {'t': np.array(es_t), 'p': np.array(es_p), 'q': np.array(es_q),
                'now': np.array(es_now)},
    }
    return out


def load_meta(csv_path):
    meta_path = os.path.splitext(csv_path)[0] + '.meta.json'
    if not os.path.isfile(meta_path):
        return {}
    try:
        with open(meta_path, encoding='utf-8') as f:
            return json.load(f)
    except Exception:                                  # noqa: BLE001
        return {}


# ================================================================ 关联
def associate(gt_t, es_t, mode, tol_exact=5e-3):
    """
    返回 (gt_idx, est_idx, info)。

    exact : est 时间戳可信 -> 最近邻 + 单调 + 容差
    floor : est 只剩整数秒 -> 顺序配对 + 全局滞后搜索 + floor 一致性校验
    """
    n_g, n_e = len(gt_t), len(es_t)
    if n_g == 0 or n_e == 0:
        return np.array([], dtype=int), np.array([], dtype=int), {
            'mode': mode, 'matched': 0, 'match_fraction': 0.0,
            'reason': 'gt 或 est 为空'}

    # ---------------- exact：最近邻
    if mode == 'exact':
        gi, ei = [], []
        last = -1
        for k in range(n_e):
            j = int(np.argmin(np.abs(gt_t - es_t[k])))
            if j <= last:                      # 保证单调递增
                j = last + 1
            if j >= n_g:
                break
            if abs(gt_t[j] - es_t[k]) > tol_exact * 10:   # 50 ms 上限
                continue
            gi.append(j)
            ei.append(k)
            last = j
        gi = np.array(gi, dtype=int)
        ei = np.array(ei, dtype=int)
        dt = np.abs(gt_t[gi] - es_t[ei]) if gi.size else np.array([])
        frac = float(gi.size) / float(n_e)
        return gi, ei, {
            'mode': 'exact',
            'matched': int(gi.size),
            'match_fraction': frac,
            'median_dt_ms': float(np.median(dt) * 1e3) if dt.size else None,
            'max_dt_ms': float(np.max(dt) * 1e3) if dt.size else None,
        }

    # ---------------- floor：全局帧滞后搜索 + 同秒校验
    #
    # A-LOAM 的时间戳只保留了整数秒，真实时刻丢了。能确定的是：
    #   1) 配对的两帧应该落在同一整数秒里；
    #   2) 帧的先后顺序可信。
    # 于是"全局帧滞后 d"（est[i] 对 gt[i+d]）可以用上面的同秒条件打分，
    # 取分数最高的 d。d 允许正负：
    #   d > 0：真值比算法多（算法暖机/前几帧没输出）—— 常见
    #   d < 0：算法比真值多（真值前几帧还没同步上）—— 也常见
    # 旧实现只搜 d >= 0，而且按 n_e 切片；一旦 est 比 gt 多，两个切片长度不等，
    # numpy 直接 broadcast 报错（真机上第一次跑就撞上了）。
    MAX_LAG = 25                       # ±2.5 s @10 Hz，足够覆盖暖机
    max_lag = min(max(0, n_g - n_e), MAX_LAG)
    min_lag = -min(max(0, n_e - n_g), MAX_LAG)

    def _score(lag):
        if lag >= 0:
            g, e = gt_t[lag:], es_t[:n_g - lag]
        else:
            g, e = gt_t[:n_g + lag], es_t[-lag:]
        m = min(len(g), len(e))
        if m <= 0:
            return -1.0
        g, e = g[:m], e[:m]
        ok = (np.floor(g + 1e-6) - np.floor(e + 1e-6)) == 0
        ok |= np.abs(g - e) < 2e-3          # 时间戳哪天修准了也认
        return float(np.mean(ok))

    best = (-1.0, 0)
    for lag in range(min_lag, max_lag + 1):
        frac = _score(lag)
        if frac > best[0] + 1e-12:
            best = (frac, lag)
        if frac >= 0.999:                   # 同等分数下优先取 |lag| 小的
            if abs(lag) < abs(best[1]) or best[0] < 0.999:
                best = (frac, lag)
            break

    frac, lag = best
    if lag >= 0:
        gi = np.arange(lag, n_g, dtype=int)
        ei = np.arange(0, n_g - lag, dtype=int)
    else:
        gi = np.arange(0, n_g + lag, dtype=int)
        ei = np.arange(-lag, n_e, dtype=int)
    m = min(len(gi), len(ei))
    gi, ei = gi[:m], ei[:m]
    g, e = gt_t[gi], es_t[ei]
    ok = ((np.floor(g + 1e-6) - np.floor(e + 1e-6)) == 0) | (np.abs(g - e) < 2e-3)
    gi, ei = gi[ok], ei[ok]                 # 只保留同秒的那些
    dt = np.abs(gt_t[gi] - es_t[ei]) if gi.size else np.array([])
    info = {
        'mode': 'floor',
        'lag': int(lag),
        'matched': int(gi.size),
        'match_fraction': float(gi.size) / float(n_e),
        'lag_candidates': int(max_lag - min_lag + 1),
        'score_at_best_lag': round(float(frac), 6),
        'median_dt_ms': float(np.median(dt) * 1e3) if dt.size else None,
        'max_dt_ms': float(np.max(dt) * 1e3) if dt.size else None,
    }
    if gi.size < n_e:
        info['note'] = (f'{n_e - gi.size} 帧没有落在同一整数秒（常见于收尾/暖机），已丢弃')
    return gi, ei, info


# ================================================================ 单次评测
def evaluate_run(csv_path, rpe_delta=1.0, mode=None):
    data = load_traj(csv_path)
    meta = load_meta(csv_path)
    algo = meta.get('algo', os.path.splitext(os.path.basename(csv_path))[0])

    gt_t, gt_p, gt_q = data['gt']['t'], data['gt']['p'], data['gt']['q']
    es_t, es_p, es_q = data['est']['t'], data['est']['p'], data['est']['q']

    result = {
        'csv': os.path.abspath(csv_path),
        'algo': algo,
        'est_topic': meta.get('est_topic'),
        'counts': {'gt': int(len(gt_t)), 'est': int(len(es_t))},
        'warnings': [],
    }
    if len(gt_t) == 0 or len(es_t) == 0:
        result['warnings'].append('gt 或 est 为空，无法评测')
        result['ok'] = False
        return result

    # ---------------------------------------------------- 关联
    if mode is None:
        # 有 5% 以上的 est 时间戳不是整数秒，就认为时间戳可信
        frac_int = float(np.mean(np.abs(es_t - np.round(es_t)) < 1e-6))
        mode = 'floor' if frac_int > 0.95 else 'exact'
    gi, ei, assoc = associate(gt_t, es_t, mode)
    result['association'] = assoc
    if gi.size < 10:
        result['warnings'].append(
            f'只有 {gi.size} 帧关联成功（mode={mode}）—— 时间戳/话题对不上，结果不可信')
        result['ok'] = False
        return result
    if assoc.get('match_fraction', 0.0) < 0.9:
        result['warnings'].append(
            f'关联合格率只有 {assoc.get("match_fraction", 0):.1%}，ATE 会偏乐观/偏悲观')

    P = es_p[ei]                     # 估计
    Q = gt_p[gi]                     # 真值
    RP = np.array([quat_to_R(q) for q in es_q[ei]])
    RQ = np.array([quat_to_R(q) for q in gt_q[gi]])
    t_rel = gt_t[gi] - gt_t[gi][0]

    # ---------------------------------------------------- ATE
    R_al, t_al = umeyama_align(P, Q)
    P_al = (R_al @ P.T).T + t_al
    err = np.linalg.norm(P_al - Q, axis=1)

    result['ate'] = stats(err)
    result['ate']['rmse_2d_m'] = float(np.sqrt(np.mean(
        np.sum((P_al[:, :2] - Q[:, :2]) ** 2, axis=1))))
    result['raw_before_alignment'] = stats(np.linalg.norm(P - Q, axis=1))

    rot_err = np.array([R_to_rotvec_deg(RQ[i].T @ (R_al @ RP[i]))
                        for i in range(len(P))])
    result['ate_rot_deg'] = stats(rot_err)

    # ---------------------------------------------------- 终点误差（对齐后）
    last = -1
    dxy = P_al[last, :2] - Q[last, :2]
    yaw_err = wrap_deg(math.degrees(yaw_from_R(R_al @ RP[last]))
                       - math.degrees(yaw_from_R(RQ[last])))
    result['final_error'] = {
        'pos_m': float(np.linalg.norm(P_al[last] - Q[last])),
        'pos_xy_m': float(np.linalg.norm(dxy)),
        'pos_z_m': float(P_al[last, 2] - Q[last, 2]),
        'yaw_deg': float(yaw_err),
    }

    # ---------------------------------------------------- RPE
    if len(t_rel) >= 3:
        dt_med = float(np.median(np.diff(t_rel)))
        step = max(1, int(round(float(rpe_delta) / dt_med))) if dt_med > 0 else 1
        trans_drift, rot_drift, dists = [], [], []
        for i in range(0, len(P) - step):
            j = i + step
            Qr = RQ[i].T @ RQ[j]
            tr = RQ[i].T @ (Q[j] - Q[i])
            Pr = RP[i].T @ RP[j]
            te = RP[i].T @ (P[j] - P[i])
            Er = Qr.T @ Pr
            et = Qr.T @ (te - tr)
            d = float(np.linalg.norm(tr))
            if d < 1e-3:
                continue
            dists.append(d)
            trans_drift.append(float(np.linalg.norm(et)) / d * 100.0)
            rot_drift.append(R_to_rotvec_deg(Er) / d)
        result['rpe'] = {
            'delta_s': float(rpe_delta),
            'frame_step': int(step),
            'n_pairs': int(len(dists)),
            'trans_pct_per_m': float(np.mean(trans_drift)) if dists else None,
            'trans_pct_per_m_median': float(np.median(trans_drift)) if dists else None,
            'rot_deg_per_m': float(np.mean(rot_drift)) if dists else None,
            'rot_deg_per_m_median': float(np.median(rot_drift)) if dists else None,
        }
    else:
        result['rpe'] = {}

    # ---------------------------------------------------- 轨迹尺度 / 鲁棒性
    gt_path_len = float(np.sum(np.linalg.norm(np.diff(gt_p[gi], axis=0), axis=1)))
    est_path_len = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
    result['path'] = {
        'gt_length_m': gt_path_len,
        'est_length_m': est_path_len,
        'length_ratio': (est_path_len / gt_path_len) if gt_path_len > 1e-6 else None,
        'duration_s': float(t_rel[-1]),
    }
    result['robustness'] = {
        'frames_over_0p5m': int(np.sum(err > 0.5)),
        'frac_over_0p5m': float(np.mean(err > 0.5)),
        'frames_over_2m': int(np.sum(err > 2.0)),
        'diverged': bool(np.max(err) > 5.0),
    }
    if gt_path_len > 1e-6:
        result['robustness']['drift_pct_of_path'] = float(
            err[-1] / gt_path_len * 100.0)

    # ---------------------------------------------------- 输出频率 / 时间戳
    now = data['est']['now']
    if len(now) >= 3 and now[-1] > now[0]:
        result['output_rate_hz'] = float((len(now) - 1) / (now[-1] - now[0]))
    diffs = np.diff(es_t)
    diffs = diffs[diffs > 0]
    result['stamp_resolution_s'] = (
        float(np.median(diffs)) if diffs.size else None)
    lat = now - es_t
    if lat.size:
        result['latency'] = {
            'mean_ms': float(np.mean(lat) * 1e3),
            'median_ms': float(np.median(lat) * 1e3),
            # A-LOAM 的时间戳被截断到整秒，latency 会整体偏大 0~1000 ms
            'trustworthy': bool(result.get('stamp_resolution_s') is not None
                                and result['stamp_resolution_s'] < 0.05),
        }

    result['ok'] = True
    return result


# ================================================================ 画图用数组
def aligned_arrays(csv_path, mode=None):
    """
    返回画图需要的原始数组（不做统计）。和 evaluate_run 用同一套关联与对齐，
    保证"图上的曲线"和"表里的数字"是同一件事。
    """
    data = load_traj(csv_path)
    gt_t, gt_p, gt_q = data['gt']['t'], data['gt']['p'], data['gt']['q']
    es_t, es_p, es_q = data['est']['t'], data['est']['p'], data['est']['q']
    if len(gt_t) == 0 or len(es_t) == 0:
        return None
    if mode is None:
        frac_int = float(np.mean(np.abs(es_t - np.round(es_t)) < 1e-6))
        mode = 'floor' if frac_int > 0.95 else 'exact'
    gi, ei, _info = associate(gt_t, es_t, mode)
    if gi.size < 2:
        return None
    P = es_p[ei]
    Q = gt_p[gi]
    R_al, t_al = umeyama_align(P, Q)
    P_al = (R_al @ P.T).T + t_al
    err = np.linalg.norm(P_al - Q, axis=1)
    meta = load_meta(csv_path)
    return {
        'algo': meta.get('algo', os.path.splitext(os.path.basename(csv_path))[0]),
        'csv': os.path.abspath(csv_path),
        't': gt_t[gi] - gt_t[gi][0],
        'gt': Q,
        'est_aligned': P_al,
        'err': err,
    }


# ================================================================ 输出
def fmt(v, nd=3):
    if v is None:
        return '-'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return '-'
        return f'{v:.{nd}f}'
    return str(v)


TABLE_COLUMNS = [
    ('algorithm', 'Algorithm', lambda r: r.get('algo', '?'), None),
    ('frames', 'Frames', lambda r: r.get('counts', {}).get('est'), None),
    ('duration_s', 'Duration (s)', lambda r: r.get('path', {}).get('duration_s'), 1),
    ('ate_rmse_m', 'ATE RMSE (m)', lambda r: r.get('ate', {}).get('rmse'), 3),
    ('ate_mean_m', 'ATE mean (m)', lambda r: r.get('ate', {}).get('mean'), 3),
    ('ate_median_m', 'ATE median (m)', lambda r: r.get('ate', {}).get('median'), 3),
    ('ate_max_m', 'ATE max (m)', lambda r: r.get('ate', {}).get('max'), 3),
    ('ate_rot_rmse_deg', 'ATE rot RMSE (deg)',
     lambda r: r.get('ate_rot_deg', {}).get('rmse'), 2),
    ('raw_rmse_m', 'Raw RMSE (m)',
     lambda r: r.get('raw_before_alignment', {}).get('rmse'), 3),
    ('rpe_trans_pct_m', 'RPE trans (%/m)',
     lambda r: r.get('rpe', {}).get('trans_pct_per_m'), 2),
    ('rpe_rot_deg_m', 'RPE rot (deg/m)',
     lambda r: r.get('rpe', {}).get('rot_deg_per_m'), 2),
    ('final_pos_err_m', 'Final pos err (m)',
     lambda r: r.get('final_error', {}).get('pos_m'), 3),
    ('final_yaw_err_deg', 'Final yaw err (deg)',
     lambda r: r.get('final_error', {}).get('yaw_deg'), 2),
    ('frames_over_0p5m', 'Frames >0.5m',
     lambda r: r.get('robustness', {}).get('frames_over_0p5m'), None),
    ('diverged', 'Diverged', lambda r: r.get('robustness', {}).get('diverged'), None),
    ('rate_hz', 'Rate (Hz)', lambda r: r.get('output_rate_hz'), 1),
]


def md_table(results):
    head = '| ' + ' | '.join(c[1] for c in TABLE_COLUMNS) + ' |'
    sep = '|' + '|'.join(['---'] * len(TABLE_COLUMNS)) + '|'
    lines = [head, sep]
    for r in results:
        lines.append('| ' + ' | '.join(fmt(c[2](r), 3 if c[3] is None else c[3])
                                      for c in TABLE_COLUMNS) + ' |')
    return '\n'.join(lines)


def csv_table(results, path):
    """和 Markdown 表同一批列，写成 CSV —— 给画图和 Excel 用。"""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow([c[0] for c in TABLE_COLUMNS] + ['csv'])
        for r in results:
            row = []
            for _key, _label, getter, _nd in TABLE_COLUMNS:
                v = getter(r)
                if v is None:
                    row.append('')
                elif isinstance(v, bool):
                    row.append('true' if v else 'false')
                elif isinstance(v, (int, float)):
                    row.append(repr(float(v)))
                else:
                    row.append(str(v))
            row.append(r.get('csv', ''))
            w.writerow(row)


def print_summary(r):
    a = r.get('ate', {})
    print(f'=== {r["algo"]} ===')
    print(f'  csv            : {r["csv"]}')
    if not r.get('ok'):
        for w in r.get('warnings', []):
            print(f'  [FAIL] {w}')
        return
    assoc = r['association']
    print(f'  association    : mode={assoc["mode"]} matched={assoc["matched"]} '
          f'({assoc.get("match_fraction", 0):.1%})')
    print(f'  ATE  RMSE/mean/max : {fmt(a.get("rmse"))} / {fmt(a.get("mean"))}'
          f' / {fmt(a.get("max"))} m')
    print(f'  ATE  rot RMSE  : {fmt(r.get("ate_rot_deg", {}).get("rmse"), 2)} deg')
    print(f'  RPE  trans     : {fmt(r.get("rpe", {}).get("trans_pct_per_m"), 2)} %/m')
    print(f'  RPE  rot       : {fmt(r.get("rpe", {}).get("rot_deg_per_m"), 2)} deg/m')
    fe = r.get('final_error', {})
    print(f'  final pos/yaw  : {fmt(fe.get("pos_m"))} m / {fmt(fe.get("yaw_deg"), 2)} deg')
    ro = r.get('robustness', {})
    print(f'  frames >0.5 m  : {fmt(ro.get("frames_over_0p5m"))} '
          f'({fmt(ro.get("frac_over_0p5m", 0) * 100, 1)}%)   diverged={ro.get("diverged")}')
    print(f'  output rate    : {fmt(r.get("output_rate_hz"), 1)} Hz  '
          f'(stamp res {fmt(r.get("stamp_resolution_s"), 3)} s)')
    for w in r.get('warnings', []):
        print(f'  [warn] {w}')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='LOAM / LIO-SAM 轨迹评测（ATE / RPE / 终点 / 鲁棒性）')
    ap.add_argument('--csv', nargs='+', required=True,
                    help='traj_recorder.py 写出的 CSV（可多个，合成一张表）')
    ap.add_argument('--rpe-delta', type=float, default=1.0,
                    help='RPE 的时间间隔（秒），默认 1.0')
    ap.add_argument('--mode', choices=['auto', 'exact', 'floor'], default='auto',
                    help='时间戳关联方式；auto = 按时间戳分辨率自动判断')
    ap.add_argument('--json-out', default=None,
                    help='把结果写成 JSON（单个 CSV 时有效）')
    ap.add_argument('--md-out', default=None,
                    help='把结果写成 Markdown 表')
    ap.add_argument('--csv-out', default=None,
                    help='把结果写成 CSV（画图 / Excel 用）')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)

    mode = None if args.mode == 'auto' else args.mode
    results = [evaluate_run(p, rpe_delta=args.rpe_delta, mode=mode)
               for p in args.csv]

    for r in results:
        if not args.quiet:
            print_summary(r)
            print()

    if args.md_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.md_out)), exist_ok=True)
        with open(args.md_out, 'w', encoding='utf-8') as f:
            f.write(md_table(results) + '\n')
        if not args.quiet:
            print(f'[ok] markdown table -> {args.md_out}')

    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
        payload = results[0] if len(results) == 1 else results
        with open(args.json_out, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        if not args.quiet:
            print(f'[ok] json -> {args.json_out}')

    if args.csv_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv_out)), exist_ok=True)
        csv_table(results, args.csv_out)
        if not args.quiet:
            print(f'[ok] csv -> {args.csv_out}')

    if not args.quiet:
        print(md_table(results))

    bad = [r for r in results if not r.get('ok')]
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
