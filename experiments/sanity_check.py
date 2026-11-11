#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sanity_check.py —— 拿到实验数据后，先判断"这组数据可信吗"
===============================================================================
`evaluate.py` 负责**算**指标，这个脚本负责**怀疑**这些指标。

两者分开是有意的：评测器只做数学，不做判断；而"LIO-SAM 的 ATE 居然比 LOAM 差"
这类结果，可能是真的发现，也可能是录错了话题、关联错帧、算法中途挂了。
先过一遍这里，能避免把一次失误写成结论。

    python3 experiments/sanity_check.py results/tables/all_runs.csv
    python3 experiments/sanity_check.py results/tables/all_runs.csv --json-dir results/json

退出码：0 = 可以拿去写报告（可能有 WARN）；1 = 有 FAIL，先别写。

判据分三档
----------
  FAIL  数据本身不成立（帧数太少、指标全 0、两条算法数字一模一样 …）
  WARN  结果可疑但仍可能是真的（单帧爆掉、关联率低、LIO-SAM 反而更差 …）
  INFO  只是需要知道（发散了、两侧帧数差很多 …）
"""

import argparse
import csv
import glob
import json
import os
import sys

MIN_FRAMES = 50
MIN_DURATION_S = 30.0
MAX_OVER_RMSE_RATIO = 25.0     # ATE max / ATE rmse 超过这个值，说明是单帧爆掉
MIN_MATCH_FRACTION = 0.90
# 真值轨迹总长下界：低于这个值就是"车基本没动"，那种数据的 ATE 没有意义
# （两条算法都会显得"极准"，因为根本没走几米）。所有场景都远大于 3 m。
MIN_GT_PATH_M = 3.0
# 估计轨迹 / 真值轨迹 的长度比合理区间（超出就是尺度出了问题）
MIN_LENGTH_RATIO = 0.5
MAX_LENGTH_RATIO = 2.0

# 这些场景里，纯几何方法在物理上就吃亏（畸变 / 退化 / 稀疏 / 长时漂移）
EXPECT_LIO_BETTER = {'s0_nominal', 's1_fast_yaw', 's2_high_speed',
                     's3_corridor', 's4_sparse_hall', 's5_long_run'}


def _f(v):
    try:
        return None if v in ('', None) else float(v)
    except (TypeError, ValueError):
        return None


def load_table(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            scene = os.path.basename(os.path.dirname((row.get('csv') or '').replace('\\', '/')))
            rows.append({
                'scene': scene,
                'algo': (row.get('algorithm') or '').lower(),
                'frames': _f(row.get('frames')),
                'duration': _f(row.get('duration_s')),
                'ate': _f(row.get('ate_rmse_m')),
                'ate_max': _f(row.get('ate_max_m')),
                'rpe_t': _f(row.get('rpe_trans_pct_m')),
                'rpe_r': _f(row.get('rpe_rot_deg_m')),
                'final_pos': _f(row.get('final_pos_err_m')),
                'rate': _f(row.get('rate_hz')),
                'diverged': (row.get('diverged') or '').strip().lower() in ('true', '1', 'yes'),
            })
    return rows


def load_json_checks(json_dir):
    """从 evaluate.py 的 json 里取关联质量（表里没有这两列）。"""
    out = {}
    if not json_dir or not os.path.isdir(json_dir):
        return out
    for path in glob.glob(os.path.join(json_dir, '*.json')):
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except Exception:                                     # noqa: BLE001
            continue
        for r in (data if isinstance(data, list) else [data]):
            scene = os.path.basename(os.path.dirname((r.get('csv') or '').replace('\\', '/')))
            key = (scene, str(r.get('algo', '')).lower())
            out[key] = {
                'match_fraction': (r.get('association') or {}).get('match_fraction'),
                'mode': (r.get('association') or {}).get('mode'),
                'gt_frames': (r.get('counts') or {}).get('gt'),
                'est_frames': (r.get('counts') or {}).get('est'),
                'warnings': r.get('warnings') or [],
                # 轨迹长度（表里没有，但它是"车到底动没动"的唯一证据）
                'gt_length_m': (r.get('path') or {}).get('gt_length_m'),
                'est_length_m': (r.get('path') or {}).get('est_length_m'),
                'length_ratio': (r.get('path') or {}).get('length_ratio'),
                # 回放起点：两条算法不一致 = 看的不是同一段数据
                'bag_start_offset': r.get('bag_start_offset'),
            }
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='实验数据可信度检查')
    ap.add_argument('table', help='results/tables/all_runs.csv')
    ap.add_argument('--json-dir', default=None, help='results/json（有的话会检查关联率）')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args(argv)

    if not os.path.isfile(args.table):
        print(f'[FAIL] 找不到 {args.table}（先跑 bash experiments/run_bench.sh）')
        return 1

    rows = load_table(args.table)
    if not rows:
        print(f'[FAIL] {args.table} 是空的')
        return 1
    extra = load_json_checks(args.json_dir)

    fails, warns, infos = [], [], []

    def emit(kind, msg):
        print(f'  [{kind}] {msg}')
        {'FAIL': fails, 'WARN': warns, 'INFO': infos}[kind].append(msg)

    print(f'[sanity] {args.table}：{len(rows)} 行，'
          f'{len({r["scene"] for r in rows})} 个场景\n')

    for r in rows:
        tag = f'{r["scene"] or "?"}/{r["algo"] or "?"}'
        if r['frames'] is None or r['frames'] < MIN_FRAMES:
            emit('FAIL', f'{tag}: 只有 {r["frames"]} 帧位姿 —— 算法中途挂了？')
        if r['ate'] is None:
            emit('FAIL', f'{tag}: ATE 缺失/NaN —— 关联失败，看 replay 日志')
        if r['duration'] is not None and r['duration'] < MIN_DURATION_S:
            emit('WARN', f'{tag}: 时长只有 {r["duration"]:.0f}s —— 录制被截断？')
        if r['ate'] is not None and r['ate_max'] is not None and r['ate'] > 1e-9:
            ratio = r['ate_max'] / r['ate']
            if ratio > MAX_OVER_RMSE_RATIO:
                emit('WARN', f'{tag}: ATE max / RMSE = {ratio:.0f} —— 单帧爆掉，'
                             f'不是整段漂移（看一眼误差曲线）')
        if r['rpe_t'] is not None and r['rpe_t'] == 0.0 and r['ate'] not in (None, 0.0):
            emit('FAIL', f'{tag}: RPE 平移恰好为 0 但 ATE 不为 0 —— 相对位姿算错了')
        if r['diverged']:
            emit('INFO', f'{tag}: 判为发散（ATE max {r["ate_max"]:.1f} m）—— 这是结果，不是错误')
        info = extra.get((r['scene'], r['algo']))
        if info:
            mf = info.get('match_fraction')
            if mf is not None and mf < MIN_MATCH_FRACTION:
                emit('WARN', f'{tag}: 关联合格率只有 {mf:.0%}（mode={info.get("mode")}）'
                             f'—— 时间戳/话题可疑，指标会偏')
            for w in info.get('warnings') or []:
                emit('WARN', f'{tag}: {w}')
            # 车到底动没动 —— 这是"包里数据对、但机器人原地没走"的唯一证据。
            # 那种数据 ATE 会小得离谱（两条算法都"很准"），不查就会得出
            # "两个算法都完美"的错误结论。
            gl = info.get('gt_length_m')
            if gl is not None and gl < MIN_GT_PATH_M:
                emit('FAIL', f'{tag}: 真值轨迹总长只有 {gl:.2f} m —— '
                             f'车基本没动（驱动没生效？世界没加载？）'
                             f'这批数据不能用，先看 record 日志里 scripted_driver 有没有报错')
            lr = info.get('length_ratio')
            if lr is not None and (lr < MIN_LENGTH_RATIO or lr > MAX_LENGTH_RATIO):
                emit('WARN', f'{tag}: 轨迹长度比 {lr:.2f}（估计/真值）—— '
                             f'尺度明显不对，可能是纯里程计尺度漂移或关联错帧')

    # ------------------------------------------------------------ 成对检查
    by_scene = {}
    for r in rows:
        by_scene.setdefault(r['scene'], {})[r['algo']] = r

    print('\n  --- 成对比较 ---')
    for scene in sorted(by_scene):
        pair = by_scene[scene]
        if 'loam' not in pair or 'liosam' not in pair:
            print(f'  [INFO] {scene}: 只有 {"/".join(pair)}，没法成对比较')
            continue
        lo, li = pair['loam'], pair['liosam']
        # 公平性硬条件：两条算法必须从包里的**同一时刻**开始回放
        off_lo = (extra.get((scene, 'loam')) or {}).get('bag_start_offset')
        off_li = (extra.get((scene, 'liosam')) or {}).get('bag_start_offset')
        if off_lo is not None and off_li is not None and abs(off_lo - off_li) > 1e-6:
            emit('FAIL', f'{scene}: 两条算法的回放起点不同（LOAM {off_lo}s / '
                         f'LIO-SAM {off_li}s）—— 它们看的不是同一段数据，不能直接比较。'
                         f'用同一个 --bag-start-offset 重跑两条算法')
        if lo['ate'] and li['ate']:
            imp = (lo['ate'] - li['ate']) / lo['ate'] * 100.0
            line = f'{scene}: LOAM {lo["ate"]:.3f} m -> LIO-SAM {li["ate"]:.3f} m ({imp:+.0f}%)'
            if imp < 0 and scene in EXPECT_LIO_BETTER:
                emit('WARN', line + '  ← LIO-SAM 反而更差。可能是真发现，'
                                    '也可能录错话题/关联错帧，先查 replay 日志')
            else:
                print(f'  [ OK ] {line}')
        # 两条算法的数字完全一样 = 几乎肯定录到了同一个话题
        same = all(lo[k] is not None and lo[k] == li[k]
                   for k in ('ate', 'ate_max', 'rpe_t', 'final_pos'))
        if same:
            emit('FAIL', f'{scene}: 两条算法的指标完全相同 —— 大概录到了同一个话题'
                         f'（检查 bench_replay.launch.py 顶部的 EST_TOPIC）')

    # ---------------------------------------------------------------- 结论
    print('\n[sanity] 汇总：'
          f'{len(fails)} FAIL / {len(warns)} WARN / {len(infos)} INFO')
    if fails:
        print('[sanity] 先修 FAIL 再写报告 —— 这些数据现在还不能当结论用')
        return 1
    if warns:
        print('[sanity] 没有硬伤，但 WARN 项要在报告里解释清楚')
    else:
        print('[sanity] 数据可用')
    return 0


if __name__ == '__main__':
    sys.exit(main())
