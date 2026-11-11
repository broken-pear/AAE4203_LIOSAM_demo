#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_test_metrics.py —— 在没有 ROS、没有仿真器的机器上验证评测器本身是对的
===============================================================================
评测器是整篇报告的数字来源，所以它必须能被单独验证。这里用"已知答案"的
合成轨迹去做断言：

  1. 完美估计（est == gt）            -> ATE / RPE 必须 ≈ 0
  2. A-LOAM 式整数秒时间戳            -> floor 关联必须找回同一个 lag、全部命中
  3. 暖机帧（gt 比 est 多 5 帧）       -> lag 必须等于 5
  4. 已知的尺度/偏航误差              -> ATE 必须落在可解析估算的范围内
  5. LIO-SAM 式精确时间戳             -> exact 关联必须全部命中
  6. 发散轨迹                         -> 必须被判为 diverged

用法： python3 setup/_test_metrics.py        （只需要 numpy）
"""

import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_bench', 'scripts'))

import evaluate as ev                                    # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if cond:
        print(f'  [ OK ] {name} {detail}')
    else:
        FAILS.append(name)
        print(f'  [FAIL] {name} {detail}')


# --------------------------------------------------------------------- 合成轨迹
def circle_traj(n, radius=1.25, omega=0.32, t0=12.0, dt=0.1):
    """和 demo 里 patrol_driver 一样的匀速圆周：r = v/omega = 1.25 m。"""
    ts, ps, qs = [], [], []
    yaw0 = 0.0
    for i in range(n):
        th = omega * i * dt
        # 车沿圆周走，位姿是 (cx + r sin, cy - r cos)，朝切线方向
        cx, cy = 2.5, 1.25
        x = cx + radius * math.sin(th)
        y = cy - radius * math.cos(th)
        yaw = th + yaw0
        ts.append(t0 + i * dt)
        ps.append([x, y, 0.40])
        qs.append([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return np.array(ts), np.array(ps), np.array(qs)


def write_csv(path, gt, est, algo, est_truncate_int=False, gt_lead=0):
    """按 traj_recorder.py 的格式写 CSV。gt_lead = gt 比 est 多的开头帧数。"""
    gt_t, gt_p, gt_q = gt
    es_t, es_p, es_q = est
    with open(path, 'w', encoding='utf-8') as f:
        f.write('kind,stamp,node_now,x,y,z,qx,qy,qz,qw\n')
        for i in range(len(gt_t)):
            f.write(f'gt,{gt_t[i]:.9f},{gt_t[i]:.9f},'
                    f'{gt_p[i][0]:.6f},{gt_p[i][1]:.6f},{gt_p[i][2]:.6f},'
                    f'{gt_q[i][0]:.7f},{gt_q[i][1]:.7f},'
                    f'{gt_q[i][2]:.7f},{gt_q[i][3]:.7f}\n')
        for i in range(len(es_t)):
            t = math.floor(es_t[i]) if est_truncate_int else es_t[i]
            f.write(f'est,{t:.9f},{es_t[i] + 0.02:.9f},'
                    f'{es_p[i][0]:.6f},{es_p[i][1]:.6f},{es_p[i][2]:.6f},'
                    f'{es_q[i][0]:.7f},{es_q[i][1]:.7f},'
                    f'{es_q[i][2]:.7f},{es_q[i][3]:.7f}\n')
    with open(os.path.splitext(path)[0] + '.meta.json', 'w', encoding='utf-8') as f:
        import json
        json.dump({'algo': algo, 'est_topic': '/x'}, f)


def main():
    # 产物写在仓库内的临时目录（不是系统 /tmp）：这样任何沙箱/权限策略下都能跑，
    # 也方便出问题时留下来看。跑完自动清掉。
    tmp = os.path.join(ROOT, '.bench-test-tmp')
    os.makedirs(tmp, exist_ok=True)
    n = 200

    print('=== 1. 完美估计（est == gt, 精确时间戳, LIO-SAM 式） ===')
    gt = circle_traj(n)
    p_perfect = os.path.join(tmp, 'perfect.csv')
    write_csv(p_perfect, gt, gt, 'liosam')
    r = ev.evaluate_run(p_perfect)
    check('关联全部命中', r['association']['matched'] == n,
          f"matched={r['association']['matched']}/{n}")
    check('ATE RMSE < 1e-6', r['ate']['rmse'] < 1e-6, f"{r['ate']['rmse']:.2e}")
    check('RPE 平移 < 1e-6 %/m', r['rpe']['trans_pct_per_m'] < 1e-6,
          f"{r['rpe']['trans_pct_per_m']:.2e}")
    check('未发散', r['robustness']['diverged'] is False)

    print('\n=== 2. A-LOAM 式整数秒时间戳（floor 关联） ===')
    p_loam = os.path.join(tmp, 'loam_perfect.csv')
    write_csv(p_loam, gt, gt, 'loam', est_truncate_int=True)
    r = ev.evaluate_run(p_loam)
    check('自动判定为 floor 模式', r['association']['mode'] == 'floor',
          r['association']['mode'])
    check('关联全部命中', r['association']['matched'] == n,
          f"matched={r['association']['matched']}/{n}")
    check('ATE RMSE < 1e-6（时间戳截断不应引入误差）',
          r['ate']['rmse'] < 1e-6, f"{r['ate']['rmse']:.2e}")

    print('\n=== 3. 暖机帧：gt 比 est 多 5 帧 ===')
    gt5 = circle_traj(n + 5)
    est5 = tuple(a[5:] for a in gt5)
    p_warm = os.path.join(tmp, 'warmup.csv')
    write_csv(p_warm, gt5, est5, 'loam', est_truncate_int=True)
    r = ev.evaluate_run(p_warm)
    check('找回 lag = 5', r['association'].get('lag') == 5,
          f"lag={r['association'].get('lag')}")
    check('关联数 = 200（gt 205 帧里的后 200 帧）',
          r['association']['matched'] == n,
          f"matched={r['association']['matched']}")
    check('ATE RMSE < 1e-6', r['ate']['rmse'] < 1e-6, f"{r['ate']['rmse']:.2e}")

    print('\n=== 4. 已知误差：位置整体放大 2% + 每帧多转 0.03 度（累积偏航漂移） ===')
    es_t, es_p, es_q = est5
    scale = 1.02
    c = np.array([2.5, 1.25, 0.0])
    p_bad = es_p.copy()
    p_bad[:, :2] = c[:2] + (p_bad[:, :2] - c[:2]) * scale
    # 累积偏航漂移：第 i 帧额外多转 i*0.03 度 —— 只有这样才会体现为 RPE 旋转漂移
    # （给每帧加一个**常量**偏航只是整体转了一下，相对旋转不变，RPE 旋转仍为 0）
    q_bad = []
    for i, q in enumerate(es_q):
        yaw = 2 * math.atan2(q[2], q[3]) + math.radians(0.03) * i
        q_bad.append([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    q_bad = np.array(q_bad)
    p_drift = os.path.join(tmp, 'drift.csv')
    write_csv(p_drift, gt5, (es_t, p_bad, q_bad), 'loam', est_truncate_int=True)
    r = ev.evaluate_run(p_drift)
    # 半径 1.25 m 放大 2% -> 横向 2.5 cm 量级的形变；累积偏航漂移再加一点
    check('ATE RMSE 落在 0.005~0.20 m', 0.005 < r['ate']['rmse'] < 0.20,
          f"rmse={r['ate']['rmse']:.4f} m")
    check('RPE 平移 > 0.1 %/m', r['rpe']['trans_pct_per_m'] > 0.1,
          f"{r['rpe']['trans_pct_per_m']:.3f} %/m")
    check('RPE 旋转 > 0.05 deg/m（累积偏航漂移被抓到）',
          r['rpe']['rot_deg_per_m'] > 0.05,
          f"{r['rpe']['rot_deg_per_m']:.3f} deg/m")

    print('\n=== 5. 精确时间戳 + 少量掉帧（exact 关联） ===')
    idx = np.arange(n + 5)
    keep = idx[idx % 7 != 0]                 # 每 7 帧掉 1 帧
    gt_missing = tuple(a[keep] for a in gt5)
    est_missing = tuple(a[keep] for a in gt5)
    p_missing = os.path.join(tmp, 'missing.csv')
    write_csv(p_missing, gt_missing, est_missing, 'liosam')
    r = ev.evaluate_run(p_missing)
    check('exact 模式', r['association']['mode'] == 'exact',
          r['association']['mode'])
    check('关联数 = 掉帧后帧数', r['association']['matched'] == len(keep),
          f"matched={r['association']['matched']} / {len(keep)}")
    check('ATE RMSE < 1e-6', r['ate']['rmse'] < 1e-6, f"{r['ate']['rmse']:.2e}")

    print('\n=== 6. 发散轨迹（第 100 帧后彻底跑偏） ===')
    es_t2, es_p2, es_q2 = est5
    p_div = es_p2.copy()
    p_div[100:, 0] += np.linspace(0, 30, len(p_div[100:]))
    p_diverged = os.path.join(tmp, 'diverged.csv')
    write_csv(p_diverged, gt5, (es_t2, p_div, es_q2), 'loam',
              est_truncate_int=True)
    r = ev.evaluate_run(p_diverged)
    check('判为发散', r['robustness']['diverged'] is True)
    check('ATE max > 5 m', r['ate']['max'] > 5.0, f"max={r['ate']['max']:.2f} m")

    print('\n=== 7. Markdown 表 ===')
    md = ev.md_table([ev.evaluate_run(p_perfect), ev.evaluate_run(p_warm)])
    lines = md.strip().split('\n')
    check('表 = 表头 + 分隔行 + 2 数据行',
          len(lines) == 4 and lines[0].startswith('| Algorithm'), f'{len(lines)} 行')
    check('两行分别是两种算法', 'liosam' in lines[2] and 'loam' in lines[3])
    print('\n' + md)

    print('\n=== 结论 ===')
    if FAILS:
        print(f'{len(FAILS)} FAILED: {FAILS}')
        return 1
    print('ALL PASS —— 评测器在合成数据上给出的是解析可预期的数字')
    # 全部通过才清理
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
