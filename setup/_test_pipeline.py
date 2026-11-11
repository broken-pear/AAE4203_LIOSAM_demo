#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_test_pipeline.py —— 把"录制后的报告链"从头跑一遍，不需要 ROS、不需要 Gazebo
===============================================================================

在 Jetson 上跑实验之前，先证明**数据变成报告的那一段**是对的：

    evaluate.py   →  results/tables/all_runs.csv + json + md
    make_figures.py → results/figures/*.png
    build_deck.py →  PPTX（数字已经从 CSV 填进去）

做法：造一批**形状与 traj_recorder.py 完全相同**的 CSV（kind,stamp,node_now,
x,y,z,qx,qy,qz,qw），其中真值是真轨迹、估计是"真值 + 已知漂移"，这样每个指标
都能对上解析值。然后依次调用三个脚本，检查产物。

几个刻意设计的点：
  * 一个场景用 A-LOAM 那样**只有整数秒**的 est 时间戳（floor 关联路径）
  * 一个场景用 LIO-SAM 那样精确的时间戳（exact 关联路径）
  * 故意让其中一个场景 LIO-SAM 更差，检查 PPT 会不会照实写 "LIO-SAM worse"
    （如果它美化成 "clear win"，说明结论又被写死了）
  * matplotlib 用桩模块注入（PYTHONPATH），这样画图代码的**逻辑**也被执行到，
    但不需要真的装 matplotlib

用法：
    python3 setup/_test_pipeline.py            # 全部跑完并清理
    python3 setup/_test_pipeline.py --keep     # 保留 .pipeline-test/ 供检查
"""

import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BENCH = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_bench', 'scripts')
WORK = os.path.join(ROOT, '.pipeline-test')

FAILS = []


def check(name, cond, detail=''):
    if cond:
        print(f'  [ OK ] {name} {detail}')
    else:
        FAILS.append(name)
        print(f'  [FAIL] {name} {detail}')


# --------------------------------------------------------------------- 合成数据
def circle(n=200, radius=1.25, omega=0.32, t0=12.0, dt=0.1):
    ts, ps, qs = [], [], []
    for i in range(n):
        th = omega * i * dt
        x = 2.5 + radius * math.sin(th)
        y = 1.25 - radius * math.cos(th)
        yaw = th
        ts.append(t0 + i * dt)
        ps.append([x, y, 0.40])
        qs.append([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return np.array(ts), np.array(ps), np.array(qs)


def distorts(ps, qs, scale=1.0, yaw_drift_deg=0.0, shift=0.0):
    """把真值捏成一条"有已知误差"的估计轨迹。"""
    out_p = ps.copy()
    center = np.array([2.5, 1.25, 0.0])
    out_p[:, :2] = center[:2] + (out_p[:, :2] - center[:2]) * scale
    out_p[:, 0] += shift
    out_q = []
    for i, q in enumerate(qs):
        yaw = 2 * math.atan2(q[2], q[3]) + math.radians(yaw_drift_deg) * i
        out_q.append([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return out_p, np.array(out_q)


def write_csv(path, gt, est, floor_stamp=False):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    gt_t, gt_p, gt_q = gt
    es_t, es_p, es_q = est
    with open(path, 'w', newline='', encoding='utf-8') as f:
        f.write('kind,stamp,node_now,x,y,z,qx,qy,qz,qw\n')
        for i in range(len(gt_t)):
            f.write(f'gt,{gt_t[i]:.9f},{gt_t[i]:.9f},{gt_p[i][0]:.6f},{gt_p[i][1]:.6f},'
                    f'{gt_p[i][2]:.6f},{gt_q[i][0]:.7f},{gt_q[i][1]:.7f},'
                    f'{gt_q[i][2]:.7f},{gt_q[i][3]:.7f}\n')
        for i in range(len(es_t)):
            t = math.floor(es_t[i]) if floor_stamp else es_t[i]
            f.write(f'est,{t:.9f},{es_t[i] + 0.02:.9f},{es_p[i][0]:.6f},{es_p[i][1]:.6f},'
                    f'{es_p[i][2]:.6f},{es_q[i][0]:.7f},{es_q[i][1]:.7f},'
                    f'{es_q[i][2]:.7f},{es_q[i][3]:.7f}\n')
    with open(os.path.splitext(path)[0] + '.meta.json', 'w', encoding='utf-8') as f:
        json.dump({'algo': os.path.basename(path).split('.')[0],
                   'est_topic': '/test'}, f)


# ------------------------------------------------------------- matplotlib 桩
STUB = '''# 由 _test_pipeline.py 生成的 matplotlib 桩：只验证"代码会不会抛异常"
import os
import numpy as np

def use(*a, **k):
    pass
'''

STUB_PYPLOT = '''import os
import numpy as np


class _Bar:
    """ax.bar() 的返回值：真实代码会读 get_x/get_width/get_height 来标数值。"""
    def get_x(self):
        return 0.0

    def get_width(self):
        return 0.1

    def get_height(self):
        return 1.0


class _Axes:
    def __getattr__(self, name):
        def _noop(*a, **k):
            return None
        return _noop

    def bar(self, *args, **kwargs):
        heights = args[1] if len(args) > 1 else kwargs.get('height', [])
        try:
            n = len(heights)
        except TypeError:
            n = 0
        return [_Bar() for _ in range(n)]

    def text(self, *args, **kwargs):
        return None


class _Figure:
    def __init__(self):
        self.n = 0

    def tight_layout(self, *a, **k):
        return None

    def savefig(self, path, *a, **k):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(b'PNG-STUB')


def subplots(nrows=1, ncols=1, figsize=None, **k):
    axes = np.empty((nrows, ncols), dtype=object)
    for i in range(nrows):
        for j in range(ncols):
            axes[i, j] = _Axes()
    if nrows == 1 and ncols == 1:
        return _Figure(), axes[0, 0]
    return _Figure(), axes


def close(fig=None):
    return None
'''


def run(cmd, env=None, timeout=180):
    e = dict(os.environ)
    e.update(env or {})
    e['PYTHONIOENCODING'] = 'utf-8'
    return subprocess.run(cmd, cwd=ROOT, env=e, capture_output=True,
                          text=True, encoding='utf-8', timeout=timeout)


def main():
    keep = '--keep' in sys.argv
    if os.path.isdir(WORK):
        shutil.rmtree(WORK)
    raw = os.path.join(WORK, 'raw')
    results = os.path.join(WORK, 'results')
    os.makedirs(os.path.join(results, 'tables'), exist_ok=True)

    py = sys.executable
    print(f'python : {py}')
    print(f'work   : {WORK}\n')

    # ---------------------------------------------------------- 1 造数据
    print('=== 1. 造两份形状与 traj_recorder.py 相同的 CSV ===')
    gt = circle()
    # S0：LOAM 差一点，整数秒时间戳（floor 关联）；LIO-SAM 好一点，精确时间戳
    write_csv(os.path.join(raw, 's0_nominal', 'loam.csv'), gt,
              (gt[0],) + distorts(gt[1], gt[2], scale=1.02), floor_stamp=True)
    write_csv(os.path.join(raw, 's0_nominal', 'liosam.csv'), gt,
              (gt[0],) + distorts(gt[1], gt[2], scale=1.004))
    # S1：故意让 LIO-SAM 更差（用来验证 PPT 会不会照实写 "LIO-SAM worse"）
    write_csv(os.path.join(raw, 's1_fast_yaw', 'loam.csv'), gt,
              (gt[0],) + distorts(gt[1], gt[2], scale=1.01), floor_stamp=True)
    write_csv(os.path.join(raw, 's1_fast_yaw', 'liosam.csv'), gt,
              (gt[0],) + distorts(gt[1], gt[2], scale=1.05, yaw_drift_deg=0.02))
    csvs = [os.path.join(raw, s, f'{a}.csv')
            for s in ('s0_nominal', 's1_fast_yaw') for a in ('loam', 'liosam')]
    check('4 个 CSV 已生成', all(os.path.isfile(p) for p in csvs))

    # ---------------------------------------------------------- 2 evaluate
    print('\n=== 2. evaluate.py（指标 + 表） ===')
    r = run([py, os.path.join(BENCH, 'evaluate.py'), '--csv', *csvs,
             '--md-out', os.path.join(results, 'tables', 'all_runs.md'),
             '--csv-out', os.path.join(results, 'tables', 'all_runs.csv'),
             '--json-out', os.path.join(results, 'json', 'all.json')])
    check('evaluate 退出码 0', r.returncode == 0, (r.stderr or '')[-400:])
    for rel in ('tables/all_runs.csv', 'tables/all_runs.md', 'json/all.json'):
        p = os.path.join(results, rel)
        check(f'产出 {rel}', os.path.isfile(p))
    table = list(csv.DictReader(open(os.path.join(results, 'tables', 'all_runs.csv'),
                                     encoding='utf-8')))
    check('表里 4 行', len(table) == 4, f'{len(table)} 行')
    # floor 关联必须生效（LOAM 的行）
    js = json.load(open(os.path.join(results, 'json', 'all.json'), encoding='utf-8'))
    modes = {r['algo'] + '@' + os.path.basename(os.path.dirname(r['csv'])):
             r['association']['mode'] for r in js}
    check('LOAM 走 floor 关联', modes.get('loam@s0_nominal') == 'floor', str(modes))
    check('LIO-SAM 走 exact 关联', modes.get('liosam@s0_nominal') == 'exact', str(modes))
    ate = {f"{r['algo']}@{os.path.basename(os.path.dirname(r['csv']))}":
           r['ate']['rmse'] for r in js}
    check('S0: LIO-SAM 比 LOAM 准', ate['liosam@s0_nominal'] < ate['loam@s0_nominal'],
          f"{ate['loam@s0_nominal']:.4f} -> {ate['liosam@s0_nominal']:.4f} m")
    check('S1: LIO-SAM 反而更差（合成的），评测器照实报',
          ate['liosam@s1_fast_yaw'] > ate['loam@s1_fast_yaw'],
          f"{ate['loam@s1_fast_yaw']:.4f} -> {ate['liosam@s1_fast_yaw']:.4f} m")

    # ------------------------------------------------------- 3 make_figures
    print('\n=== 3. make_figures.py（用 matplotlib 桩，验证逻辑不抛异常） ===')
    stub_dir = os.path.join(WORK, 'stub', 'matplotlib')
    os.makedirs(stub_dir, exist_ok=True)
    with open(os.path.join(stub_dir, '__init__.py'), 'w', encoding='utf-8') as f:
        f.write(STUB)
    with open(os.path.join(stub_dir, 'pyplot.py'), 'w', encoding='utf-8') as f:
        f.write(STUB_PYPLOT)
    figs = os.path.join(results, 'figures')
    r = run([py, os.path.join(BENCH, 'make_figures.py'),
             '--raw-dir', raw, '--out', figs],
            env={'PYTHONPATH': os.path.join(WORK, 'stub')})
    check('make_figures 退出码 0', r.returncode == 0, (r.stderr or '')[-500:])
    made = sorted(os.listdir(figs)) if os.path.isdir(figs) else []
    for want in ('xy_s0_nominal.png', 'err_s0_nominal.png', 'ate_summary.png'):
        check(f'产出 {want}', want in made, str(made))
    if 'xy_all.png' in made:
        print('  [ OK ] 产出 xy_all.png')

    # ------------------------------------------------------------ 4 deck
    print('\n=== 4. build_deck.py（数字要落进幻灯片） ===')
    deck = os.path.join(WORK, 'deck.pptx')
    r = run([py, os.path.join(ROOT, 'report', 'build_deck.py'),
             '--results', results, '--out', deck])
    check('build_deck 退出码 0', r.returncode == 0, (r.stderr or '')[-600:])
    check('PPTX 已生成', os.path.isfile(deck))

    try:
        from pptx import Presentation
        prs = Presentation(deck)
        texts = []
        for slide in prs.slides:
            for sh in slide.shapes:
                if sh.has_text_frame:
                    texts.append(sh.text_frame.text)
                if getattr(sh, 'has_table', False) and sh.has_table:
                    for row in sh.table.rows:
                        for cell in row.cells:
                            texts.append(cell.text)
        blob = '\n'.join(texts)
        check('幻灯片数量 18', len(prs.slides) == 18, f'{len(prs.slides)} 张')

        # 由 CSV 反推应该出现的百分比，再检查它真的在稿子里
        imp = {}
        for row in table:
            scene = os.path.basename(os.path.dirname(row['csv']))
            imp.setdefault(scene, {})[row['algorithm']] = float(row['ate_rmse_m'])
        for scene in ('s0_nominal', 's1_fast_yaw'):
            lo, li = imp[scene]['loam'], imp[scene]['liosam']
            expect = f'{(lo - li) / lo * 100:+.0f}%'
            check(f'{scene} 的改善 {expect} 出现在幻灯片里', expect in blob)
        check('S1 被标成 "LIO-SAM worse"（结论没被写死）',
              'LIO-SAM worse' in blob)
        check('没有残留 DATA PENDING 占位',
              'DATA PENDING' not in blob)
    except ImportError:
        print('  [skip] 没装 python-pptx，跳过幻灯片内容检查')

    # ------------------------------------------------------- 5 sanity_check
    print('\n=== 5. sanity_check.py（先怀疑数据，再谈结论） ===')
    table_csv = os.path.join(results, 'tables', 'all_runs.csv')
    r = run([py, os.path.join(ROOT, 'experiments', 'sanity_check.py'), table_csv,
             '--json-dir', os.path.join(results, 'json')])
    check('正常数据判定可用（WARN 可以有，FAIL 不能有）',
          r.returncode == 0 and '[FAIL]' not in r.stdout, (r.stdout or '')[-300:])

    # 造一份"两条算法数字完全相同"的表：那几乎肯定是录到了同一个话题，必须判 FAIL
    with open(table_csv, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    by = {}
    for row in rows:
        scene = os.path.basename(os.path.dirname(row['csv']))
        by[(scene, row['algorithm'])] = row
    for row in rows:
        scene = os.path.basename(os.path.dirname(row['csv']))
        lo = by.get((scene, 'loam'))
        if row['algorithm'] == 'liosam' and lo:
            for k in ('ate_rmse_m', 'ate_max_m', 'rpe_trans_pct_m', 'final_pos_err_m'):
                row[k] = lo[k]
    broken = os.path.join(WORK, 'broken.csv')
    with open(broken, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    r = run([py, os.path.join(ROOT, 'experiments', 'sanity_check.py'), broken])
    check('两条算法数字相同 -> 判 FAIL（防"录到同一个话题"）',
          r.returncode == 1 and '[FAIL]' in r.stdout, (r.stdout or '')[-260:])

    # ------------------------------------------------------------ 收尾
    print('\n=== 结论 ===')
    if FAILS:
        print(f'{len(FAILS)} FAILED: {FAILS}')
        print(f'（产物保留在 {WORK}，方便查）')
        return 1
    print('ALL PASS —— 录制之后的那条链（指标 → 表 → 图 → PPT）是通的')
    if not keep:
        shutil.rmtree(WORK, ignore_errors=True)
    else:
        print(f'产物保留: {WORK}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
