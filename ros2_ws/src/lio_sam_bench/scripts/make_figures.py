#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_figures.py —— 把评测结果画成报告/PPT 能直接用的图
===============================================================================
输入：results/raw/<scenario>/<algo>.csv（traj_recorder.py 录的）
输出（默认写到 results/figures/）：

    xy_<scenario>.png        真值 vs 两条算法的水平轨迹（叠加）
    err_<scenario>.png       逐帧位置误差 vs 时间
    ate_summary.png          各场景 ATE RMSE 柱状对比（报告主图）
    xy_all.png               所有场景拼成一张总览

依赖 matplotlib（Jetson 上： sudo apt install python3-matplotlib）。
没装的话本脚本会直接告诉你，不会崩。

用法：
    python3 make_figures.py --raw-dir results/raw --out results/figures
    python3 make_figures.py --raw-dir results/raw --out results/figures --only s0_nominal
"""

import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate as ev                                    # noqa: E402


ALGO_STYLE = {
    'loam':   {'color': '#ff9900', 'label': 'LOAM (baseline)'},
    'liosam': {'color': '#2f80ed', 'label': 'LIO-SAM'},
}

# 图上要写人话。目录名是 s0_nominal 这种，报告里读者想看到的是 "S0 nominal"。
SCEN_LABEL = {
    's0_nominal': 'S0  nominal',
    's1_fast_yaw': 'S1  high yaw rate',
    's2_high_speed': 'S2  high speed',
    's3_corridor': 'S3  corridor',
    's4_sparse_hall': 'S4  sparse hall',
    's5_long_run': 'S5  long run',
}

# 鲁棒性判据：误差超过这个值的帧，报告里单独计数（和 evaluate.py 一致）
FAIL_THRESHOLD_M = 0.5


def scen_label(name):
    return SCEN_LABEL.get(name, name)


def _style(algo):
    return ALGO_STYLE.get(algo, {'color': '#888888', 'label': algo})


def collect(raw_dir, only=None):
    """把 raw_dir 下所有 CSV 按场景分组：{scenario: {algo: path}}"""
    groups = {}
    for path in sorted(glob.glob(os.path.join(raw_dir, '**', '*.csv'),
                                 recursive=True)):
        scenario = os.path.basename(os.path.dirname(os.path.abspath(path)))
        if only and scenario not in only:
            continue
        meta = ev.load_meta(path)
        algo = meta.get('algo') or os.path.splitext(os.path.basename(path))[0]
        groups.setdefault(scenario, {})[algo] = path
    return groups


def fig_xy(plt, arrays, scenario, out_path):
    """一条场景里：真值 + 各算法对齐后的水平轨迹。"""
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ax.plot(arrays['gt'][:, 0], arrays['gt'][:, 1],
            color='black', lw=2.0, label='Ground truth', zorder=1)
    for name, a in arrays['runs'].items():
        st = _style(a['algo'])
        ax.plot(a['est_aligned'][:, 0], a['est_aligned'][:, 1],
                color=st['color'], lw=1.4, label=st['label'], zorder=2)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'Trajectory comparison — {scen_label(scenario)}')
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def fig_err(plt, arrays, scenario, out_path):
    """逐帧位置误差（对齐后）vs 时间。"""
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    ymax = 0.0
    for name, a in arrays['runs'].items():
        st = _style(a['algo'])
        ax.plot(a['t'], a['err'], color=st['color'], lw=1.2, label=st['label'])
        if a['err'].size:
            ymax = max(ymax, float(np.max(a['err'])))

    # 0.5 m 参考线只在误差真的接近它时才画。
    # 否则（比如 ATE 只有 2 cm 的名义场景）这条线会把纵轴撑到 0.5，
    # 曲线被压成贴着底部的一条直线 —— 图就废了。
    show_thr = ymax > 0.4 * FAIL_THRESHOLD_M
    if show_thr:
        ax.axhline(FAIL_THRESHOLD_M, color='#999999', ls='--', lw=0.8,
                   label=f'{FAIL_THRESHOLD_M:.1f} m failure threshold')
    ax.set_ylim(0, max(ymax * 1.15,
                       FAIL_THRESHOLD_M * 1.05 if show_thr else 1e-3))
    ax.grid(alpha=0.3)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('position error [m]')
    ax.set_title(f'Position error after SE(3) alignment — {scen_label(scenario)}')
    ax.legend(loc='best', fontsize=8.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def fig_ate_summary(plt, metrics, out_path):
    """各场景 ATE RMSE 柱状图 —— 报告里的主图。柱子上直接标数值。"""
    scenarios = sorted({m['scenario'] for m in metrics})
    algos = sorted({m['algo'] for m in metrics})
    x = np.arange(len(scenarios))
    width = 0.8 / max(len(algos), 1)

    fig, ax = plt.subplots(figsize=(max(6.4, 1.5 * len(scenarios) + 3), 4.0))
    for k, algo in enumerate(algos):
        vals = []
        for s in scenarios:
            v = [m['ate_rmse'] for m in metrics
                 if m['scenario'] == s and m['algo'] == algo]
            vals.append(v[0] if v and v[0] is not None else np.nan)
        st = _style(algo)
        bars = ax.bar(x + k * width, vals, width, color=st['color'], label=st['label'])
        # 标数值：报告读者不该靠目测刻度。
        # 位数按量级自适应 —— 6 mm 的 ATE 用三位小数会显示成 0.006 没问题，
        # 但亚毫米的误差会被四舍五入成 0.000，那就等于没标。
        for rect, v in zip(bars, vals):
            if v is None or not np.isfinite(v):
                continue
            if v == 0:
                label = '0'
            elif v < 0.01:
                label = f'{v:.4f}'
            elif v < 1:
                label = f'{v:.3f}'
            else:
                label = f'{v:.2f}'
            ax.text(rect.get_x() + rect.get_width() / 2.0,
                    rect.get_height(),
                    label,
                    ha='center', va='bottom', fontsize=7.5, color='#333333')
    ax.set_xticks(x + width * (len(algos) - 1) / 2)
    ax.set_xticklabels([scen_label(s) for s in scenarios], rotation=12,
                       ha='right', fontsize=8.5)
    ax.set_ylabel('ATE RMSE [m]')
    ax.set_title('Absolute trajectory error by scenario (lower is better)')
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description='生成报告用的轨迹图 / 误差图 / ATE 柱状图')
    ap.add_argument('--raw-dir', default='results/raw')
    ap.add_argument('--out', default='results/figures')
    ap.add_argument('--only', nargs='+', default=None,
                    help='只处理这些场景（目录名）')
    args = ap.parse_args(argv)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('[skip] 没装 matplotlib，画不了图。装一下：')
        print('       sudo apt install python3-matplotlib')
        print('       （评测数字不受影响，可以用 evaluate.py --csv-out 出表）')
        return 0

    groups = collect(args.raw_dir, args.only)
    if not groups:
        print(f'[skip] {args.raw_dir} 下没有 CSV')
        return 1

    os.makedirs(args.out, exist_ok=True)
    metrics = []
    overview = {}

    for scenario, runs in sorted(groups.items()):
        arrays = {'runs': {}}
        for algo, path in sorted(runs.items()):
            a = ev.aligned_arrays(path)
            if a is None:
                print(f'[warn] {path}: 关联失败，跳过')
                continue
            arrays['runs'][algo] = a
            r = ev.evaluate_run(path)
            if r.get('ok'):
                metrics.append({'scenario': scenario, 'algo': algo,
                                'ate_rmse': r['ate']['rmse']})
        if not arrays['runs']:
            continue
        # 真值只取一份（各算法的 gt 是同一串）
        arrays['gt'] = next(iter(arrays['runs'].values()))['gt']
        overview[scenario] = arrays

        fig_xy(plt, arrays, scenario, os.path.join(args.out, f'xy_{scenario}.png'))
        fig_err(plt, arrays, scenario, os.path.join(args.out, f'err_{scenario}.png'))
        print(f'[ok] {scenario}: xy_{scenario}.png, err_{scenario}.png')

    if metrics:
        fig_ate_summary(plt, metrics, os.path.join(args.out, 'ate_summary.png'))
        print('[ok] ate_summary.png')

    # 所有场景拼一张总览（最多 6 个）
    if len(overview) > 1:
        items = sorted(overview.items())[:6]
        ncol = 3 if len(items) > 2 else len(items)
        nrow = int(np.ceil(len(items) / ncol))
        fig, axes = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 4.0 * nrow))
        axes = np.atleast_1d(axes).ravel()
        for ax, (scenario, arrays) in zip(axes, items):
            ax.plot(arrays['gt'][:, 0], arrays['gt'][:, 1],
                    color='black', lw=1.8, label='GT')
            for name, a in arrays['runs'].items():
                st = _style(a['algo'])
                ax.plot(a['est_aligned'][:, 0], a['est_aligned'][:, 1],
                        color=st['color'], lw=1.2, label=st['label'])
            ax.set_aspect('equal', adjustable='datalim')
            ax.set_title(scen_label(scenario), fontsize=9)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=7)
        for ax in axes[len(items):]:
            ax.axis('off')
        axes[0].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, 'xy_all.png'), dpi=160)
        plt.close(fig)
        print('[ok] xy_all.png')

    return 0


if __name__ == '__main__':
    sys.exit(main())
