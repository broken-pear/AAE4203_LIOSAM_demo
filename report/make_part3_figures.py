"""生成 Part 3 需要的两张插图。

1. map_s0 … map_s5  —— 六个场景的极简俯视图（墙体 + 实际行驶路径），
   几何从各自的 world SDF 读，路径从场景 yaml 的速度/时长积分。
   刻意保持朴素：浅灰墙体、橙色路径、黑点起点，不放任何 Gazebo 元素，
   目的是让观众一眼看出"这个场景的几何长什么样"。

2. ate_math.png —— ATE 那一页的图解：对齐前后的两条轨迹、
   逐帧误差向量、以及对齐矩阵云。

输出到 results/figures/
"""
import math
import os
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_gz_bringup', 'worlds')
SCEN = os.path.join(ROOT, 'experiments', 'scenarios')
FIGS = os.path.join(ROOT, 'results', 'figures')

WALL = '#d9dde3'
WALL_E = '#9aa2ad'
ORANGE = '#f5a01c'
INK = '#262626'
GREY = '#8a929c'
BLUE = '#2f7ed8'

# 出生点：launch 里 ros_gz_sim create 的 -x -y
SPAWN = (2.5, 0.0)


def world_boxes(name):
    """返回世界里的矩形（墙/柱/地板），单位 m"""
    root = ET.parse(os.path.join(WORLDS, name)).getroot()
    out = []
    for m in root.find('world').findall('model'):
        nm = m.get('name', '')
        if nm == 'ground':
            continue
        pose = [float(v) for v in m.find('pose').text.split()]
        size = None
        for col in m.iter('collision'):
            sz = col.find('.//size')
            if sz is not None:
                size = [float(v) for v in sz.text.split()]
        out.append((pose[0], pose[1], size[0], size[1]))
    return out


def scenario_arrays(path):
    text = open(path, encoding='utf-8').read()
    vals = {}
    for key in ('linear_speed', 'angular_speed', 'duration'):
        m = re.search(key + r':\s*\[([^\]]+)\]', text)
        if m:
            vals[key] = [float(x) for x in m.group(1).replace(',', ' ').split()]
        else:
            m1 = re.search(key + r':\s*([\d.]+)', text)
            vals[key] = [float(m1.group(1))]
    m = re.search(r'rate_hz:\s*([\d.]+)', text)
    vals['rate'] = float(m.group(1)) if m else 20.0
    m = re.search(r'world:\s*(\S+)', text)
    vals['world'] = m.group(1)
    return vals


def integrate(vals, x0, y0, max_steps=None):
    xs, ys = [x0], [y0]
    x, y, yaw = x0, y0, 0.0
    dt = 1.0 / vals['rate']
    for v, w, dur in zip(vals['linear_speed'], vals['angular_speed'],
                         vals['duration']):
        for _ in range(int(round(dur * vals['rate']))):
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
            xs.append(x)
            ys.append(y)
    return np.array(xs), np.array(ys)


def draw_map(ax, boxes, xs, ys):
    """极简俯视图：墙体 + 路径 + 起点。不画刻度/标题 —— 放在幻灯片上时
    这几个像素宽的刻度只会变成噪点；场景编号用幻灯片上的文字标。"""
    for (cx, cy, sx, sy) in boxes:
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                               facecolor=WALL, edgecolor=WALL_E,
                               linewidth=0.5, zorder=2))
    ax.plot(xs, ys, color=ORANGE, linewidth=1.4, zorder=4)
    ax.plot([xs[0]], [ys[0]], marker='o', markersize=3.0, color=INK, zorder=5)

    # 视野按世界包围盒 + 少量留白，保证每个场景都占满画面
    x_all = np.concatenate([xs, [b[0] - b[2] / 2 for b in boxes],
                            [b[0] + b[2] / 2 for b in boxes]])
    y_all = np.concatenate([ys, [b[1] - b[3] / 2 for b in boxes],
                            [b[1] + b[3] / 2 for b in boxes]])
    mx = 0.04 * max(1.0, float(x_all.max() - x_all.min()))
    my = 0.10 * max(1.0, float(y_all.max() - y_all.min()))
    ax.set_xlim(x_all.min() - mx, x_all.max() + mx)
    ax.set_ylim(y_all.min() - my, y_all.max() + my)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color('#cdd3da')
        sp.set_linewidth(0.6)


def _crop_whitespace(path, pad=0.04):
    """裁掉图片四周的纯白像素（保留 pad 比例的边）"""
    from PIL import Image, ImageChops
    im = Image.open(path).convert('RGB')
    bg = Image.new('RGB', im.size, (255, 255, 255))
    diff = ImageChops.difference(im, bg).convert('L')
    box = diff.point(lambda v: 255 if v > 8 else 0).getbbox()
    if not box:
        return
    x0, y0, x1, y1 = box
    px, py = int((x1 - x0) * pad) + 2, int((y1 - y0) * pad) + 2
    im.crop((max(0, x0 - px), max(0, y0 - py),
             min(im.width, x1 + px), min(im.height, y1 + py))).save(path)


def main():
    os.makedirs(FIGS, exist_ok=True)
    names = ['s0_nominal', 's1_fast_yaw', 's2_high_speed',
             's3_corridor', 's4_sparse_hall', 's5_long_run']
    for i, nm in enumerate(names):
        vals = scenario_arrays(os.path.join(SCEN, nm + '.yaml'))
        boxes = world_boxes(vals['world'])
        xs, ys = integrate(vals, *SPAWN)
        fig, ax = plt.subplots(figsize=(1.90, 1.30), dpi=280)
        draw_map(ax, boxes, xs, ys)
        fig.tight_layout(pad=0.25)
        out = os.path.join(FIGS, 'map_s%d.png' % i)
        fig.savefig(out, format='png')
        plt.close(fig)
        _crop_whitespace(out)
        print('%-16s world=%-16s path %6.1f m  -> %s'
              % (nm, vals['world'], np.sum(np.hypot(np.diff(xs), np.diff(ys))),
                 os.path.basename(out)))

    # ---------------------------------------------------------------- ATE 图解
    rng = np.random.default_rng(7)
    t = np.linspace(0, 1, 9)
    gt = np.stack([3.2 * t, 1.1 * np.sin(2.4 * t)], axis=1)
    th = math.radians(32)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    est = gt @ R.T + np.array([-1.1, 0.55]) + rng.normal(0, 0.12, gt.shape)

    mu_g, mu_e = gt.mean(0), est.mean(0)
    H = (est - mu_e).T @ (gt - mu_g)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Rr = Vt.T @ np.diag([1, d]) @ U.T
    # aligned = R*(est-mu_e) + mu_g  —— 注意不是再加 mu_e，
    # 第一版写错成 (R(est-mu_e)+mu_e+t)，结果对齐后整体偏了一个质心。
    aligned = (Rr @ (est - mu_e).T).T + mu_g
    err = np.linalg.norm(aligned - gt, axis=1)
    rmse = float(np.sqrt(np.mean(err ** 2)))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.4, 2.9), dpi=200)
    ax1.plot(gt[:, 0], gt[:, 1], color=INK, linewidth=1.6, label='ground truth')
    ax1.plot(est[:, 0], est[:, 1], color=BLUE, linewidth=1.4,
             label='estimated (own frame)')
    ax1.set_title('1  before alignment: two different world frames',
                  fontsize=9, color=INK)
    ax1.legend(fontsize=7, loc='upper left', framealpha=0.9)

    ax2.plot(gt[:, 0], gt[:, 1], color=INK, linewidth=1.6)
    ax2.plot(aligned[:, 0], aligned[:, 1], color=BLUE, linewidth=1.4)
    ax2.plot(gt[:, 0], gt[:, 1], 'o', color=INK, markersize=3.2)
    ax2.plot(aligned[:, 0], aligned[:, 1], 'o', color=BLUE, markersize=3.2)
    for (gx, gy), (ax_, ay_) in zip(gt, aligned):
        ax2.add_patch(FancyArrowPatch((gx, gy), (ax_, ay_), arrowstyle='-',
                                      color='#c0392b', linewidth=1.5,
                                      alpha=0.85))
    ax2.set_title('2  after alignment: e$_i$ = red segments, '
                  'RMSE = %.2f m' % rmse, fontsize=9, color=INK)
    for a in (ax1, ax2):
        a.set_aspect('equal')
        a.tick_params(labelsize=7)
        a.grid(alpha=0.2)
        for s in ('top', 'right'):
            a.spines[s].set_visible(False)
        a.set_xlabel('x  [m]', fontsize=7.5)
    ax1.set_ylabel('y  [m]', fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, 'ate_math.png'), format='png')
    plt.close(fig)
    print('ate_math.png   example RMSE = %.3f m' % rmse)


if __name__ == '__main__':
    main()
