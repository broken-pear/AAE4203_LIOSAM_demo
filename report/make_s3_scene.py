"""画 S3 走廊场景的俯视示意图（第三版）。

左栏：世界俯视 + 机器人实际走的四段往返（几何与轨迹都从配置文件读）
右栏：**一次真实扫描的点云俯视** —— 按 corridor.sdf 的四面墙做射线求交，
      每个点按量程着色。看到的就是两条平行点线，这解释了为什么沿 x
      平移几乎不改变观测量。

输出 results/figures/s3_scene.png
"""
import math
import os
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_gz_bringup',
                   'worlds', 'corridor.sdf')
YAML = os.path.join(ROOT, 'experiments', 'scenarios', 's3_corridor.yaml')
OUT = os.path.join(ROOT, 'results', 'figures', 's3_scene.png')

ORANGE = '#f5a01c'
ORANGE_D = '#b9740a'
INK = '#262626'
GREY = '#595959'
RED = '#c0392b'
WALL = '#c9ced6'
WALL_E = '#8a9099'


def read_models():
    root = ET.parse(SDF).getroot()
    out = []
    for m in root.find('world').findall('model'):
        name = m.get('name', '')
        pose = [float(v) for v in m.find('pose').text.split()]
        size = None
        for col in m.iter('collision'):
            sz = col.find('.//size')
            if sz is not None:
                size = [float(v) for v in sz.text.split()]
        out.append((name, pose[0], pose[1], size[0], size[1]))
    return out


def read_motion():
    text = open(YAML, encoding='utf-8').read()
    vals = {}
    for key in ('linear_speed', 'angular_speed', 'duration'):
        m = re.search(key + r':\s*\[([^\]]+)\]', text)
        vals[key] = [float(x) for x in m.group(1).replace(',', ' ').split()]
    m = re.search(r'rate_hz:\s*([\d.]+)', text)
    vals['rate'] = float(m.group(1)) if m else 20.0
    return list(zip(vals['linear_speed'], vals['angular_speed'],
                    vals['duration'])), vals['rate']


def integrate(seg, rate, x0=2.5, y0=0.0, yaw0=0.0):
    pts = [(x0, y0, yaw0)]
    x, y, yaw = x0, y0, yaw0
    dt = 1.0 / rate
    for v, w, dur in seg:
        for _ in range(int(round(dur * rate))):
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
            pts.append((x, y, yaw))
    return pts


def cast(boxes, ox, oy, n=1440):
    """从 (ox,oy) 均匀发射 n 条射线，返回命中点与量程（只算水平面）"""
    hits = []
    for k in range(n):
        a = 2 * math.pi * k / n
        ca, sa = math.cos(a), math.sin(a)
        best = None
        for (x0, y0, x1, y1) in boxes:
            # 轴对齐盒的 slab 求交
            tmin, tmax = 0.0, 1e9
            ok = True
            for (p, d, lo, hi) in ((ox, ca, x0, x1), (oy, sa, y0, y1)):
                if abs(d) < 1e-12:
                    if p < lo or p > hi:
                        ok = False
                        break
                else:
                    t1, t2 = (lo - p) / d, (hi - p) / d
                    if t1 > t2:
                        t1, t2 = t2, t1
                    tmin, tmax = max(tmin, t1), min(tmax, t2)
                    if tmin > tmax:
                        ok = False
                        break
            if ok and tmin > 1e-6:
                best = tmin if best is None else min(best, tmin)
        if best is not None and best < 30:
            hits.append((ox + best * ca, oy + best * sa, best))
    return hits


def main():
    models = read_models()
    seg, rate = read_motion()
    pts = integrate(seg, rate)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    total = sum(math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
                for i in range(len(pts) - 1))

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(9.6, 2.85), dpi=210,
        gridspec_kw={'width_ratios': [2.0, 1.05], 'wspace': 0.14})

    # -------------------------------------------------- 左：世界 + 轨迹
    # 走廊长 30 m、宽 3 m。等比例画全长的结果是 10:1 的细条，什么都看不清，
    # 所以只画机器人用到的那一段（x = 0 … 12 m），并在图上注明总长。
    for name, cx, cy, sx, sy in models:
        if name == 'ground':
            continue
        ax1.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                facecolor=WALL, edgecolor=WALL_E,
                                linewidth=0.8, zorder=2))
    ax1.plot(xs, ys, color=ORANGE, linewidth=2.6, zorder=4,
             solid_capstyle='round')
    for xv in (min(xs), max(xs)):
        ax1.add_patch(Circle((xv, 0), 0.45, fill=False, linestyle='--',
                             linewidth=0.9, edgecolor=ORANGE_D, zorder=5))
    ax1.plot([xs[0]], [ys[0]], marker='o', markersize=6.5,
             color=ORANGE_D, markeredgecolor='white', zorder=6)
    ax1.annotate('start', (xs[0], ys[0]), textcoords='offset points',
                 xytext=(-8, 10), fontsize=8.5, color=GREY)

    ax1.annotate('', xy=(max(xs), -0.85), xytext=(min(xs), -0.85),
                 arrowprops=dict(arrowstyle='<->', color=ORANGE_D, lw=1.2))
    ax1.text((min(xs) + max(xs)) / 2, -1.18, '7.2 m per leg', ha='center',
             fontsize=8.6, color=ORANGE_D)

    ax1.annotate('', xy=(12.2, -1.5), xytext=(12.2, 1.5),
                 arrowprops=dict(arrowstyle='<->', color=INK, lw=0.9))
    ax1.text(12.55, 0, '3 m', rotation=90, va='center', fontsize=9, color=INK)
    ax1.text(6.0, 1.92, 'corridor is 30 m long — this is the stretch the robot '
             'drives (x = 0 … 12 m)', ha='center', fontsize=8.4, color=GREY)

    ax1.set_xlim(-0.6, 13.4)
    ax1.set_ylim(-2.05, 2.15)
    ax1.set_aspect('equal')
    ax1.set_xlabel('x  [m]', fontsize=9)
    ax1.set_ylabel('y  [m]', fontsize=9)
    ax1.tick_params(labelsize=8.5)
    ax1.set_yticks([-1.5, 0, 1.5])
    ax1.set_title('S3 world: a 3 m wide corridor, driven along its axis',
                  fontsize=10.2, color=INK)
    ax1.grid(alpha=0.22, zorder=0)
    for sp in ('top', 'right'):
        ax1.spines[sp].set_visible(False)

    # -------------------------------------------------- 右：一次扫描的点云
    boxes = [(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
             for name, cx, cy, sx, sy in models if name != 'ground']
    rx, ry = 6.0, 0.0
    hits = cast(boxes, rx, ry, n=1440)
    hx = [h[0] for h in hits]
    hy = [h[1] for h in hits]
    hr = [h[2] for h in hits]
    sc = ax2.scatter(hx, hy, c=hr, s=1.6, cmap='viridis', zorder=3,
                     vmin=0, vmax=max(hr) if hr else 1)
    ax2.plot([rx], [ry], marker='s', markersize=7, color=ORANGE_D,
             markeredgecolor='white', zorder=6)

    ax2.annotate('', xy=(rx + 2.6, -0.55), xytext=(rx + 0.7, -0.55),
                 arrowprops=dict(arrowstyle='->', color=RED, lw=1.6))
    ax2.text(rx - 0.6, -1.28,
             'sliding along x moves these points along themselves',
             fontsize=7.2, color=RED, ha='center')

    ax2.text(4.0, 1.32, 'two parallel lines of points', fontsize=8.4,
             color=INK, ha='center')

    ax2.set_xlim(0.2, 12.3)
    ax2.set_ylim(-2.1, 2.1)
    ax2.set_xlabel('x  [m]', fontsize=9)
    ax2.tick_params(labelsize=8.5)
    ax2.set_yticks([-1.5, 0, 1.5])
    ax2.set_title('One sweep (1 440 rays) from inside', fontsize=10.2,
                  color=INK)
    cb = fig.colorbar(sc, ax=ax2, pad=0.015, fraction=0.045)
    cb.set_label('range [m]', fontsize=7.5)
    cb.ax.tick_params(labelsize=7)
    for s in ('top', 'right'):
        ax2.spines[s].set_visible(False)

    fig.subplots_adjust(left=0.045, right=0.995, top=0.90, bottom=0.15)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, format='png')
    print('wrote', OUT)
    print('path %.2f m, x %.2f..%.2f, rays hit %d'
          % (total, min(xs), max(xs), len(hits)))


if __name__ == '__main__':
    main()
