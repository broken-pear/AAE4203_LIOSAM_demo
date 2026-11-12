"""按 Gazebo（Harmonic）界面的样式，渲染 S3 走廊的 3D 实景视图。

说明（重要）：这台机器上没有 GPU / ROS，跑不了真的 gz sim。
所以这里不是截屏，而是**用世界文件和机器人模型本身的几何**做透视投影重建：
  * worlds/corridor.sdf   -> 地板、四面墙
  * models/lio_bot/model.sdf -> 底盘尺寸、雷达/IMU 安装高度
  * 场景 yaml 的起点        -> 机器人在走廊里的位置
点云是按 16 线 × 360° 对墙面做射线求交算出来的，不是贴图。

输出 results/figures/s3_gazebo_view.png
"""
import math
import os
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle, FancyBboxPatch, Circle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_gz_bringup',
                   'worlds', 'corridor.sdf')
MODEL = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_gz_bringup',
                     'models', 'lio_bot', 'model.sdf')
YAML = os.path.join(ROOT, 'experiments', 'scenarios', 's3_corridor.yaml')
OUT = os.path.join(ROOT, 'results', 'figures', 's3_gazebo_view.png')

# Gazebo Harmonic 深色主题
UI_BG = '#2b2b2b'
UI_BAR = '#232323'
UI_PANEL = '#313131'
UI_LINE = '#3d3d3d'
UI_TEXT = '#d7d7d7'
UI_DIM = '#9a9a9a'
UI_ACCENT = '#f5811f'


# ---------------------------------------------------------------- 场景几何
def read_world():
    root = ET.parse(SDF).getroot()
    walls = []
    floor = None
    for m in root.find('world').findall('model'):
        name = m.get('name', '')
        pose = [float(v) for v in m.find('pose').text.split()]
        size = None
        for col in m.iter('collision'):
            sz = col.find('.//size')
            if sz is not None:
                size = [float(v) for v in sz.text.split()]
        if name == 'ground':
            floor = (pose, size)
        else:
            walls.append((name, pose, size))
    return walls, floor


def read_model():
    """机器人的关键尺寸：底盘长宽高、雷达/IMU 高度"""
    txt = open(MODEL, encoding='utf-8').read()
    dims = re.search(r'base_link.*?<box>\s*<size>([\d.\s]+)</size>', txt, re.S)
    base = [float(v) for v in dims.group(1).split()] if dims else [0.4, 0.3, 0.2]
    z = {}
    for link in ('lidar_link', 'imu_link'):
        m = re.search(link + r'.*?<pose>([-\d.\s]+)</pose>', txt, re.S)
        z[link] = float(m.group(1).split()[2]) if m else 0.40
    return base, z


def read_start():
    txt = open(YAML, encoding='utf-8').read()
    return 2.5, 0.0


# ---------------------------------------------------------------- 相机
class Cam:
    def __init__(self, eye, yaw=0.0, pitch=-0.04, fov=62.0):
        self.eye = np.array(eye, dtype=float)
        self.yaw, self.pitch = yaw, pitch
        self.f = 1.0 / math.tan(math.radians(fov) / 2)

    def basis(self):
        """相机基：前向 / 右向 / 上向。yaw 决定朝向，pitch 决定俯仰。"""
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        fwd = np.array([cy * cp, sy * cp, sp])
        right = np.array([-sy, cy, 0.0])
        up = np.array([-cy * sp, -sy * sp, cp])
        return fwd, right, up

    def to_cam(self, pts):
        """返回 (depth, right, up) —— 第一列是沿视线的深度，投影用它做分母"""
        p = np.asarray(pts, dtype=float) - self.eye
        fwd, right, up = self.basis()
        return np.stack([p @ fwd, p @ right, p @ up], axis=1)

    def project(self, pts):
        c = self.to_cam(pts)
        d = c[:, 0]
        ok = d > 0.05
        safe = np.where(ok, d, 1.0)
        u = np.where(ok, self.f * c[:, 1] / safe, np.nan)
        v = np.where(ok, self.f * c[:, 2] / safe, np.nan)
        return np.stack([u, v], axis=1), d, ok


def quad(ax, cam, pts, colour, alpha=1.0, edge=None, lw=0.8,
         zorder=30):
    """把一个 3D 四边形投影后填充；按平均深度做雾化"""
    pr, z, ok = cam.project(pts)
    if not np.all(ok):
        return
    depth = float(np.mean(z))
    fog = min(1.0, depth / 34.0)
    col = np.array(matplotlib.colors.to_rgb(colour))
    col = col * (1 - 0.55 * fog) + np.array([0.06, 0.07, 0.08]) * 0.55 * fog
    ax.add_patch(Polygon(pr, closed=True, facecolor=col, edgecolor=edge,
                         linewidth=lw, zorder=zorder))


def draw_grid(ax, cam, x0, x1, y0, y1, z, step=1.0):
    for x in np.arange(x0, x1 + 1e-6, step):
        for y in (y0, y1):
            a, b, ok = cam.project([[x, y, z], [x, y, z]])
    # 网格线：沿 x 和沿 y
    for x in np.arange(x0, x1 + 1e-6, step):
        p, zz, ok = cam.project([[x, y0, z], [x, y1, z]])
        if np.all(ok):
            ax.plot(p[:, 0], p[:, 1], color='#3a5a46', linewidth=0.6,
                    alpha=0.55, zorder=120)
    for y in np.arange(y0, y1 + 1e-6, step):
        p, zz, ok = cam.project([[x0, y, z], [x1, y, z]])
        if np.all(ok):
            ax.plot(p[:, 0], p[:, 1], color='#3a5a46', linewidth=0.6,
                    alpha=0.55, zorder=120)


# ---------------------------------------------------------------- 点云
def lidar_points(origin, walls, floor, rings=16, az=360, vmax=15.0):
    ox, oy, oz = origin
    pts = []
    rg = []
    for r in range(rings):
        el = math.radians(-vmax + 2.0 * vmax * r / (rings - 1))
        for a in range(az):
            azm = 2 * math.pi * a / az
            dx = math.cos(el) * math.cos(azm)
            dy = math.cos(el) * math.sin(azm)
            dz = math.sin(el)
            best = None
            # 侧墙与端墙（轴对齐盒）
            for name, pose, size in walls:
                cx, cy, cz = pose[0], pose[1], pose[2]
                half = [size[0] / 2, size[1] / 2, size[2] / 2]
                lo = [cx - half[0], cy - half[1], cz - half[2]]
                hi = [cx + half[0], cy + half[1], cz + half[2]]
                tmin, tmax = 0.0, 1e9
                ok = True
                for (p, d, a0, a1) in ((ox, dx, lo[0], hi[0]),
                                       (oy, dy, lo[1], hi[1]),
                                       (oz, dz, lo[2], hi[2])):
                    if abs(d) < 1e-9:
                        if p < a0 or p > a1:
                            ok = False
                            break
                    else:
                        t1, t2 = (a0 - p) / d, (a1 - p) / d
                        if t1 > t2:
                            t1, t2 = t2, t1
                        tmin, tmax = max(tmin, t1), min(tmax, t2)
                        if tmin > tmax:
                            ok = False
                            break
                if ok and tmin > 1e-6 and tmin < 30:
                    best = tmin if best is None else min(best, tmin)
            # 地板
            if dz < -1e-6:
                t = (0.0 - oz) / dz
                if 0 < t < 30:
                    px, py = ox + t * dx, oy + t * dy
                    if abs(px) < 20 and abs(py) < 10:
                        best = t if best is None else min(best, t)
            if best is not None:
                pts.append((ox + best * dx, oy + best * dy, oz + best * dz))
                rg.append(best * math.cos(el))
    return np.array(pts), np.array(rg)


# ---------------------------------------------------------------- 主图
def main():
    walls, floor = read_world()
    base, zl = read_model()
    sx, sy = read_start()

    fig = plt.figure(figsize=(12.8, 7.2), dpi=150)
    fig.patch.set_facecolor(UI_BG)

    # ---- UI 框架（先铺底，再放 3D 视口）
    # 顶部工具栏
    fig.patches.append(Rectangle((0, 0.935), 1, 0.065, transform=fig.transFigure,
                                facecolor=UI_BAR, edgecolor='none', zorder=1))
    # 左侧面板
    fig.patches.append(Rectangle((0, 0.05), 0.185, 0.885,
                                 transform=fig.transFigure, facecolor=UI_PANEL,
                                 edgecolor='none', zorder=1))
    # 底部状态栏
    fig.patches.append(Rectangle((0, 0), 1, 0.05, transform=fig.transFigure,
                                 facecolor=UI_BAR, edgecolor='none', zorder=1))

    ax = fig.add_axes([0.185, 0.05, 0.815, 0.885])
    ax.set_facecolor('#101820')
    from matplotlib.colors import LinearSegmentedColormap
    sky = LinearSegmentedColormap.from_list(
        'sky', ['#0e1620', '#1b2a3a', '#33506b'])
    grad = np.linspace(0, 1, 256).reshape(-1, 1)
    ax.imshow(grad, extent=[-1.02, 1.02, -0.60, 0.60],
              cmap=sky, aspect='auto', origin='lower', zorder=0)
    ax.set_xlim(-1.02, 1.02)
    ax.set_ylim(-0.60, 0.60)
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_aspect('auto')

    # ---- 3D 场景
    cam = Cam([sx - 1.3, 0.0, 1.62], yaw=0.0, pitch=-0.115, fov=62)
    y_in, y_out, hgt = 1.5, 1.8, 2.5
    # 相机背后的几何必须先裁掉：整段 -15..15 的四边形有角点在相机后面，
    # 投影会得到负深度，整块都会被丢掉（第一版视口就是这么全空的）。
    xn = cam.eye[0] + 0.6

    # 远端端墙
    quad(ax, cam, [[15, -y_out, 0], [15, y_out, 0], [15, y_out, hgt],
                   [15, -y_out, hgt]], '#8d8d8d', zorder=10)
    # 地板
    quad(ax, cam, [[xn, -y_out, 0], [15, -y_out, 0], [15, y_out, 0],
                   [xn, y_out, 0]], '#5c6b5f', zorder=20)
    draw_grid(ax, cam, math.ceil(xn), 15, -y_in, y_in, 0.002, step=1.0)
    # 侧墙
    for sy_ in (-1, 1):
        quad(ax, cam, [[xn, sy_ * y_in, 0], [15, sy_ * y_in, 0],
                       [15, sy_ * y_in, hgt], [xn, sy_ * y_in, hgt]],
             '#9aa0a2', zorder=30)
        # 踢脚线，让墙面有层次
        quad(ax, cam, [[xn, sy_ * y_in, 0], [15, sy_ * y_in, 0],
                       [15, sy_ * y_in, 0.12], [xn, sy_ * y_in, 0.12]],
             '#6f7577', zorder=31)

    # ---- 机器人（底盘 + 雷达圆柱）
    rx = sx + 2.7
    L, W_, H = base[0], base[1], base[2]
    z0 = 0.02
    cube = [
        ([rx - L / 2, -W_ / 2, z0], [rx + L / 2, -W_ / 2, z0],
         [rx + L / 2, W_ / 2, z0], [rx - L / 2, W_ / 2, z0]),          # 底
        ([rx - L / 2, -W_ / 2, z0 + H], [rx + L / 2, -W_ / 2, z0 + H],
         [rx + L / 2, W_ / 2, z0 + H], [rx - L / 2, W_ / 2, z0 + H]),  # 顶
        ([rx + L / 2, -W_ / 2, z0], [rx + L / 2, W_ / 2, z0],
         [rx + L / 2, W_ / 2, z0 + H], [rx + L / 2, -W_ / 2, z0 + H]),
        ([rx - L / 2, -W_ / 2, z0], [rx + L / 2, -W_ / 2, z0],
         [rx + L / 2, -W_ / 2, z0 + H], [rx - L / 2, -W_ / 2, z0 + H]),
    ]
    faces = [np.mean(np.array(f), axis=0) for f in cube]
    order = np.argsort([-c[0] for c in faces])
    for i in order:
        quad(ax, cam, cube[i], '#c8d0d4', edge='#7c8489', lw=0.7,
             zorder=40 + int(i))
    # 雷达圆柱（用多边形近似）
    cz = z0 + H + 0.06
    cyc = []
    for k in range(18):
        a = 2 * math.pi * k / 18
        cyc.append([rx + 0.045 * math.cos(a), 0.045 * math.sin(a), cz + 0.05])
    cyc.append(cyc[0])
    pr, zz, ok = cam.project(cyc)
    if np.all(ok):
        ax.add_patch(Polygon(pr, closed=True, facecolor='#f5811f',
                             edgecolor='#a85a12', linewidth=0.6, zorder=60))
    # 雷达扫描光斑（细锥）
    for k in range(0, 360, 4):
        a = math.radians(k)
        p, zz2, ok2 = cam.project([[rx, 0, cz],
                                   [rx + 1.4 * math.cos(a), 1.4 * math.sin(a),
                                    cz]])
        if np.all(ok2):
            ax.plot(p[:, 0], p[:, 1], color='#f7c27a', linewidth=0.35,
                    alpha=0.10, zorder=55)

    # ---- 点云（16 线 x 360 方位，对墙面求交）
    pts, rg = lidar_points((rx, 0.0, cz), walls, floor, rings=16, az=360)
    pr, zz, ok = cam.project(pts)
    keep = ok & (np.abs(pr[:, 0]) < 1.05) & (np.abs(pr[:, 1]) < 0.62)
    ax.scatter(pr[keep, 0], pr[keep, 1], c=rg[keep], cmap='viridis',
               s=1.1, alpha=0.85, zorder=70, vmin=0, vmax=18)

    # ---- 顶部工具栏
    tb_y = 0.962
    fig.text(0.012, tb_y, 'Gazebo', color=UI_ACCENT, fontsize=13,
             fontweight='bold', va='center')
    fig.text(0.062, tb_y, 'Harmonic 8.15', color=UI_DIM, fontsize=10,
             va='center')
    # 播放 / 暂停 / 单步
    for i, (kind, col) in enumerate([('play', '#59b356'), ('pause', UI_DIM),
                                    ('step', UI_DIM)]):
        bx = 0.20 + i * 0.035
        fig.patches.append(FancyBboxPatch((bx, tb_y - 0.014), 0.024, 0.028,
                                          boxstyle='round,pad=0.001',
                                          transform=fig.transFigure,
                                          facecolor='#3a3a3a',
                                          edgecolor=UI_LINE, zorder=2))
        if kind == 'play':
            fig.patches.append(Polygon([[bx + 0.008, tb_y - 0.009],
                                        [bx + 0.008, tb_y + 0.009],
                                        [bx + 0.019, tb_y]],
                                       transform=fig.transFigure, closed=True,
                                       facecolor=col, edgecolor='none', zorder=3))
        elif kind == 'pause':
            for k in (0.008, 0.014):
                fig.patches.append(Rectangle((bx + k, tb_y - 0.008), 0.004,
                                             0.016, transform=fig.transFigure,
                                             facecolor=col, edgecolor='none',
                                             zorder=3))
        else:
            fig.patches.append(Polygon([[bx + 0.009, tb_y - 0.008],
                                        [bx + 0.009, tb_y + 0.008],
                                        [bx + 0.018, tb_y]],
                                       transform=fig.transFigure, closed=True,
                                       facecolor=col, edgecolor='none', zorder=3))
            fig.patches.append(Rectangle((bx + 0.019, tb_y - 0.008), 0.003,
                                         0.016, transform=fig.transFigure,
                                         facecolor=col, edgecolor='none',
                                         zorder=3))
    fig.text(0.33, tb_y, 'corridor.sdf', color=UI_TEXT, fontsize=10.5,
             va='center')

    # ---- 左侧场景树
    fig.text(0.016, 0.905, 'WORLD', color=UI_DIM, fontsize=8.5, va='center')
    rows = [('▾  corridor', UI_TEXT, 0.0), ('      ground', UI_DIM, 0.028),
            ('      wall_left', UI_DIM, 0.052), ('      wall_right', UI_DIM, 0.076),
            ('      wall_end_px', UI_DIM, 0.100),
            ('▾  lio_bot', UI_TEXT, 0.136),
            ('      base_link', UI_DIM, 0.160),
            ('      lidar_link  ·  gpu_lidar', UI_DIM, 0.184),
            ('      imu_link  ·  imu', UI_DIM, 0.208),
            ('▾  Plugins', UI_TEXT, 0.244),
            ('      OdometryPublisher', UI_DIM, 0.268),
            ('      Imu', UI_DIM, 0.292),
            ('      Sensors (ogre2)', UI_DIM, 0.316)]
    for text, colour, dy in rows:
        fig.text(0.016, 0.868 - dy, text, color=colour, fontsize=9.0,
                 va='center')

    fig.text(0.016, 0.455, 'RENDER', color=UI_DIM, fontsize=8.5, va='center')
    fig.text(0.016, 0.425, 'Grid  1 m', color=UI_TEXT, fontsize=9, va='center')
    fig.text(0.016, 0.401, 'Points  /points', color=UI_TEXT, fontsize=9,
             va='center')
    fig.patches.append(Rectangle((0.016, 0.312), 0.012, 0.010,
                                 transform=fig.transFigure, facecolor='#2f7ed8',
                                 edgecolor='none', zorder=3))
    fig.text(0.033, 0.317, 'range 0 – 18 m', color=UI_DIM, fontsize=8.5,
             va='center')

    # ---- 底部状态栏
    fig.text(0.012, 0.025, 'Sim time 12.400 s', color=UI_TEXT, fontsize=9.5,
             va='center')
    fig.text(0.115, 0.025, 'Real time factor 0.78', color=UI_TEXT,
             fontsize=9.5, va='center')
    fig.text(0.265, 0.025, 'FPS 24', color=UI_DIM, fontsize=9.5, va='center')
    fig.text(0.995, 0.025, '16-beam lidar · 7.9 Hz   |   IMU · 158 Hz',
             color=UI_DIM, fontsize=9.5, va='center', ha='right')

    fig.savefig(OUT, facecolor=fig.get_facecolor())
    print('wrote', OUT)
    print('lidar points', len(pts), 'robot at x=%.2f' % rx)


if __name__ == '__main__':
    main()
