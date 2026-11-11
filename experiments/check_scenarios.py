#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_scenarios.py —— 离线验证每个场景"开得通"：不撞墙、不出界
===============================================================================
为什么值得单独跑
----------------
场景脚本（experiments/scenarios/*.yaml）只是几个 (v, ω, 秒数)。如果某一段把车
带进方柱或者墙角，Gazebo 里会发生碰撞：车被推开、真值跳变、点云退化。
那一段数据既不能算 LOAM 的错，也不能算 LIO-SAM 的错 —— 整个场景白跑一小时。

这个脚本不启动 Gazebo，直接用刚体运动学把轨迹积分出来，再用**二维 OBB 的
分离轴定理（SAT）**判断车体（0.50 × 0.34 m，再加安全裕度）有没有和障碍物重叠。

    python3 experiments/check_scenarios.py                 # 全部场景
    python3 experiments/check_scenarios.py --scenarios s2_high_speed --verbose

退出码 0 = 全部开得通；1 = 有场景撞了。

建模假设（都在报告里可写清楚）
------------------------------
  * 无滑移：x' = v·cosψ, y' = v·sinψ, ψ' = ω（差速车在小速度下够准）
  * 第一段有软启动斜坡（和 scripted_driver.py 的实现一致）
  * 忽略惯性/悬挂：车不会"漂"过墙角
  * 障碍物只取 `<collision><box>`，且只取顶面高于地面的（地面本身不算障碍）
"""

import argparse
import glob
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_gz_bringup', 'worlds')
SCEN = os.path.join(ROOT, 'experiments', 'scenarios')

ROBOT_LEN = 0.50          # model.sdf: base_link collision 0.50 x 0.34
ROBOT_WID = 0.34
SPAWN = (2.5, 0.0, 0.0)   # gazebo_lab1.launch.py 里的 spawn 位姿


# ------------------------------------------------------------------ 几何
def rect(cx, cy, sx, sy, yaw):
    """→ 四个角点（逆时针），世界系。"""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for lx, ly in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2),
                   (sx / 2, sy / 2), (-sx / 2, sy / 2)):
        pts.append((cx + c * lx - s * ly, cy + s * lx + c * ly))
    return pts


def _axes(pts):
    """多边形的可选分离轴（每条边的法向）。矩形只需要两条。"""
    out = []
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        ex, ey = x2 - x1, y2 - y1
        n = math.hypot(ex, ey)
        if n > 1e-9:
            out.append((-ey / n, ex / n))
    return out


def _proj(pts, ax):
    vals = [p[0] * ax[0] + p[1] * ax[1] for p in pts]
    return min(vals), max(vals)


def overlap(poly_a, poly_b):
    """SAT：两个凸多边形是否有重叠面积。"""
    for ax in _axes(poly_a) + _axes(poly_b):
        a0, a1 = _proj(poly_a, ax)
        b0, b1 = _proj(poly_b, ax)
        if a1 < b0 or b1 < a0:
            return False
    return True


def point_seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def clear_bound(center, obstacles):
    """车心到最近障碍边界的距离（下界意义：实际车体比车心更近）。"""
    best = float('inf')
    for poly in obstacles:
        for i in range(len(poly)):
            d = point_seg_dist(center[0], center[1], *poly[i], *poly[(i + 1) % len(poly)])
            best = min(best, d)
    return best


# ------------------------------------------------------------------ 解析
def parse_world(path):
    """→ 障碍物矩形列表 [(poly, name)]，跳过地面和顶面低于 0 的东西。"""
    root = ET.parse(path).getroot()
    world = root.find('world')
    if world is None:
        world = root
    out = []
    for model in world.findall('model'):
        name = model.get('name', '?')
        pose = model.find('pose')
        px = py = pz = ry = 0.0
        if pose is not None and pose.text:
            v = [float(t) for t in pose.text.split()]
            if len(v) >= 6:
                px, py, pz = v[0], v[1], v[2]
                ry = v[5]
        for coll in model.iter('collision'):
            cpose = coll.find('pose')
            ox = oy = oyaw = 0.0
            if cpose is not None and cpose.text:
                v = [float(t) for t in cpose.text.split()]
                if len(v) >= 6:
                    ox, oy = v[0], v[1]
                    oyaw = v[5]
            box = coll.find('geometry/box/size')
            if box is None or not box.text:
                continue
            sx, sy, sz = [float(t) for t in box.text.split()]
            if pz + sz / 2 <= 0.02:        # 地面之类，车在上面跑不算障碍
                continue
            # 碰撞体在模型系里的偏移（本 demo 的 world 里基本都是 0）
            c, s = math.cos(ry), math.sin(ry)
            cx = px + c * ox - s * oy
            cy = py + s * ox + c * oy
            out.append((rect(cx, cy, sx, sy, ry + oyaw), name))
    return out


def load_scenario(path):
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    drv = (d.get('scripted_driver', {}) or {}).get('ros__parameters', {}) or {}
    sc = (d.get('scenario', {}) or {}).get('ros__parameters', {}) or {}
    return drv, sc


# ------------------------------------------------------------------ 积分
def trace(drv, dt=0.01):
    """按 scripted_driver.py 的规则积分，返回轨迹点列表 [(t, x, y, yaw, v)]。"""
    v_list = [float(x) for x in drv.get('linear_speed') or []]
    w_list = [float(x) for x in drv.get('angular_speed') or []]
    d_list = [float(x) for x in drv.get('duration') or []]
    ramp = float(drv.get('ramp_time') or 0.0)
    if not (len(v_list) == len(w_list) == len(d_list)) or not v_list:
        raise ValueError('场景参数长度不一致')

    x, y, yaw = SPAWN
    pts = []
    t = 0.0
    for i, (v0, w0, dur) in enumerate(zip(v_list, w_list, d_list)):
        n = max(1, int(round(dur / dt)))
        for k in range(n):
            tt = t + k * dt
            k_ramp = min(tt / ramp, 1.0) if ramp > 0 and i == 0 else 1.0
            v = v0 * k_ramp
            w = w0 * k_ramp
            pts.append((tt, x, y, yaw, v))
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
        t += n * dt
    pts.append((t, x, y, yaw, 0.0))
    return pts


def check_scenario(name, worlds, margin=0.05, dt=0.01, verbose=False):
    drv, sc = load_scenario(os.path.join(SCEN, f'{name}.yaml'))
    world_file = sc.get('world')
    if not world_file:
        return {'name': name, 'error': '缺少 scenario.world'}
    world_path = world_file if os.path.isabs(world_file) else os.path.join(WORLDS, world_file)
    if not os.path.isfile(world_path):
        return {'name': name, 'error': f'找不到 world {world_file}'}

    obstacles = [poly for poly, _ in parse_world(world_path)]
    names = [n for _, n in parse_world(world_path)]
    pts = trace(drv, dt=dt)

    hits = {}
    min_clear = float('inf')
    path_len = 0.0
    prev = None
    for (t, x, y, yaw, v) in pts:
        body = rect(x, y, ROBOT_LEN + 2 * margin, ROBOT_WID + 2 * margin, yaw)
        for poly, nm in zip(obstacles, names):
            if overlap(body, poly):
                hits[nm] = hits.get(nm, 0) + 1
        min_clear = min(min_clear, clear_bound((x, y), obstacles))
        if prev is not None:
            path_len += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)

    return {
        'name': name,
        'world': os.path.basename(world_path),
        'duration': pts[-1][0] if pts else 0.0,
        'path_len': path_len,
        'hits': hits,
        'hit_steps': sum(hits.values()),
        'min_center_clear': min_clear,
        'radius': math.hypot(ROBOT_LEN, ROBOT_WID) / 2 + margin,
        'obstacles': len(obstacles),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description='离线验证场景轨迹是否撞墙/出界')
    ap.add_argument('--scenarios', nargs='+', default=None)
    ap.add_argument('--margin', type=float, default=0.05,
                    help='车体外扩裕度（米），默认 5 cm')
    ap.add_argument('--dt', type=float, default=0.01)
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args(argv)

    names = args.scenarios or sorted(
        os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(SCEN, '*.yaml')))

    print(f'{"场景":<16}{"world":<16}{"时长":>6}{"行程":>8}{"障碍":>5}'
          f'{"安全下界":>10}{"碰撞步":>8}   判定')
    print('-' * 86)
    bad = []
    for nm in names:
        r = check_scenario(nm, WORLDS, margin=args.margin, dt=args.dt,
                           verbose=args.verbose)
        if 'error' in r:
            print(f'{nm:<16}  {r["error"]}')
            bad.append(nm)
            continue
        lower = r['min_center_clear'] - r['radius']
        ok = r['hit_steps'] == 0
        verdict = 'OK' if ok else 'COLLISION'
        print(f'{r["name"]:<16}{r["world"]:<16}{r["duration"]:>6.1f}'
              f'{r["path_len"]:>8.1f}{r["obstacles"]:>5}{lower:>10.2f}'
              f'{r["hit_steps"]:>8}   {verdict}')
        if not ok:
            bad.append(nm)
            for k, v in sorted(r['hits'].items(), key=lambda kv: -kv[1]):
                print(f'{"":>16}↳ 与 {k} 重叠 {v} 步')

    print('-' * 86)
    print('安全下界 = 车心到最近障碍边界的距离 − 车体外接半径；'
          '负数只说明"按距离看已经不安全"，最终以 SAT 碰撞判定为准')
    if bad:
        print(f'\n{len(bad)} 个场景有碰撞/错误: {bad}')
        print('处理：调小 v、改掉头方式，或换 world。别带着碰撞去录包。')
        return 1
    print('\n全部场景都能开：车体（含裕度）不与任何障碍重叠。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
