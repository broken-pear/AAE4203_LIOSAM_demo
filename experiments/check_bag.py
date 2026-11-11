#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_bag.py —— 录完立刻验：这份 bag 能不能用来评测
===============================================================================
为什么要这一步
--------------
录制"成功"不等于数据可用。最坑的一种情况是 `/gt/odom` 一条消息都没有：
rosbag2 照样写盘、`run_bench.sh` 照样往下跑、两条算法也照样给出轨迹，
直到最后 `evaluate.py` 报 "gt 或 est 为空" —— 那时已经浪费了两轮回放的时间，
而且错误信息指向的是评测器，不是真正的根因。

所以录完的 10 秒内就检查：话题在不在、条数够不够、时间跨度够不够，
每条都给出**根因提示**（而不是只说"失败"）。

    python3 experiments/check_bag.py experiments/data/s0_nominal
    python3 experiments/check_bag.py --from-file bag_info.txt   # 离线验解析逻辑

退出码 0 = 可用；1 = 不可用（run_bench.sh 会跳过这个场景）。
"""

import argparse
import re
import subprocess
import sys

# 话题 -> (最少条数, 不够时的根因提示)
REQUIRED = {
    '/points': (
        50,
        '点云没进 bag。先确认 gazebo_lab1.launch.py 单独能跑，且 '
        'ros2 topic hz /points_raw ≈ 10 Hz；桥接的 gz 话题必须是 /points_raw/points'),
    '/imu/data': (
        1000,
        'IMU 没进 bag。world 里需要 <plugin filename="gz-sim-imu-system">，'
        '缺它的话题会存在但永远不发数据'),
    '/gt/odom': (
        50,
        '真值没进 bag —— **评测的 ground truth 全靠它**。检查 model.sdf 里有没有 '
        'gz-sim-odometry-publisher-system 插件，以及 gz_bridge.yaml 里 /gt/odom 的桥接'),
    '/tf_static': (
        1,
        '静态 TF 没进 bag。LIO-SAM 的 TransformFusion 每帧都要 lookup '
        'base_link->lidar_link，缺了它 odom->base_link 就完全不发'),
}

MIN_DURATION_S = 30.0

_TOPIC_RE = re.compile(r'Topic:\s*(\S+)\s*\|\s*Type:\s*(\S+)\s*\|\s*Count:\s*(\d+)')
_DUR_RE = re.compile(r'^\s*Duration:\s*([0-9.]+)\s*s', re.M)


def parse_info(text):
    """从 `ros2 bag info` 的输出里抠出 (话题 -> 条数) 和时长。"""
    topics = {}
    for name, _type, count in _TOPIC_RE.findall(text):
        topics[name] = int(count)
    m = _DUR_RE.search(text)
    duration = float(m.group(1)) if m else None
    return topics, duration


def get_info(bag_path):
    try:
        p = subprocess.run(['ros2', 'bag', 'info', bag_path],
                           capture_output=True, text=True, timeout=120)
    except FileNotFoundError:
        return None, '找不到 ros2 命令（先 source /opt/ros/jazzy/setup.bash）'
    except subprocess.TimeoutExpired:
        return None, 'ros2 bag info 超时'
    if p.returncode != 0:
        return None, (p.stderr or p.stdout).strip()[:400]
    return p.stdout, None


def main(argv=None):
    ap = argparse.ArgumentParser(description='校验录好的 bag 是否可以用于评测')
    ap.add_argument('bag', nargs='?', default=None, help='rosbag2 目录')
    ap.add_argument('--from-file', default=None,
                    help='不调用 ros2，直接解析这个文本文件（离线验解析逻辑用）')
    ap.add_argument('--min-duration', type=float, default=MIN_DURATION_S)
    args = ap.parse_args(argv)

    if args.from_file:
        try:
            with open(args.from_file, encoding='utf-8') as f:
                text = f.read()
        except OSError as e:
            print(f'[FAIL] 读不了 {args.from_file}: {e}')
            return 1
        label = args.from_file
    else:
        if not args.bag:
            ap.error('要么给 bag 目录，要么给 --from-file')
        text, err = get_info(args.bag)
        if err:
            print(f'[FAIL] 读不了 bag: {err}')
            return 1
        label = args.bag

    topics, duration = parse_info(text)
    print(f'[bag] {label}')
    print(f'[bag] 时长 {duration if duration is not None else "?"} s, '
          f'{len(topics)} 个话题')

    fails = 0
    for name, (need, hint) in REQUIRED.items():
        got = topics.get(name, 0)
        if got >= need:
            print(f'  [ OK ] {name:12s} {got:>7d} 条')
        else:
            fails += 1
            print(f'  [FAIL] {name:12s} {got:>7d} 条（至少要 {need}）')
            print(f'         根因：{hint}')

    if duration is not None and duration < args.min_duration:
        fails += 1
        print(f'  [FAIL] 时长 {duration:.1f}s < {args.min_duration:.0f}s：'
              f'录制被提前中断了（Gazebo 崩了？超时参数太小？）')

    # 顺带把没用的、体积大的话题点出来
    for name, count in sorted(topics.items()):
        if name not in REQUIRED and name not in ('/tf', '/clock'):
            print(f'  [info] 额外话题 {name} ({count} 条)')

    if fails:
        print(f'\n[bag] 不可用：{fails} 项不达标 —— 别急着跑算法，先修上面这些')
        return 1
    print('\n[bag] 可用')
    return 0


if __name__ == '__main__':
    sys.exit(main())
