#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bench_replay.launch.py —— 同一份 bag，分别喂给两种算法，录成 CSV
===============================================================================
这是公平实验的执行单元。一次运行 = 一条算法 × 一个场景，输入是**同一个 bag**：

    ros2 launch lio_sam_bench bench_replay.launch.py \
        bag:=experiments/data/s0_nominal \
        algo:=loam \
        out_csv:=results/raw/s0_nominal/loam.csv

它启动什么：
    1. ros2 bag play <bag> --clock --rate <rate>      ← 唯一的传感器/真值来源
    2. gt_sync.py                                     ← 真值对齐到点云时间戳
    3. traj_recorder.py                               ← 逐帧写 CSV
    4. 算法节点（algo:=loam 起 A-LOAM；algo:=liosam 起 LIO-SAM）

为什么用"回放同一个 bag"而不是"跑两次 Gazebo"
------------------------------------------------
跑两次 Gazebo 时，两次的传感器噪声、起点、速度积分都不完全一样，
两条算法的输入就不同了，ATF 差异会混入"输入差异"。回放同一个 bag 则保证：

    两条算法看到的点云流 **逐字节相同**
    两条算法对齐到的真值   **逐帧相同**

于是结果差异只能来自算法本身。这也是论文里做消融实验的标准做法。

时间戳
------
回放用 `--clock`，所有节点 use_sim_time:=true。bag 里**必须没有** /clock
（实验脚本录制时故意排除了它），否则会出现两个时钟源。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# EqualsSubstitution 是较新的 launch API（Jazzy 有）。为了在稍老的发行版上也能跑，
# 这里做一个退化：没有它就用已弃用但仍可用的 LaunchConfigurationEquals。
# 只是为了"选哪条算法"这一个判断，不值得让整份实验因为一个 API 缺失而起不来。
try:
    from launch.substitutions import EqualsSubstitution

    def algo_is(value):
        return IfCondition(EqualsSubstitution(LaunchConfiguration('algo'), value))
except ImportError:                                        # pragma: no cover
    from launch.conditions import LaunchConfigurationEquals

    def algo_is(value):
        return LaunchConfigurationEquals('algo', value)


# 两条算法各自"要评测的位姿话题"
#   LIO-SAM : /lio_sam/mapping/odometry     = 因子图优化后的位姿
#   A-LOAM  : /aft_mapped_to_init           = scan-to-map 优化后的位姿
# 都取"建图后"的位姿，不用纯里程计，这样比的是最终输出。
EST_TOPIC = {
    'liosam': '/lio_sam/mapping/odometry',
    'loam': '/aft_mapped_to_init',
}


def generate_launch_description():
    bench_pkg = get_package_share_directory('lio_sam_bench')
    bringup_pkg = get_package_share_directory('lio_sam_gz_bringup')

    bag = LaunchConfiguration('bag')
    algo = LaunchConfiguration('algo')
    out_csv = LaunchConfiguration('out_csv')
    rate = LaunchConfiguration('rate')
    start_offset = LaunchConfiguration('start_offset')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')

    gt_params = os.path.join(bench_pkg, 'config', 'bench.yaml')

    # ---------------------------------------------------------------- bag 回放
    # 注意：不加 /clock 的第二个来源 —— bag 里本来就没有 /clock，
    # 由 rosbag2 的 --clock 统一发。两个时钟源会让 TF 时间戳乱跳。
    # start_offset > 0 时从包的中间开始放。
    # 用途：LIO-SAM 的 IMU 预积分在"完全静止起步"那一段可能出现
    # 加速度计偏置与俯仰角互相不可分辨（秩亏），跳过起步静止段即可避开。
    # 注意：两条算法必须用同一个 offset，否则就不是同一份数据了 ——
    # run_bench.sh 会把同一个值传给两次回放。
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate,
             '--start-offset', start_offset],
        output='screen',
    )

    # ------------------------------------------------------------- 真值 + 录制
    # 不用 parameters=[<yaml 文件>]：真机上 rclpy 节点走 --params-file 这条路时，
    # rclpy.init() 会以 "Failed to parse global arguments" 直接崩（scripted_driver
    # 就是这么死的）。这里在 launch 里把 bench.yaml 读进来、以 inline dict 传参，
    # 既保留"配置只有一个来源"，又不经过 --params-file。
    def _make_gt_sync(context, *args, **kwargs):
        sim = str(LaunchConfiguration('use_sim_time').perform(context)).lower() in ('true', '1')
        params = {'use_sim_time': sim}
        path = os.path.join(bench_pkg, 'config', 'bench.yaml')
        try:
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            params.update((data.get('gt_sync', {}) or {}).get('ros__parameters', {}) or {})
        except Exception as e:                                # noqa: BLE001
            print(f'[bench_replay] 读不了 {path}: {e} —— gt_sync 用默认参数')
        print(f'[bench_replay] gt_sync 参数 = {params}')
        return [Node(
            package='lio_sam_bench',
            executable='gt_sync.py',
            name='gt_sync',
            parameters=[params],
            output='screen',
        )]

    gt_sync = OpaqueFunction(function=_make_gt_sync)

    recorder_common = {
        'gt_topic': '/gt/pose_sync',
        'cloud_topic': '/points',
        'out_csv': out_csv,
        'use_sim_time': use_sim_time,
        # 记进 meta：两条算法必须用同一个 offset，sanity_check 会核对
        'bag_start_offset': start_offset,
    }

    recorder_loam = Node(
        package='lio_sam_bench',
        executable='traj_recorder.py',
        name='traj_recorder',
        parameters=[dict(recorder_common,
                         algo='loam',
                         est_topic=EST_TOPIC['loam'])],
        condition=algo_is('loam'),
        output='screen',
    )

    recorder_liosam = Node(
        package='lio_sam_bench',
        executable='traj_recorder.py',
        name='traj_recorder',
        parameters=[dict(recorder_common,
                         algo='liosam',
                         est_topic=EST_TOPIC['liosam'])],
        condition=algo_is('liosam'),
        output='screen',
    )

    # ---------------------------------------------------------------- 算法节点
    algo_loam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'loam_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=algo_is('loam'),
    )

    algo_liosam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'lio_sam_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
        condition=algo_is('liosam'),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bench_pkg, 'rviz', 'slam_compare.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
        output='screen',
    )

    # 先让各节点建立订阅、静态 TF 进入缓冲区，再放 bag。
    # 原来 3s 太紧：LIO-SAM 的 imuPreintegration 在头几帧就要查 base_link→lidar_link，
    # 查不到会打印 "two or more unconnected trees"，进而让 IMU 初始化失效。
    delayed_bag = TimerAction(period=6.0, actions=[bag_play])

    # bag 放完 -> 自动收工（recorder 在 SIGINT 里会把 meta.json 写完）
    stop_on_bag_end = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[EmitEvent(event=Shutdown(
                reason='bag play finished'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='rosbag2 目录（experiments/data/<scenario>）'),
        DeclareLaunchArgument('algo', default_value='loam',
                              description='loam / liosam'),
        DeclareLaunchArgument('out_csv',
                              default_value='results/raw/traj.csv',
                              description='输出 CSV 路径'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='bag 回放倍速；Jetson 上跟不上就调小（如 0.7）'),
        DeclareLaunchArgument('start_offset', default_value='0.0',
                              description='从包的第几秒开始放（跳过起步静止段用）'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_pkg, 'config', 'params_gazebo.yaml')),

        stop_on_bag_end,
        gt_sync,
        recorder_loam,
        recorder_liosam,
        algo_loam,
        algo_liosam,
        rviz_node,
        delayed_bag,
    ])
