#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_bag.launch.py —— 录一份"干净"的原始数据 bag（公平实验的数据源）
===============================================================================
一次录制的产物就是整个实验的**唯一数据源**：之后 LOAM 和 LIO-SAM 都从这一份
bag 回放，所以两条算法的输入逐字节相同。

    ros2 launch lio_sam_bench record_bag.launch.py \
        world:=$(ros2 pkg prefix lio_sam_gz_bringup)/share/lio_sam_gz_bringup/worlds/lab1_room.sdf \
        driver_params:=experiments/scenarios/s0_nominal.yaml \
        out:=experiments/data/s0_nominal

录什么（**注意两个"不录"**）
------------------------------
    /points     16 线点云（带 ring/time）
    /imu/data   200 Hz IMU
    /gt/odom    世界系真值（评测用）

不录 `/clock`：回放时用 `ros2 bag play --clock` 统一发时钟。
如果 bag 里也有一份 /clock，回放时就有两个时钟源，时间戳会来回跳
（这正是 TROUBLESHOOTING 里 C6 那个"两个仿真同时在跑"的现象）。

不录 `/tf` `/tf_static`：**录制阶段根本没有节点在发它们**。
  * 车上的 DiffDrive 插件设了 publish_odom_tf=false
  * 那 3 个静态 TF（map→odom、base→lidar_link、base→imu_link）是
    lio_sam_nodes.launch.py 在**回放时**才起的
如果硬要录，`ros2 bag record` 会为一个没有发布者的话题干等，
而 lio_sam_bench/scripts/check_bag.py 也会因为 /tf_static 空而判录制失败。
真值/点云都带 frame_id，回放侧自带静态 TF，所以这两路本来就不需要进包。

启动什么
--------
    gazebo_lab1.launch.py   （rviz:=false, drive:=none —— 驱动由 scripted_driver 负责）
    scripted_driver.py      （按 driver_params 里的分段脚本开车）
    ros2 bag record         （写到 out 目录）

脚本控制停止：run_bench.sh 会用超时发送 SIGINT，bag 会被正常收尾。
手动跑的时候按 Ctrl-C 也一样。

参数
----
    world          场景 SDF 路径
    out            输出目录（必须不存在，或为空；rosbag2 不会覆盖已有包）
    driver_params  scripted_driver 的参数文件（experiments/scenarios/*.yaml）
    use_sim_time   true
    start_gazebo   true   关掉可以用来在已有 Gazebo 上只录数据
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 只录"录制阶段真的有节点在发"的话题。
# /tf 与 /tf_static 故意不在列表里：车上 DiffDrive 关掉了 publish_odom_tf，
# 静态 TF 是回放时才起的（见 lio_sam_nodes.launch.py），录制阶段录它们只会得到空话题。
# 这个列表和 experiments/check_bag.py 的 REQUIRED 必须保持一致（_selfcheck.py 会核对）。
BAG_TOPICS = ['/points', '/imu/data', '/gt/odom']


def generate_launch_description():
    bringup_pkg = get_package_share_directory('lio_sam_gz_bringup')

    world = LaunchConfiguration('world')
    out = LaunchConfiguration('out')
    driver_params = LaunchConfiguration('driver_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_gazebo = LaunchConfiguration('start_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'gazebo_lab1.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'drive': 'none',          # 车由 scripted_driver 控制，不用 patrol_driver
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
        condition=IfCondition(start_gazebo),
    )

    # 为什么不用 parameters=[driver_params]（把 yaml 当 --params-file 传给节点）：
    # 实测在真机上这个节点的 rclpy.init() 会以 "Failed to parse global arguments"
    # 直接崩掉（日志里只给这一句，不说是哪个参数）。所以这里改成在 launch 里
    # **读好 yaml、以 inline dict 传参**：既没有 --params-file，也就没有那类解析问题，
    # 而且能把最终参数打进日志，出问题时一眼能看到驱动到底拿到了什么。
    def _make_driver(context, *args, **kwargs):
        path = LaunchConfiguration('driver_params').perform(context)
        sim = str(LaunchConfiguration('use_sim_time').perform(context)).lower() in ('true', '1')
        params = {'use_sim_time': sim}
        try:
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            drv = (data.get('scripted_driver', {}) or {}).get('ros__parameters', {}) or {}
        except Exception as e:                                # noqa: BLE001
            # 这里**必须**直接失败：拿默认参数跑出来的是一条"看起来正常、
            # 其实和场景不符"的轨迹，那种数据比没有数据更糟。
            raise RuntimeError(
                f'[record_bag] 读不了 driver_params={path}: {e}\n'
                f'           （这个文件决定车怎么走；读不到就不录，免得录出一条假轨迹）')
        if not drv:
            raise RuntimeError(
                f'[record_bag] {path} 里没有 scripted_driver.ros__parameters —— '
                f'场景文件格式不对？')
        params.update(drv)
        print(f'[record_bag] scripted_driver 参数 = {params}')
        return [Node(
            package='lio_sam_gz_bringup',
            executable='scripted_driver.py',
            name='scripted_driver',
            parameters=[params],
            output='screen',
        )]

    driver = OpaqueFunction(function=_make_driver)

    # --storage mcap 还是 sqlite3？默认 sqlite3，Jetson 上最稳，且 rosbag2 不需要额外插件
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out] + BAG_TOPICS,
        output='screen',
    )

    # 话题看门狗：三路数据在 give_up_s 内没到齐，它就带着诊断表退出（rc=1），
    # 下面的 on_exit 立刻关掉整条 launch —— 不用等到 102 s 超时才失败。
    # 成功时它不退出（否则 on_exit 会在数据刚到时误杀录制）。
    watchdog = Node(
        package='lio_sam_bench',
        executable='topic_watchdog.py',
        name='topic_watchdog',
        parameters=[{'give_up_s': 45.0, 'use_sim_time': False}],
        output='screen',
    )

    stop_on_watchdog = RegisterEventHandler(
        OnProcessExit(
            target_action=watchdog,
            on_exit=[EmitEvent(event=Shutdown(
                reason='recording produced no data — see topic_watchdog report'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_pkg, 'worlds', 'lab1_room.sdf')),
        DeclareLaunchArgument('out', default_value='experiments/data/bag'),
        DeclareLaunchArgument(
            'driver_params', default_value='experiments/scenarios/s0_nominal.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),

        gazebo,
        driver,
        watchdog,
        stop_on_watchdog,
        bag_record,
    ])
