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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
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

    driver = Node(
        package='lio_sam_gz_bringup',
        executable='scripted_driver.py',
        name='scripted_driver',
        parameters=[driver_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # --storage mcap 还是 sqlite3？默认 sqlite3，Jetson 上最稳，且 rosbag2 不需要额外插件
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out] + BAG_TOPICS,
        output='screen',
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
        bag_record,
    ])
