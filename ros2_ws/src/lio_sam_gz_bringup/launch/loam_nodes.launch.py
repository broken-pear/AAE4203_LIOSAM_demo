#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loam_nodes.launch.py —— 只起 A-LOAM 的三个节点（不含 Gazebo、不含 RViz）
===============================================================================
被两处复用：
    loam_gazebo.launch.py     Gazebo 在线跑（Demo 3）
    bench_replay.launch.py    离线回放 bag 跑（正式实验）

参数与 LIO-SAM 侧的对应关系见 loam_gazebo.launch.py 文件头的对齐表。
这两处共用一份节点定义，避免"在线跑"和"回放跑"的参数悄悄不一致。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ALOAM_PKG = 'aloam_velodyne_ros2'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_line = LaunchConfiguration('scan_line')
    min_range = LaunchConfiguration('minimum_range')
    skip_frame = LaunchConfiguration('mapping_skip_frame')
    line_res = LaunchConfiguration('mapping_line_resolution')
    plane_res = LaunchConfiguration('mapping_plane_resolution')

    scan_registration = Node(
        package=ALOAM_PKG,
        executable='ascanRegistration',
        name='ascanRegistration',
        parameters=[{
            'scan_line': scan_line,          # int
            'minimum_range': min_range,      # double
            'use_sim_time': use_sim_time,
        }],
        # 上游把话题名写死成 /velodyne_points；我们的雷达发 /points。
        # 用 remap 而不是改源码，保证 baseline 就是原版 LOAM。
        remappings=[('/velodyne_points', '/points')],
        output='screen',
    )

    laser_odometry = Node(
        package=ALOAM_PKG,
        executable='alaserOdometry',
        name='alaserOdometry',
        parameters=[{
            'mapping_skip_frame': skip_frame,   # int，1 = 每帧都送给建图
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    laser_mapping = Node(
        package=ALOAM_PKG,
        executable='alaserMapping',
        name='alaserMapping',
        parameters=[{
            'mapping_line_resolution': line_res,     # double（patch_aloam.sh 改过类型）
            'mapping_plane_resolution': plane_res,   # double
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_line', default_value='16'),
        DeclareLaunchArgument('minimum_range', default_value='0.3'),
        DeclareLaunchArgument('mapping_skip_frame', default_value='1'),
        DeclareLaunchArgument('mapping_line_resolution', default_value='0.1'),
        DeclareLaunchArgument('mapping_plane_resolution', default_value='0.2'),
        scan_registration,
        laser_odometry,
        laser_mapping,
    ])
