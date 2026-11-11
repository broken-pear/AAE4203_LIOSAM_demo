#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lio_sam_nodes.launch.py —— 只起 LIO-SAM 的 4 个节点 + 3 个静态 TF
===============================================================================
被 bench_replay.launch.py（离线回放）和 compare_gazebo.launch.py（同屏对比）复用。

⚠️ 与 lio_sam_gazebo.launch.py 里的节点定义**必须保持一致**（Demo 2 是已经在
Jetson 上实测通过的版本，故意不去动它）。改这里的时候记得同步改那边。

为什么这 3 个静态 TF 在回放模式下也必须有：
  * map -> odom             LIO-SAM 自己不发（官方 run.launch.py 也是外部补的）
  * base_link -> lidar_link  TransformFusion 每一帧都要 lookup 这个变换
  * base_link -> imu_link    同上（外参）
没有它们，TransformFusion 会一直报 TF lookup 失败，odom->base_link 就不发了。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf(name, x, y, z, roll, pitch, yaw, parent, child, use_sim_time):
    """
    静态 TF **不要**跟着仿真时钟走（use_sim_time 强制 False）。

    踩过的坑：三个静态变换原本设了 use_sim_time:=true，于是它们要等 `/clock`
    出现才发布 —— 而 `/clock` 是 `ros2 bag play --clock` 起来后才有的。
    结果第一批点云（sim time 0.2 s）到达时，tf2 缓冲区里最早的变换是 0.44 s，
    lio_sam_imuPreintegration 查 base_link→lidar_link 直接报：

        Could not find a connection between 'lidar_link' and 'base_link'
        Tf has two or more unconnected trees.
        Lookup would require extrapolation into the past: requested 0.2,
        earliest data is at 0.44

    拿不到外参 -> IMU 初始化错 -> 因子图秩亏 -> GTSAM 抛
    IndeterminantLinearSystemException（我们追了好几轮的"崩溃"其实是它的下游症状）。

    静态变换与时间无关（tf2 里对 /tf_static 的查询对所有时刻都有效），
    所以这里用墙钟发布，launch 一起就能查到。
    """
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )


def generate_launch_description():
    pkg = get_package_share_directory('lio_sam_gz_bringup')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_params = os.path.join(pkg, 'config', 'params_gazebo.yaml')

    lio_nodes = [
        Node(package='lio_sam', executable='lio_sam_imuPreintegration',
             name='lio_sam_imuPreintegration',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='lio_sam', executable='lio_sam_imageProjection',
             name='lio_sam_imageProjection',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='lio_sam', executable='lio_sam_featureExtraction',
             name='lio_sam_featureExtraction',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
        Node(package='lio_sam', executable='lio_sam_mapOptimization',
             name='lio_sam_mapOptimization',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             output='screen'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='LIO-SAM 参数文件'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        _static_tf('tf_map_to_odom', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   'map', 'odom', use_sim_time),
        _static_tf('tf_base_to_lidar', 0.0, 0.0, 0.28, 0.0, 0.0, 0.0,
                   'base_link', 'lidar_link', use_sim_time),
        _static_tf('tf_base_to_imu', 0.0, 0.0, 0.28, 0.0, 0.0, 0.0,
                   'base_link', 'imu_link', use_sim_time),

        *lio_nodes,
    ])
