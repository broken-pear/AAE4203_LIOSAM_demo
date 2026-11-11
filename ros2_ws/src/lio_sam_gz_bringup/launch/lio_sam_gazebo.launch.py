#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lio_sam_gazebo.launch.py —— Demo 2：Gazebo + LIO-SAM + RViz2 完整跑通

前提：已经按 README 编译好 LIO-SAM（ros2 分支），并且 source 过工作空间。

一条命令：
    ros2 launch lio_sam_gz_bringup lio_sam_gazebo.launch.py

它会：先跑 Demo 1 的全部内容（但不重复开 RViz），再加上：
    * 3 个静态 TF
        map       -> odom        （LIO-SAM 官方 run.launch.py 里就是这么干的）
        base_link -> lidar_link  （车体安装外参）
        base_link -> imu_link
    * LIO-SAM 的 4 个节点
        lio_sam_imuPreintegration   IMU 预积分 + 高频里程计 + odom->base_link TF
        lio_sam_imageProjection     点云去畸变 + 投影成距离图像
        lio_sam_featureExtraction   提取 edge / surf 特征
        lio_sam_mapOptimization     iSAM2 因子图优化 + map（全局地图）
    * RViz2（按 config/rviz/lio_sam_slam.rviz 打开）

跑通后 RViz 里应该看到：
    红色小球=TF 原点，白色/彩色点云=建出来的地图，绿线=轨迹
    Fixed Frame 是 map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf(name, x, y, z, roll, pitch, yaw, parent, child, use_sim_time):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )


def generate_launch_description():
    pkg = get_package_share_directory('lio_sam_gz_bringup')

    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('start_rviz')
    drive = LaunchConfiguration('drive')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    default_params = os.path.join(pkg, 'config', 'params_gazebo.yaml')
    slam_rviz = os.path.join(pkg, 'rviz', 'lio_sam_slam.rviz')

    # ------------------------------------------------ Demo 1 的全部内容（不含 RViz）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo_lab1.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'drive': drive,
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
    )

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', slam_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='LIO-SAM 参数文件'),
        DeclareLaunchArgument('start_rviz', default_value='true',
                              description='是否启动建图版 RViz2（注意：不是 gazebo_lab1 里的 rviz）'),
        DeclareLaunchArgument('drive', default_value='auto',
                              description='auto / teleop / none'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg, 'worlds', 'lab1_room.sdf')),

        gazebo,

        # LIO-SAM 只发 odom->lidar_link 和 odom->base_link，
        # 不会再发 map->odom，所以这里要自己补一个静态的（和官方 run.launch.py 一致）。
        _static_tf('tf_map_to_odom', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   'map', 'odom', use_sim_time),
        # 传感器的安装位置：model.sdf 里 lidar/imu 在 z=0.40，base_link 在 z=0.12
        _static_tf('tf_base_to_lidar', 0.0, 0.0, 0.28, 0.0, 0.0, 0.0,
                   'base_link', 'lidar_link', use_sim_time),
        _static_tf('tf_base_to_imu', 0.0, 0.0, 0.28, 0.0, 0.0, 0.0,
                   'base_link', 'imu_link', use_sim_time),

        *lio_nodes,

        rviz_node,
    ])
