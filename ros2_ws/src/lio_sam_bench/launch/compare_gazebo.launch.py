#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_gazebo.launch.py —— Demo 4：两条算法在**同一场仿真**里同时跑，同屏对比
===============================================================================
这是给报告/PPT 出图和录像用的"横评"模式：Gazebo 只开一次，/points 同时喂给
LOAM 和 LIO-SAM，两条轨迹 + 两张地图在同一个 RViz 里叠着看。

    ros2 launch lio_sam_bench compare_gazebo.launch.py

    # 只录数据不要界面（正式采集就用 run_bench.sh，不需要这个）
    ros2 launch lio_sam_bench compare_gazebo.launch.py start_rviz:=false

它会同时写出两份 CSV（可直接 evaluate.py）：
    <out_dir>/loam.csv      A-LOAM 的 /aft_mapped_to_init  + 真值
    <out_dir>/liosam.csv    LIO-SAM 的 /lio_sam/mapping/odometry + 真值

⚠️ 负载警告：Jetson Orin Nano（8 GB）上 Gazebo + 两条算法同时跑会吃满 CPU，
   实时因子会掉。**报告里的数字请用 run_bench.sh（回放同一个 bag、一次只跑一
   条算法）**，那个才是受控条件；这个 launch 的意义是"肉眼对比 + 录演示视频"。

⚠️ 两套 TF 的共存方式：
     LIO-SAM 发 odom->base_link / odom->lidar_link，并自带 map->odom 静态 TF
     A-LOAM  发 camera_init->aft_mapped
   这里再加一条 map->camera_init，于是两棵 TF 树挂在同一个 map 下，
   两个算法都从各自的起始位姿（= 同一个物理位姿）开始，轨迹可以直接叠。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bench_pkg = get_package_share_directory('lio_sam_bench')
    bringup_pkg = get_package_share_directory('lio_sam_gz_bringup')

    drive = LaunchConfiguration('drive')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    out_dir = LaunchConfiguration('out_dir')

    gt_params = os.path.join(bench_pkg, 'config', 'bench.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'gazebo_lab1.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'drive': drive,
            'use_sim_time': use_sim_time,
            'world': world,
        }.items(),
    )

    loam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'loam_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    liosam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'lio_sam_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # A-LOAM 的 world 系叫 camera_init，挂到 map 下面，两棵树就能叠在一起
    static_tf_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'camera_init',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    gt_sync = Node(
        package='lio_sam_bench',
        executable='gt_sync.py',
        name='gt_sync',
        parameters=[gt_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    common = {
        'gt_topic': '/gt/pose_sync',
        'cloud_topic': '/points',
        'use_sim_time': use_sim_time,
    }

    rec_loam = Node(
        package='lio_sam_bench',
        executable='traj_recorder.py',
        name='traj_recorder_loam',
        parameters=[dict(common, algo='loam',
                         est_topic='/aft_mapped_to_init',
                         out_csv=PathJoinSubstitution([out_dir, 'loam.csv']))],
        output='screen',
    )

    rec_liosam = Node(
        package='lio_sam_bench',
        executable='traj_recorder.py',
        name='traj_recorder_liosam',
        parameters=[dict(common, algo='liosam',
                         est_topic='/lio_sam/mapping/odometry',
                         out_csv=PathJoinSubstitution([out_dir, 'liosam.csv']))],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bench_pkg, 'rviz', 'slam_compare.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='auto',
                              description='auto / teleop / none'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_pkg, 'worlds', 'lab1_room.sdf')),
        DeclareLaunchArgument(
            'out_dir', default_value='results/raw/compare',
            description='两份 CSV 的输出目录'),

        gazebo,
        static_tf_camera_init,
        loam,
        liosam,
        gt_sync,
        rec_loam,
        rec_liosam,
        rviz_node,
    ])
