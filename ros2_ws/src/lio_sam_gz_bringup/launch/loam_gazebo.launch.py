#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loam_gazebo.launch.py —— Demo 3：同一个 Gazebo 场景，把雷达数据喂给 **LOAM**
================================================================================
这是对照实验的 baseline 组。除了算法本身，其它一切都和 Demo 2 的 LIO-SAM
**逐项对齐**（同一个 world、同一个 16 线雷达参数、同一台车、同一条
patrol_driver 轨迹、同一个 spawn 位姿），这样两组结果的差异只能来自算法。

一条命令：
    ros2 launch lio_sam_gz_bringup loam_gazebo.launch.py

它启动什么：
    * gazebo_lab1.launch.py 的全部（Gazebo + 车 + 桥接 + sensor_prep + 绕圈驱动）
      但不重复开 RViz
    * A-LOAM 的 3 个节点（见 loam_nodes.launch.py）
    * 1 个静态 TF  map -> camera_init
    * RViz2（rviz/loam_slam.rviz）

--------------------------------------------------------------------------------
★ 关键：为什么 A-LOAM 能直接吃我们的 /points
--------------------------------------------------------------------------------
A-LOAM 订阅 /velodyne_points（sensor_msgs/PointCloud2），内部用
pcl::PointCloud<pcl::PointXYZ> 读取，只认 x/y/z 三个字段，多出来的
intensity/ring/time 会被忽略。线号它自己按垂直角反算：

    N_SCANS == 16 时   scanID = int((angle_deg + 15) / 2 + 0.5)

我们 model.sdf 里正是 16 线、垂直 ±15°、每线 2°，所以算出来的 scanID 正确。
于是只需要一个 remap：/velodyne_points <-- /points。

**没有做任何额外预处理**（不去畸变、不排序、不下采样），所以 LOAM 拿到的就是
带运动畸变的原始一帧 —— 这正是它相对 LIO-SAM 的短板，也是本实验要量的东西。

--------------------------------------------------------------------------------
★ 参数对齐表（LOAM 侧 vs LIO-SAM 侧，报告里要贴这张）
--------------------------------------------------------------------------------
  项目                       LOAM               LIO-SAM（params_gazebo.yaml）
  ------------------------   -----------------  ---------------------------
  线数                        16                  N_SCAN=16
  最近距离                    0.3 m               lidarMinRange=0.3
  建图角点体素                0.1 m               mappingCornerLeafSize=0.1
  建图面点体素                0.2 m               mappingSurfLeafSize=0.2
  建图频率                    10 Hz (skip=1)      mappingProcessInterval=0.15
  IMU                         ✗ 不用              ✓ 200 Hz 紧耦合
  点云去畸变                  ✗ 无                ✓ IMU 插值 deskew
  回环                        ✗ 无                ✗ 关闭（两边一致）
  后端                        帧到地图迭代        iSAM2 因子图

用法示例：
    # 不开 RViz（跑实验时用这个，省 CPU）
    ros2 launch lio_sam_gz_bringup loam_gazebo.launch.py start_rviz:=false
    # 车不动，只想看传感器
    ros2 launch lio_sam_gz_bringup loam_gazebo.launch.py drive:=none
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lio_sam_gz_bringup')

    drive = LaunchConfiguration('drive')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    start_rviz = LaunchConfiguration('start_rviz')

    loam_rviz = os.path.join(pkg, 'rviz', 'loam_slam.rviz')

    # ------------------------------------------- Demo 1 的全部内容（不含 RViz）
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

    # A-LOAM 三个节点（定义在 loam_nodes.launch.py，和离线回放共用一份）
    loam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'loam_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_line': LaunchConfiguration('scan_line'),
            'minimum_range': LaunchConfiguration('minimum_range'),
            'mapping_skip_frame': LaunchConfiguration('mapping_skip_frame'),
            'mapping_line_resolution': LaunchConfiguration('mapping_line_resolution'),
            'mapping_plane_resolution': LaunchConfiguration('mapping_plane_resolution'),
        }.items(),
    )

    # A-LOAM 只发 camera_init -> aft_mapped 的 TF。
    # RViz 的 Fixed Frame 用 map，所以补一个 map -> camera_init
    #（等价于把第一帧雷达位姿当作 map 原点），和 LIO-SAM 的 RViz 配置保持一致。
    static_tf = Node(
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', loam_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true',
                              description='是否启动 LOAM 版 RViz2'),
        DeclareLaunchArgument('drive', default_value='auto',
                              description='auto / teleop / none'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg, 'worlds', 'lab1_room.sdf')),
        # 下面 5 个参数要和 LIO-SAM 的 params_gazebo.yaml 对齐（见文件头表格）
        DeclareLaunchArgument('scan_line', default_value='16'),
        DeclareLaunchArgument('minimum_range', default_value='0.3'),
        DeclareLaunchArgument('mapping_skip_frame', default_value='1'),
        DeclareLaunchArgument('mapping_line_resolution', default_value='0.1'),
        DeclareLaunchArgument('mapping_plane_resolution', default_value='0.2'),

        gazebo,
        static_tf,
        loam,
        rviz_node,
    ])
