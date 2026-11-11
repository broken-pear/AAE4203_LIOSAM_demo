#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_lab1.launch.py —— Demo 1：Gazebo 仿真 + RViz2 看传感器（不含 LIO-SAM）

一条命令起全套：
    ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py

它会启动：
    1. Gazebo Harmonic，加载 worlds/lab1_room.sdf
    2. ros_gz_sim create，把 models/lio_bot/model.sdf 里的"雷达+IMU 小车"放进场景
    3. ros_gz_bridge，按 config/gz_bridge.yaml 桥接 /clock /points_raw /imu_raw /cmd_vel
    4. sensor_prep.py，把点云打包成 LIO-SAM 需要的 x,y,z,intensity,ring,time
    5. patrol_driver.py，让小车自动绕圈
    6. RViz2，显示 /points、TF

常用参数：
    rviz:=false        不开 RViz（无显示器 / SSH 时用）
    drive:=teleop      不自动绕圈，改为手动按键控制（配合 teleop_twist_keyboard）
    drive:=none        不控制，车不动
    world:=<路径>      换场景
    use_sim_time:=true 默认 true，别乱改
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch.logging as launch_logging
from launch_ros.actions import Node

# Jazzy 起 LaunchConfigurationEquals 被标记为 deprecated，每次启动都会刷一段
# 警告（警告里还建议改用 EqualsSubstitution）。这里优先用新 API，老发行版退回旧 API，
# 免得每次看日志都要先跳过一大段黄字。
try:
    from launch.substitutions import EqualsSubstitution

    def drive_is(value):
        return IfCondition(EqualsSubstitution(LaunchConfiguration('drive'), value))
except ImportError:                                            # pragma: no cover
    from launch.conditions import LaunchConfigurationEquals

    def drive_is(value):
        return LaunchConfigurationEquals('drive', value)


def generate_launch_description():
    pkg = get_package_share_directory('lio_sam_gz_bringup')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    drive = LaunchConfiguration('drive')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    world_file = os.path.join(pkg, 'worlds', 'lab1_room.sdf')
    model_file = os.path.join(pkg, 'models', 'lio_bot', 'model.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'lab1_sensors.rviz')

    # ---------------------------------------------------------------- Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        # -r = 启动后立刻开始仿真（不加就一直暂停）
        launch_arguments={'gz_args': ['-r -v 3 ', world]}.items(),
    )

    # 把小车 spawn 进场景。ros_gz_sim create 会一直等 /world/<name>/create 服务出现，
    # 所以不用操心"Gazebo 还没起来"的时序问题。
    #
    # ⚠ 这里的世界名**不能写死**。踩过的坑：原来写的是 'lab1_room'，换成
    # corridor.sdf / hall_sparse.sdf 之后，create 一直在等
    # /world/lab1_room/create —— 而那个世界根本不存在，于是小车永远没被生成，
    # 四个话题全部 0 条消息，录出来一个空包（s3/s4 各浪费两轮，共 4 分钟）。
    # 现在从世界文件的 <world name="..."> 里读，换场景自动跟随。
    def _world_name(path):
        try:
            import xml.etree.ElementTree as ET
            root = ET.parse(path).getroot()
            w = root.find('world')
            if w is not None and w.get('name'):
                return w.get('name')
        except Exception as exc:                      # noqa: BLE001
            launch_logging.get_logger("gazebo_lab1").warn('读不出世界名（%s），退回 lab1_room: %s' % (path, exc))
        return 'lab1_room'

    def _make_spawn(context, *args, **kwargs):
        wpath = LaunchConfiguration('world').perform(context)
        name = _world_name(wpath)
        launch_logging.get_logger("gazebo_lab1").info('spawn 目标世界: %s（来自 %s）' % (name, wpath))
        return [Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', name,
                '-name', LaunchConfiguration('robot_name').perform(context),
                '-file', model_file,
                '-x', '2.5', '-y', '0.0', '-z', '0.02',
            ],
            output='screen',
        )]

    spawn_robot = OpaqueFunction(function=_make_spawn)

    # ------------------------------------------------------------ ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg}],
        output='screen',
    )

    # -------------------------------------------------- 点云/IMU 格式规范化
    sensor_prep = Node(
        package='lio_sam_gz_bringup',
        executable='sensor_prep.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ------------------------------------------------------------ 自动绕圈
    patrol = Node(
        package='lio_sam_gz_bringup',
        executable='patrol_driver.py',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=drive_is('auto'),
        output='screen',
    )

    # ------------------------------------------------------------ RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_file,
                              description='Gazebo world (sdf) 路径'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('drive', default_value='auto',
                              description='auto = 自动绕圈 / teleop = 手动 / none = 不动'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='用 Gazebo 的仿真时间'),
        DeclareLaunchArgument('robot_name', default_value='lio_bot',
                              description='Gazebo 里的模型名'),

        gz_sim,
        spawn_robot,
        bridge,
        sensor_prep,
        patrol,
        rviz_node,
    ])
