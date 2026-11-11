"""本地静态校验：YAML / SDF / Python 语法 / launch 文件结构。"""
import ast
import os
import sys
import xml.etree.ElementTree as ET

import yaml

# 仓库根目录 = 本文件所在目录的上一级（不依赖任何绝对路径）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "ros2_ws", "src", "lio_sam_gz_bringup")

fail = 0


def ok(msg):
    print("  [OK]   " + msg)


def bad(msg):
    global fail
    fail += 1
    print("  [FAIL] " + msg)


print("=== YAML ===")
for rel in ("config/gz_bridge.yaml", "config/params_gazebo.yaml",
            "rviz/lab1_sensors.rviz", "rviz/lio_sam_slam.rviz"):
    p = os.path.join(PKG, rel)
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        ok(f"{rel}: 解析成功 ({type(data).__name__})")
    except Exception as e:  # noqa: BLE001
        bad(f"{rel}: {e}")

print("\n=== params_gazebo.yaml 关键项 ===")
with open(os.path.join(PKG, "config", "params_gazebo.yaml"), "r", encoding="utf-8") as f:
    params = yaml.safe_load(f)
rp = params["/**"]["ros__parameters"]
expect = {
    "pointCloudTopic": "/points",
    "imuTopic": "/imu/data",
    "sensor": "velodyne",
    "N_SCAN": 16,
    "Horizon_SCAN": 360,
    "lidarFrame": "lidar_link",
    "baselinkFrame": "base_link",
}
for k, v in expect.items():
    if rp.get(k) == v:
        ok(f"{k} = {v!r}")
    else:
        bad(f"{k} 期望 {v!r}，实际 {rp.get(k)!r}")
for k in ("extrinsicRot", "extrinsicRPY"):
    if rp.get(k) == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]:
        ok(f"{k} 是 3x3 单位阵（9 个数）")
    else:
        bad(f"{k} 不是单位阵: {rp.get(k)}")
if rp.get("extrinsicTrans") == [0.0, 0.0, 0.0]:
    ok("extrinsicTrans = [0,0,0]")
else:
    bad("extrinsicTrans 不是零向量")

print("\n=== gz_bridge.yaml 结构 ===")
with open(os.path.join(PKG, "config", "gz_bridge.yaml"), "r", encoding="utf-8") as f:
    bridges = yaml.safe_load(f)
allowed = {"GZ_TO_ROS", "ROS_TO_GZ", "BIDIRECTIONAL"}
for b in bridges:
    miss = {"ros_topic_name", "gz_topic_name", "ros_type_name",
            "gz_type_name", "direction"} - set(b)
    if miss:
        bad(f"{b.get('ros_topic_name')}: 缺字段 {miss}")
    elif b["direction"] not in allowed:
        bad(f"{b.get('ros_topic_name')}: direction={b['direction']}")
    else:
        ok(f"{b['ros_topic_name']:14s} <-> {b['gz_topic_name']:12s} [{b['direction']}]"
           f"  {b['ros_type_name']} <-> {b['gz_type_name']}")

print("\n=== SDF / XML ===")
for rel in ("worlds/lab1_room.sdf", "models/lio_bot/model.sdf",
            "package.xml", "CMakeLists.txt"):
    p = os.path.join(PKG, rel)
    if rel.endswith("CMakeLists.txt"):
        continue
    try:
        tree = ET.parse(p)
        ok(f"{rel}: XML 合法, 根标签 <{tree.getroot().tag}>")
    except Exception as e:  # noqa: BLE001
        bad(f"{rel}: {e}")

print("\n=== world 里的关键系统插件 / 重力 ===")
w = open(os.path.join(PKG, "worlds/lab1_room.sdf"), encoding="utf-8").read()
for needle in ("gz-sim-physics-system", "gz-sim-sensors-system",
               "gz-sim-scene-broadcaster-system", "ogre2",
               "<gravity>0 0 -9.80511</gravity>", 'name="lab1_room"'):
    ok(f"world 含 {needle}") if needle in w else bad(f"world 缺 {needle}")

print("\n=== model 里的关键传感器 / 插件 ===")
m = open(os.path.join(PKG, "models/lio_bot/model.sdf"), encoding="utf-8").read()
for needle in ('type="gpu_lidar"', 'type="imu"', "<samples>360</samples>",
               "<samples>16</samples>", "gz-sim-diff-drive-system",
               "<topic>/points_raw</topic>", "<topic>/imu_raw</topic>",
               "<topic>/cmd_vel</topic>", "publish_odom_tf"):
    ok(f"model 含 {needle}") if needle in m else bad(f"model 缺 {needle}")

print("\n=== Python 语法 ===")
for rel in ("scripts/sensor_prep.py", "scripts/patrol_driver.py",
            "launch/gazebo_lab1.launch.py", "launch/lio_sam_gazebo.launch.py"):
    p = os.path.join(PKG, rel)
    try:
        src = open(p, encoding="utf-8").read()
        ast.parse(src)
        ok(f"{rel}: 语法 OK ({len(src.splitlines())} 行)")
    except SyntaxError as e:
        bad(f"{rel}: {e}")

print("\n=== launch 文件里的关键字符串 ===")
g = open(os.path.join(PKG, "launch/gazebo_lab1.launch.py"), encoding="utf-8").read()
for needle in ("gz_sim.launch.py", "gz_args", "'create'", "config_file",
               "sensor_prep.py", "patrol_driver.py", "lab1_sensors.rviz"):
    ok(f"gazebo_lab1 含 {needle}") if needle in g else bad(f"gazebo_lab1 缺 {needle}")

s = open(os.path.join(PKG, "launch/lio_sam_gazebo.launch.py"), encoding="utf-8").read()
for needle in ("lio_sam_imuPreintegration", "lio_sam_imageProjection",
               "lio_sam_featureExtraction", "lio_sam_mapOptimization",
               "--frame-id", "map", "odom", "lidar_link", "imu_link",
               "lio_sam_slam.rviz"):
    ok(f"lio_sam_gazebo 含 {needle}") if needle in s else bad(f"lio_sam_gazebo 缺 {needle}")

print("\n=== CMakeLists / package.xml 一致性 ===")
cml = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
for d in ("launch", "config", "rviz", "worlds", "models"):
    if os.path.isdir(os.path.join(PKG, d)) and d in cml:
        ok(f"install(DIRECTORY) 覆盖 {d}/")
    else:
        bad(f"{d}/ 存在但没有被 install")
for s_ in ("sensor_prep.py", "patrol_driver.py"):
    if s_ in cml and os.path.isfile(os.path.join(PKG, "scripts", s_)):
        ok(f"install(PROGRAMS) 覆盖 scripts/{s_}")
    else:
        bad(f"scripts/{s_} 没有被 install")

print("\n=== 结论 ===")
print("ALL PASS" if fail == 0 else f"{fail} FAILED")
sys.exit(1 if fail else 0)
