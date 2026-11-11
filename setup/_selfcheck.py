"""本地静态校验：YAML / SDF / Python 语法 / launch 文件结构 / 两套算法的一致性。

在没有 ROS 的机器上（比如写报告用的 Windows）也能跑，用来保证"提交的东西没坏"：

    python3 setup/_selfcheck.py          # 全部检查，退出码 0 = 通过
    python3 setup/_selfcheck.py -q       # 只打印失败项

它**不能**证明算法能跑（那要 colcon build + Gazebo），但能抓住 90% 的低级错误：
文件没被 install、参数名写错、yaml 数组长度不一致、launch 里 remap 丢了、
两条管线的量程/体素对不上……
"""
import ast
import glob
import re
import shutil
import subprocess
import os
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:                                           # pragma: no cover
    print('这个检查需要 PyYAML：')
    print('    sudo apt install python3-yaml      # 机器人上（install_deps.sh 会装）')
    print('    pip install pyyaml                # 其它机器上')
    sys.exit(2)

QUIET = '-q' in sys.argv

# 仓库根目录 = 本文件所在目录的上一级（不依赖任何绝对路径）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "ros2_ws", "src", "lio_sam_gz_bringup")
BENCH = os.path.join(ROOT, "ros2_ws", "src", "lio_sam_bench")
SCEN = os.path.join(ROOT, "experiments", "scenarios")

fail = 0
passed = 0


def ok(msg):
    global passed
    passed += 1
    if not QUIET:
        print("  [OK]   " + msg)


def bad(msg):
    global fail
    fail += 1
    print("  [FAIL] " + msg)


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def parse_yaml(path, label=None):
    try:
        with open(path, encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception as e:  # noqa: BLE001
        bad(f"{label or path}: {e}")
        return None


# =============================================================== YAML
print("=== YAML ===")
for rel in ("config/gz_bridge.yaml", "config/params_gazebo.yaml",
            "rviz/lab1_sensors.rviz", "rviz/lio_sam_slam.rviz",
            "rviz/loam_slam.rviz"):
    p = os.path.join(PKG, rel)
    data = parse_yaml(p, rel)
    if data is not None:
        ok(f"{rel}: 解析成功 ({type(data).__name__})")

for rel in ("config/bench.yaml", "rviz/slam_compare.rviz"):
    p = os.path.join(BENCH, rel)
    data = parse_yaml(p, rel)
    if data is not None:
        ok(f"lio_sam_bench/{rel}: 解析成功")

# ==================================================== LIO-SAM 参数关键项
print("\n=== params_gazebo.yaml 关键项 ===")
params = parse_yaml(os.path.join(PKG, "config", "params_gazebo.yaml")) or {}
rp = (params.get("/**", {}) or {}).get("ros__parameters", {}) or {}
expect = {
    "pointCloudTopic": "/points",
    "imuTopic": "/imu/data",
    "sensor": "velodyne",
    "N_SCAN": 16,
    "Horizon_SCAN": 360,
    "lidarFrame": "lidar_link",
    "baselinkFrame": "base_link",
    "lidarMinRange": 0.3,
    "mappingCornerLeafSize": 0.1,
    "mappingSurfLeafSize": 0.2,
    "loopClosureEnableFlag": False,
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

# ==================================== 两条管线的量程/体素必须对得上
print("\n=== 公平性：LOAM 参数 vs LIO-SAM 参数 ===")
loam_launch = read(os.path.join(PKG, "launch", "loam_nodes.launch.py"))
pairs = [
    # (LOAM launch 里的默认值, LIO-SAM 参数名, 期望值)
    ("'minimum_range', default_value='0.3'", "lidarMinRange", 0.3),
    ("'mapping_line_resolution', default_value='0.1'", "mappingCornerLeafSize", 0.1),
    ("'mapping_plane_resolution', default_value='0.2'", "mappingSurfLeafSize", 0.2),
    ("'scan_line', default_value='16'", "N_SCAN", 16),
]
for needle, pname, val in pairs:
    if needle in loam_launch and rp.get(pname) == val:
        ok(f"两侧一致: {pname} = {val}")
    else:
        bad(f"两侧不一致: LOAM launch 里 {needle!r} vs LIO-SAM {pname}={rp.get(pname)!r}")
if "'/velodyne_points', '/points'" in loam_launch:
    ok("A-LOAM 输入 remap: /velodyne_points <- /points")
else:
    bad("A-LOAM 缺 remap /velodyne_points <- /points（它会收不到点云）")

# ========================================================== gz_bridge
print("\n=== gz_bridge.yaml 结构 ===")
bridges = parse_yaml(os.path.join(PKG, "config", "gz_bridge.yaml")) or []
allowed = {"GZ_TO_ROS", "ROS_TO_GZ", "BIDIRECTIONAL"}
topics = set()
for b in bridges:
    miss = {"ros_topic_name", "gz_topic_name", "ros_type_name",
            "gz_type_name", "direction"} - set(b)
    if miss:
        bad(f"{b.get('ros_topic_name')}: 缺字段 {miss}")
    elif b["direction"] not in allowed:
        bad(f"{b.get('ros_topic_name')}: direction={b['direction']}")
    else:
        topics.add(b["ros_topic_name"])
        ok(f"{b['ros_topic_name']:14s} <-> {b['gz_topic_name']:12s} [{b['direction']}]")
for need in ("/points_raw", "/imu_raw", "/clock", "/cmd_vel", "/gt/odom"):
    if need in topics:
        ok(f"桥接了 {need}")
    else:
        bad(f"没有桥接 {need}")

# ============================================================ SDF / XML
print("\n=== SDF / XML ===")
sdf_files = [
    os.path.join(PKG, "worlds", "lab1_room.sdf"),
    os.path.join(PKG, "worlds", "corridor.sdf"),
    os.path.join(PKG, "worlds", "hall_sparse.sdf"),
    os.path.join(PKG, "models", "lio_bot", "model.sdf"),
    os.path.join(PKG, "package.xml"),
    os.path.join(BENCH, "package.xml"),
]
for p in sdf_files:
    try:
        tree = ET.parse(p)
        ok(f"{os.path.relpath(p, ROOT)}: XML 合法, 根标签 <{tree.getroot().tag}>")
    except Exception as e:  # noqa: BLE001
        bad(f"{os.path.relpath(p, ROOT)}: {e}")

print("\n=== world 里的系统插件 / 重力 ===")
for wname in ("lab1_room.sdf", "corridor.sdf", "hall_sparse.sdf"):
    w = read(os.path.join(PKG, "worlds", wname))
    missing = [n for n in ("gz-sim-physics-system", "gz-sim-sensors-system",
                           "gz-sim-imu-system", "gz-sim-scene-broadcaster-system",
                           "<gravity>0 0 -9.80511</gravity>")
               if n not in w]
    if missing:
        bad(f"worlds/{wname} 缺 {missing}")
    else:
        ok(f"worlds/{wname}: 插件与重力齐全")

# ======================================================== model.sdf
print("\n=== model: 传感器 / 驱动 / 真值插件 ===")
m = read(os.path.join(PKG, "models", "lio_bot", "model.sdf"))
for needle in ('type="gpu_lidar"', 'type="imu"', "<samples>360</samples>",
               "<samples>16</samples>", "gz-sim-diff-drive-system",
               "<topic>/points_raw</topic>", "<topic>/imu_raw</topic>",
               "<topic>/cmd_vel</topic>", "publish_odom_tf",
               "gz-sim-odometry-publisher-system", "<odom_topic>/gt/odom</odom_topic>",
               "<odom_frame>world</odom_frame>", "<dimensions>3</dimensions>"):
    if needle in m:
        ok(f"model 含 {needle}")
    else:
        bad(f"model 缺 {needle}")

# ======================================================= 场景 yaml
print("\n=== experiments/scenarios/*.yaml ===")
scen_files = sorted(glob.glob(os.path.join(SCEN, "*.yaml")))
if not scen_files:
    bad("一个场景文件都没有")
for p in scen_files:
    name = os.path.basename(p)
    d = parse_yaml(p, name) or {}
    # 这个文件同时是 ROS 2 参数文件 -> 每个顶层节点名下面都必须有 ros__parameters，
    # 否则参数解析器可能判定整份文件非法，录制直接起不来。
    bad_sections = [k for k, v in d.items()
                    if not (isinstance(v, dict) and isinstance(v.get("ros__parameters"), dict))]
    if bad_sections:
        bad(f"{name}: 这些顶层节缺 ros__parameters（ROS 参数文件会解析失败）: {bad_sections}")
        continue
    drv = d.get("scripted_driver", {}).get("ros__parameters", {}) or {}
    sc = d.get("scenario", {}).get("ros__parameters", {}) or {}
    v = drv.get("linear_speed") or []
    w = drv.get("angular_speed") or []
    t = drv.get("duration") or []
    if not (len(v) == len(w) == len(t) and len(v) > 0):
        bad(f"{name}: linear/angular/duration 长度不一致 "
            f"({len(v)}/{len(w)}/{len(t)})")
        continue
    if not sc.get("record_seconds"):
        bad(f"{name}: 缺 scenario.record_seconds")
        continue
    if sum(float(x) for x in t) > float(sc["record_seconds"]):
        bad(f"{name}: 动作总时长 {sum(float(x) for x in t):.1f}s "
            f"超过录制时长 {sc['record_seconds']}s（截断了）")
        continue
    world = sc.get("world", "")
    if not os.path.isfile(os.path.join(PKG, "worlds", world)):
        bad(f"{name}: 引用的 world 不存在: {world}")
        continue
    ok(f"{name}: {len(t)} 段 {sum(float(x) for x in t):.0f}s, world={world}, "
       f"record={sc['record_seconds']}s, 两个 ros__parameters 节齐全")

# ======================================================== Python 语法
print("\n=== Python 语法 ===")
py_files = [
    os.path.join(PKG, "scripts", "sensor_prep.py"),
    os.path.join(PKG, "scripts", "patrol_driver.py"),
    os.path.join(PKG, "scripts", "scripted_driver.py"),
    os.path.join(PKG, "launch", "gazebo_lab1.launch.py"),
    os.path.join(PKG, "launch", "lio_sam_gazebo.launch.py"),
    os.path.join(PKG, "launch", "loam_gazebo.launch.py"),
    os.path.join(PKG, "launch", "loam_nodes.launch.py"),
    os.path.join(PKG, "launch", "lio_sam_nodes.launch.py"),
    os.path.join(BENCH, "scripts", "gt_sync.py"),
    os.path.join(BENCH, "scripts", "traj_recorder.py"),
    os.path.join(BENCH, "scripts", "evaluate.py"),
    os.path.join(BENCH, "scripts", "make_figures.py"),
    os.path.join(BENCH, "launch", "bench_replay.launch.py"),
    os.path.join(BENCH, "launch", "compare_gazebo.launch.py"),
    os.path.join(BENCH, "launch", "record_bag.launch.py"),
    os.path.join(ROOT, "setup", "_selfcheck.py"),
    os.path.join(ROOT, "setup", "_test_metrics.py"),
    os.path.join(ROOT, "setup", "_test_pipeline.py"),
    os.path.join(ROOT, "setup", "_test_nodes.py"),
    os.path.join(ROOT, "setup", "_test_sanity.py"),
    os.path.join(ROOT, "setup", "_test_run_bench.py"),
    os.path.join(ROOT, "experiments", "check_bag.py"),
    os.path.join(ROOT, "experiments", "sanity_check.py"),
    os.path.join(ROOT, "experiments", "check_scenarios.py"),
]
for p in py_files:
    try:
        src = read(p)
        ast.parse(src)
        ok(f"{os.path.relpath(p, ROOT)}: 语法 OK ({len(src.splitlines())} 行)")
    except SyntaxError as e:
        bad(f"{os.path.relpath(p, ROOT)}: {e}")
    except FileNotFoundError:
        bad(f"缺文件: {os.path.relpath(p, ROOT)}")

# =================================================== launch 关键字符串
print("\n=== launch 文件里的关键字符串 ===")
launch_needles = {
    "gazebo_lab1.launch.py": ("gz_sim.launch.py", "gz_args", "'create'",
                              "config_file", "sensor_prep.py",
                              "patrol_driver.py", "lab1_sensors.rviz"),
    "lio_sam_gazebo.launch.py": ("lio_sam_imuPreintegration",
                                 "lio_sam_imageProjection",
                                 "lio_sam_featureExtraction",
                                 "lio_sam_mapOptimization", "start_rviz",
                                 "lio_sam_slam.rviz"),
    "loam_gazebo.launch.py": ("gazebo_lab1.launch.py", "loam_nodes.launch.py",
                              "camera_init", "loam_slam.rviz"),
    "loam_nodes.launch.py": ("aloam_velodyne_ros2", "ascanRegistration",
                             "alaserOdometry", "alaserMapping",
                             "/velodyne_points"),
    "lio_sam_nodes.launch.py": ("lio_sam_imuPreintegration", "map", "odom",
                                "base_link", "lidar_link", "imu_link"),
}
for fname, needles in launch_needles.items():
    src = read(os.path.join(PKG, "launch", fname))
    missing = [n for n in needles if n not in src]
    if missing:
        bad(f"{fname} 缺 {missing}")
    else:
        ok(f"{fname}: 关键字符串齐全")

bench_needles = {
    "bench_replay.launch.py": ("bag", "algo", "out_csv", "--clock", "gt_sync.py",
                               "traj_recorder.py", "/aft_mapped_to_init",
                               "/lio_sam/mapping/odometry", "Shutdown",
                               "--start-offset", "start_offset"),
    "compare_gazebo.launch.py": ("gazebo_lab1.launch.py", "loam_nodes.launch.py",
                                 "lio_sam_nodes.launch.py", "camera_init",
                                 "slam_compare.rviz"),
    "record_bag.launch.py": ("ros2", "bag", "record", "/points", "/imu/data",
                             "/gt/odom", "scripted_driver.py",
                             "--storage", "sqlite3", "topic_watchdog"),
}
for fname, needles in bench_needles.items():
    src = read(os.path.join(BENCH, "launch", fname))
    missing = [n for n in needles if n not in src]
    if missing:
        bad(f"lio_sam_bench/{fname} 缺 {missing}")
    else:
        ok(f"lio_sam_bench/{fname}: 关键字符串齐全")
# ============== launch 里的 Node(package, executable) 与被 include 的 launch 目标
# 名字写错（traj_recoder.py / loam_node.launch.py）不会在静态字符串检查里暴露，
# 但 ros2 launch 一启动就报 "executable not found"。这里把两份清单对起来。
print("\n=== launch 的 Node(package, executable) 与 include 目标 ===")


def installed_programs(cml_path):
    src = read(cml_path)
    out = set()
    for m in re.finditer(r"install\(PROGRAMS(.*?)\n\)", src, re.S):
        out |= set(re.findall(r"scripts/([\w]+\.py)", m.group(1)))
    return out


OWN_PROGS = {
    "lio_sam_gz_bringup": installed_programs(os.path.join(PKG, "CMakeLists.txt")),
    "lio_sam_bench": installed_programs(os.path.join(BENCH, "CMakeLists.txt")),
}
for k, v in OWN_PROGS.items():
    if v:
        ok(f"{k} 安装的脚本: {sorted(v)}")
    else:
        bad(f"{k}: 解析不到 install(PROGRAMS) 的脚本清单")
for k, v in OWN_PROGS.items():
    if not os.path.isdir(os.path.join(PKG if k == "lio_sam_gz_bringup" else BENCH, "scripts")):
        continue
    for s in sorted(v):
        if not os.path.isfile(os.path.join(PKG if k == "lio_sam_gz_bringup" else BENCH,
                                           "scripts", s)):
            bad(f"{k}: CMakeLists 里声明了 scripts/{s}，但文件不存在")

# 外部包提供的 launch 目标（我们无权校验它是否装了）
EXTERNAL_INCLUDES = {"gz_sim.launch.py"}

launch_dirs = {
    "lio_sam_gz_bringup": os.path.join(PKG, "launch"),
    "lio_sam_bench": os.path.join(BENCH, "launch"),
}
external_execs = {}
included_names = set()
for pkg_name, ldir in launch_dirs.items():
    for path in sorted(glob.glob(os.path.join(ldir, "*.launch.py"))):
        base = os.path.basename(path)
        src = read(path)
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call) and getattr(node.func, "id", "") == "Node"):
                continue
            kw = {k.arg: k.value for k in node.keywords}
            pv = getattr(kw.get("package"), "value", None)
            ev = getattr(kw.get("executable"), "value", None)
            if not pv or not ev:
                continue
            if pv in OWN_PROGS:
                if ev in OWN_PROGS[pv]:
                    ok(f"{base}: {pv}/{ev}")
                else:
                    bad(f"{base}: 引用了 {pv}/{ev}，但该包没有安装这个脚本")
            else:
                external_execs.setdefault(pv, set()).add(ev)
        for nm in re.findall(r"'([\w.]+\.launch\.py)'", src):
            included_names.add(nm)

for nm in sorted(included_names):
    if nm in EXTERNAL_INCLUDES:
        ok(f"include 目标来自外部包: {nm}（跳过校验）")
    elif any(os.path.isfile(os.path.join(d, nm)) for d in launch_dirs.values()):
        ok(f"include 目标存在: {nm}")
    else:
        bad(f"include 目标在本仓库里找不到: {nm}")

for pv, ex in sorted(external_execs.items()):
    ok(f"外部包 {pv} 的 executable: {sorted(ex)}（需已安装对应包）")
# ======================= 估计位姿的订阅 QoS（踩过的坑，必须钉住）
# LIO-SAM 用 BEST_EFFORT 发 /lio_sam/mapping/odometry（上游 utility.hpp 的 qos_profile），
# RELIABLE 订阅者与它不兼容 —— 一条都收不到，表现却像"算法没出位姿"。
# A-LOAM 发 /aft_mapped_to_init 用 RELIABLE，BEST_EFFORT 订阅者两种都能收。
# 这条检查就是防止有人把 recorder 的订阅改回 RELIABLE。
print("\n=== 估计位姿订阅的 QoS ===")
_rec = read(os.path.join(BENCH, "scripts", "traj_recorder.py"))
if re.search(r"est_qos\s*=\s*QoSProfile\([\s\S]*?"
             r"reliability=ReliabilityPolicy\.BEST_EFFORT", _rec):
    ok("traj_recorder: est_qos 是 BEST_EFFORT")
else:
    bad("traj_recorder 的 est_qos 不是 BEST_EFFORT —— LIO-SAM 的 odometry 是 "
        "BEST_EFFORT 发的，RELIABLE 订阅者收不到任何消息（会表现为 0 帧）")
if re.search(r"create_subscription\(Odometry,\s*self\.est_topic,\s*self\.on_est,"
             r"\s*est_qos\)", _rec):
    ok("traj_recorder: est_topic 用的就是 est_qos")
else:
    bad("traj_recorder: est_topic 没有使用 est_qos（可能又被改成 RELIABLE 的 pub_qos）")
if "self.gt_topic, self.on_gt, est_qos" in _rec:
    ok("traj_recorder: gt_topic 也用 BEST_EFFORT（真值由我们自己的 gt_sync 发）")
else:
    bad("traj_recorder: gt_topic 没有用 est_qos")

# ======================= 文档里的相对链接是否都能打开
# README 有八百多行、指向 docs/experiments/report 里的多个文件。
# 链接写错不会报错，只会让读者点到一个 404 —— 对交付仓库是硬伤。
print("\n=== 文档内部链接 ===")
_MD_LINK = re.compile(r"\]\(([^)#\s]+)(?:#[^)]*)?\)")
_md_files = [os.path.join(ROOT, "README.md")]
for _d in ("docs", "experiments", "report", "setup"):
    _md_files += sorted(glob.glob(os.path.join(ROOT, _d, "*.md")))
_bad = 0
_checked = 0
for _md in _md_files:
    if not os.path.isfile(_md):
        continue
    _base = os.path.dirname(_md)
    _src = read(_md)
    for _target in _MD_LINK.findall(_src):
        if _target.startswith(("http://", "https://", "mailto:", "#")):
            continue
        _checked += 1
        _p = os.path.normpath(os.path.join(_base, _target))
        if not os.path.exists(_p):
            _bad += 1
            bad(f"{os.path.relpath(_md, ROOT)}: 链接指向不存在的 {_target}")
if _bad == 0:
    ok(f"{len(_md_files)} 份文档、{_checked} 个相对链接全部有效")
else:
    bad(f"{_bad} 个链接失效（共 {_checked} 个）")

# ======================= 静态 TF 不能跟仿真时钟走
# 踩过的坑：静态 TF 设 use_sim_time:=true 时要等 /clock 才发布，于是第一批点云
# 到达时 tf2 缓冲区里还没有 base_link->lidar_link，LIO-SAM 报
# "Tf has two or more unconnected trees"，外参拿不到 -> IMU 初始化失效 ->
# GTSAM 秩亏崩溃。静态变换与时间无关，必须用墙钟发布。
print("\n=== 静态 TF 的 use_sim_time ===")
for _f in ("lio_sam_nodes.launch.py", "lio_sam_gazebo.launch.py"):
    _p = os.path.join(PKG, "launch", _f)
    _s = read(_p)
    _m = re.search(r"def _static_tf\(.*?parameters=\[\{'use_sim_time':\s*(\w+)\}\]",
                   _s, re.S)
    if not _m:
        bad(f"{_f}: 找不到 _static_tf 的 use_sim_time 设置")
    elif _m.group(1) != "False":
        bad(f"{_f}: 静态 TF 的 use_sim_time={_m.group(1)} —— 必须为 False，"
            f"否则它会晚于第一批点云发布，LIO-SAM 查不到外参")
    else:
        ok(f"{_f}: 静态 TF 用墙钟（use_sim_time=False）")
_cg = read(os.path.join(BENCH, "launch", "compare_gazebo.launch.py"))
if "tf_map_to_camera_init" in _cg:
    if re.search(r"child-frame-id', 'camera_init',\s*\],\s*#?[^\n]*\n\s*parameters="
                 r"\[\{'use_sim_time': False\}\],", _cg):
        ok("compare_gazebo.launch.py: camera_init 静态 TF 用墙钟")
    else:
        bad("compare_gazebo.launch.py: camera_init 静态 TF 仍在跟仿真时钟")

# ======================= 雷达安装高度：三处必须一致
# 这个数字错了不会崩、不会报错，只会给每条 ATE 加一个常量偏差 ——
# 具体说：写成 0.40 而不是 0.28，两条算法的 ATE 会同时被抬高 12 cm，
# 差距被压平，报告里"提升多少"就失真了。所以让三个来源互相核对。
print("\n=== 雷达安装高度（model.sdf / 静态 TF / gt_sync）===")
import xml.etree.ElementTree as _ET

_sdf = os.path.join(PKG, "models", "lio_bot", "model.sdf")
_pose_z = {}
try:
    _root = _ET.parse(_sdf).getroot()
    for _link in _root.iter("link"):
        _name = _link.get("name")
        _p = _link.find("pose")
        if _name and _p is not None and _p.text:
            _vals = _p.text.split()
            if len(_vals) >= 3:
                _pose_z[_name] = float(_vals[2])
    _dz = _pose_z.get("lidar_link", 0.0) - _pose_z.get("base_link", 0.0)
    ok(f"model.sdf: lidar_link z={_pose_z.get('lidar_link')} , "
       f"base_link z={_pose_z.get('base_link')} -> 差 {_dz:.3f} m")
except Exception as _e:                                       # noqa: BLE001
    bad(f"解析 {_sdf} 失败: {_e}")
    _dz = None

if _dz is not None:
    # 1) gt_sync 的默认值
    _src = read(os.path.join(BENCH, "scripts", "gt_sync.py"))
    _m = re.search(r"declare_parameter\('lidar_z_offset',\s*([0-9.]+)\)", _src)
    if not _m:
        bad("gt_sync.py 里找不到 lidar_z_offset 的默认值")
    elif abs(float(_m.group(1)) - _dz) > 1e-6:
        bad(f"gt_sync 默认 lidar_z_offset={_m.group(1)} 与 model.sdf 的 {_dz:.3f} 不一致")
    else:
        ok(f"gt_sync 默认 lidar_z_offset={_m.group(1)} 与 model.sdf 一致")

    # 2) bench.yaml（运行时会覆盖默认值，所以它才是真正生效的那个）
    _y = os.path.join(BENCH, "config", "bench.yaml")
    _m = re.search(r"^\s*lidar_z_offset:\s*([0-9.]+)", read(_y), re.M)
    if not _m:
        bad(f"{_y} 里没有 lidar_z_offset")
    elif abs(float(_m.group(1)) - _dz) > 1e-6:
        bad(f"bench.yaml 的 lidar_z_offset={_m.group(1)} 与 model.sdf 的 {_dz:.3f} 不一致 —— "
            f"运行时会用这个值，ATE 会带上常量偏差")
    else:
        ok(f"bench.yaml 的 lidar_z_offset={_m.group(1)} 与 model.sdf 一致")

    # 3) 两个 launch 里的 base_link -> lidar_link 静态 TF
    for _f in ("lio_sam_gazebo.launch.py", "lio_sam_nodes.launch.py"):
        _p = os.path.join(PKG, "launch", _f)
        _m = re.search(r"_static_tf\('tf_base_to_lidar',\s*[0-9.-]+,\s*[0-9.-]+,\s*([0-9.-]+)",
                       read(_p))
        if not _m:
            bad(f"{_f}: 找不到 tf_base_to_lidar 的 z")
        elif abs(float(_m.group(1)) - _dz) > 1e-6:
            bad(f"{_f}: 静态 TF base->lidar z={_m.group(1)} 与 model.sdf 的 {_dz:.3f} 不一致")
        else:
            ok(f"{_f}: 静态 TF base->lidar z={_m.group(1)} 与 model.sdf 一致")

# ================== EST_TOPIC 是否真的是上游发布的话题（源码 clone 下来后才查）
# 录错话题的后果是"est 一条都没有"，本来也会在回放阶段报出来；
# 这里在 install 之后先拦一道，省一次 100 秒的回放。
print("\n=== EST_TOPIC 与上游源码是否一致 ===")
LIO_SRC = os.path.join(ROOT, "ros2_ws", "src", "LIO-SAM")
ALOAM_SRC = os.path.join(ROOT, "ros2_ws", "src", "A-LOAM-ROS2")
replay_src = read(os.path.join(BENCH, "launch", "bench_replay.launch.py"))
for label, src_file, needle, ours in (
        ("LIO-SAM", os.path.join(LIO_SRC, "src", "mapOptmization.cpp"),
         "lio_sam/mapping/odometry", "/lio_sam/mapping/odometry"),
        ("A-LOAM", os.path.join(ALOAM_SRC, "src", "laserMapping.cpp"),
         "/aft_mapped_to_init", "/aft_mapped_to_init"),
):
    if not os.path.isfile(src_file):
        ok(f"{label} 源码未 clone，跳过（install_deps.sh 跑完后再查）")
        continue
    if needle in read(src_file):
        ok(f"{label} 上游确实发布 {ours}")
    else:
        bad(f"{label} 上游源码里找不到 {needle} —— bench_replay 的 EST_TOPIC 可能录错话题")
    if ours not in replay_src:
        bad(f"bench_replay.launch.py 里没有 {ours}（EST_TOPIC 表被改坏了？）")
# ============================== 录制话题 vs 校验器（防止改一边忘另一边）
print("\n=== 录制话题与 check_bag 的一致性 ===")
rec_src = read(os.path.join(BENCH, "launch", "record_bag.launch.py"))
m = re.search(r"BAG_TOPICS\s*=\s*\[(.*?)\]", rec_src, re.S)
bag_topics = re.findall(r"'([^']+)'", m.group(1)) if m else []
cb_src = read(os.path.join(ROOT, "experiments", "check_bag.py"))
req_block = cb_src.split("REQUIRED = {", 1)[1].split("\n}", 1)[0] if "REQUIRED = {" in cb_src else ""
req_topics = re.findall(r"'(/[^']+)':", req_block)

# 录制阶段没有任何 TF 发布者：录它们只会得到空话题，还会让 check_bag 误判
FORBIDDEN_IN_BAG = ("/tf", "/tf_static", "/clock")
if not bag_topics:
    bad("record_bag.launch.py 里没找到 BAG_TOPICS")
else:
    bad_forbidden = [t for t in bag_topics if t in FORBIDDEN_IN_BAG]
    if bad_forbidden:
        bad(f"录制列表里有这些话题，但录制阶段没人发布它们: {bad_forbidden}")
    else:
        ok(f"录制列表不含 /tf /tf_static /clock: {bag_topics}")
    if set(bag_topics) != set(req_topics):
        bad(f"两份清单不一致：record_bag 录 {bag_topics}，check_bag 要求 {req_topics}")
    else:
        ok(f"check_bag 要求的 {req_topics} 与录制列表一致")
    # 录制阶段真正有发布者的只有这三路（sensor_prep x2 + 桥接 x1）
    if set(bag_topics) <= {"/points", "/imu/data", "/gt/odom"}:
        ok("录制的话题都有发布者（/points, /imu/data, /gt/odom）")
    else:
        bad(f"录制了不该录的话题: {set(bag_topics) - {'/points', '/imu/data', '/gt/odom'}}")

# ============================================== CMakeLists / package.xml
print("\n=== CMakeLists 安装项 ===")
cml = read(os.path.join(PKG, "CMakeLists.txt"))
for d in ("launch", "config", "rviz", "worlds", "models"):
    if os.path.isdir(os.path.join(PKG, d)) and d in cml:
        ok(f"lio_sam_gz_bringup install(DIRECTORY) 覆盖 {d}/")
    else:
        bad(f"{d}/ 存在但没有被 install")
for s_ in ("sensor_prep.py", "patrol_driver.py", "scripted_driver.py"):
    if s_ in cml and os.path.isfile(os.path.join(PKG, "scripts", s_)):
        ok(f"lio_sam_gz_bringup install(PROGRAMS) 覆盖 scripts/{s_}")
    else:
        bad(f"scripts/{s_} 没有被 install")

bcml = read(os.path.join(BENCH, "CMakeLists.txt"))
for d in ("launch", "config", "rviz"):
    if os.path.isdir(os.path.join(BENCH, d)) and d in bcml:
        ok(f"lio_sam_bench install(DIRECTORY) 覆盖 {d}/")
    else:
        bad(f"lio_sam_bench/{d}/ 存在但没有被 install（colcon build 会直接报错）")
for s_ in ("gt_sync.py", "traj_recorder.py", "evaluate.py", "make_figures.py"):
    if s_ in bcml and os.path.isfile(os.path.join(BENCH, "scripts", s_)):
        ok(f"lio_sam_bench install(PROGRAMS) 覆盖 scripts/{s_}")
    else:
        bad(f"lio_sam_bench/scripts/{s_} 没有被 install")

# ======================================================== 依赖与文档
print("\n=== 依赖清单 / 实验脚本 ===")
repos = parse_yaml(os.path.join(ROOT, "demo.repos")) or {}
r = repos.get("repositories", {}) or {}
for name, url_key in (("LIO-SAM", "TixiaoShan/LIO-SAM"),
                      ("A-LOAM-ROS2", "G-Apple1/A-LOAM-ROS2")):
    entry = r.get(name) or {}
    if url_key in (entry.get("url") or ""):
        ok(f"demo.repos 含 {name} ({entry.get('version')})")
    else:
        bad(f"demo.repos 缺 {name}")

install = read(os.path.join(ROOT, "setup", "install_deps.sh"))
for needle in ("A-LOAM-ROS2", "patch_aloam.sh", "libceres-dev",
               "libgoogle-glog-dev", "ALOAM_SHA"):
    if needle in install:
        ok(f"install_deps.sh 含 {needle}")
    else:
        bad(f"install_deps.sh 缺 {needle}")

for rel in ("setup/patch_aloam.sh", "setup/patch_lio_sam.sh",
            "setup/fix_gtsam_cmake.sh", "experiments/run_bench.sh"):
    p = os.path.join(ROOT, rel)
    if os.path.isfile(p):
        ok(f"{rel} 存在")
    else:
        bad(f"{rel} 缺失")

patch = read(os.path.join(ROOT, "setup", "patch_aloam.sh"))
for needle in ("ceres::Manifold", "EigenQuaternionManifold",
               "CMAKE_CXX_STANDARD 17", "declare_parameter<double>",
               "rosbag2_cpp"):
    if needle in patch:
        ok(f"patch_aloam.sh 处理了 {needle}")
    else:
        bad(f"patch_aloam.sh 没处理 {needle}")

# ================================================= 编码与换行（Windows 编辑 Linux 项目）
print("\n=== 编码 / 换行 ===")
SKIP_DIRS = (".git", ".acl-recovery", "build", "install", "log",
             ".deck-preview", ".deck-selftest", "__pycache__")
enc_files = []
for pattern in ("*.sh", "*.py", "*.yaml", "*.yml", "*.sdf", "*.md",
                "*.rviz", "*.repos", "*.txt", "*.xml", "*.launch.py"):
    for p in glob.glob(os.path.join(ROOT, "**", pattern), recursive=True):
        rel = os.path.relpath(p, ROOT).replace("\\", "/")
        if any(f"/{d}/" in f"/{rel}/" or rel.startswith(d + "/")
               for d in SKIP_DIRS):
            continue
        enc_files.append(p)
bom = []
crlf = []
for p in enc_files:
    with open(p, "rb") as f:
        head = f.read(3)
        f.seek(0)
        body = f.read()
    if head == b"\xef\xbb\xbf":
        bom.append(os.path.relpath(p, ROOT))
    if p.endswith((".sh", ".py")) and b"\r\n" in body:
        crlf.append(os.path.relpath(p, ROOT))
if bom:
    bad(f"这些文件带 UTF-8 BOM，Linux 上 shebang 会失效: {bom}")
else:
    ok(f"{len(enc_files)} 个文本文件都没有 BOM")
if crlf:
    bad(f"这些 .sh/.py 是 CRLF，Linux 上会报 bad interpreter: {crlf}")
else:
    ok("no CRLF in .sh/.py（.gitattributes 也强制了 LF）")

bench_readme = os.path.join(ROOT, "experiments", "README.md")
if os.path.isfile(bench_readme) and len(read(bench_readme)) > 1500:
    ok("experiments/README.md 有内容（实验协议）")
else:
    bad("experiments/README.md 缺失或过短")

# ============================================ 场景可行性（离线运动学 + 碰撞）
print("\n=== 场景可行性：轨迹能不能开出来 ===")
try:
    sys.path.insert(0, os.path.join(ROOT, "experiments"))
    import check_scenarios as _cs
    for p in scen_files:
        sname = os.path.splitext(os.path.basename(p))[0]
        r = _cs.check_scenario(sname, _cs.WORLDS)
        if "error" in r:
            bad(f"{sname}: {r['error']}")
        elif r["hit_steps"]:
            bad(f"{sname}: 车体与 {sorted(r['hits'])} 重叠 {r['hit_steps']} 步"
                f"（真值会被碰撞污染）")
        else:
            ok(f"{sname}: {r['path_len']:.1f} m，{r['obstacles']} 个障碍，无碰撞")
except Exception as e:                                        # noqa: BLE001
    bad(f"场景可行性检查跑不起来: {e}")

# ============================================ shell 语法（bash -n，机器人上必有）
print("\n=== shell 脚本语法 ===")
_BASH = None
for cand in (shutil.which("bash"),
             r"C:\Program Files\Git\bin\bash.exe",
             r"C:\Program Files\Git\usr\bin\bash.exe",
             "/bin/bash", "/usr/bin/bash"):
    if cand and os.path.isfile(cand):
        _BASH = cand
        break
if _BASH is None:
    ok("找不到 bash，跳过 shell 语法检查")
else:
    sh_files = sorted(glob.glob(os.path.join(ROOT, "setup", "*.sh")) +
                      glob.glob(os.path.join(ROOT, "experiments", "*.sh")))
    for p in sh_files:
        rel = os.path.relpath(p, ROOT)
        try:
            r = subprocess.run([_BASH, "-n", p], capture_output=True,
                               text=True, timeout=60)
            if r.returncode == 0:
                ok(f"{rel}: bash -n 通过")
            else:
                bad(f"{rel}: bash -n 报错 -> {(r.stderr or '').strip()[:160]}")
        except Exception as e:                                # noqa: BLE001
            bad(f"{rel}: 跑 bash -n 失败 {e}")

# ================================================================ 结论
print("\n=== 结论 ===")
print(f"{passed} 项通过, {fail} 项失败")
print("ALL PASS" if fail == 0 else f"{fail} FAILED")
sys.exit(1 if fail else 0)
