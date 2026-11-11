#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
_test_nodes.py —— 不装 ROS，也能把 gt_sync / traj_recorder 真正跑一遍
===============================================================================

问题：这两个节点每一次实验都会跑，但在写报告的机器上没有 ROS，它们从来没被执行过。
只看代码很容易漏掉：slerp 插值错、时间区间取错、雷达偏移转错方向、CSV 列写错。

办法：把 rclpy 和那几个消息类型换成**最小桩模块**（写在 .nodes-test/stub 里），
然后用 `PYTHONPATH` 让被测节点 import 到桩。这样节点本身一行不改，
`on_gt` / `on_cloud` / `on_est` 这些回调会被真的调用，输出也被真的检查。

验的是"逻辑"，不是"ROS 集成"：
  * gt_sync  —— 给定真值轨迹，查询时刻的插值位姿必须等于解析解；
                模型系->雷达系的偏移必须随姿态正确旋转；时间戳必须沿用点云的。
  * recorder —— 写出来的 CSV 必须是指定列、行数正确、数值精度够；
                meta.json 必须在 close() 时落盘。

用法： python3 setup/_test_nodes.py [--keep]
"""

import importlib
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORK = os.path.join(ROOT, '.nodes-test')
BENCH_SCRIPTS = os.path.join(ROOT, 'ros2_ws', 'src', 'lio_sam_bench', 'scripts')

FAILS = []


def check(name, cond, detail=''):
    if cond:
        print(f'  [ OK ] {name} {detail}')
    else:
        FAILS.append(name)
        print(f'  [FAIL] {name} {detail}')


# ============================================================ 桩：rclpy
STUB_RCLPY = '''"""最小 rclpy 桩：只实现被测节点用到的那几个 API。"""

# 测试在构造节点前把参数塞这里，模拟 launch 传参（节点 __init__ 里 declare 时生效）
PRESETS = {}


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg):
        self.lines.append((level, msg))

    def info(self, msg, *a, **k):    self._log('info', msg)
    def warning(self, msg, *a, **k): self._log('warning', msg)
    def warn(self, msg, *a, **k):    self._log('warning', msg)
    def error(self, msg, *a, **k):   self._log('error', msg)


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        clock = self

        class _Stamp:
            sec = int(clock.t)
            nanosec = int((clock.t - int(clock.t)) * 1e9)

        class _Time:
            def to_msg(self_inner):
                return _Stamp()
        return _Time()


class Node:
    instances = []

    def __init__(self, name='stub_node'):
        self.name = name
        self._params = {}
        self._subs = {}
        self._pubs = {}
        self._logger = _Logger()
        self._clock = _Clock()
        Node.instances.append(self)

    # --- 参数
    def declare_parameter(self, name, value=None):
        if name in PRESETS:
            value = PRESETS[name]
        self._params[name] = value
        return _Param(value)

    def get_parameter(self, name):
        return _Param(self._params.get(name))

    # --- 通信（只登记，由测试手动触发回调）
    def create_publisher(self, msg_type, topic, qos=None):
        pub = _Publisher(topic)
        self._pubs[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos=None):
        self._subs[topic] = callback
        return callback

    def create_timer(self, period, cb):
        return None

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        return None


def init(*a, **k):
    return None


def shutdown(*a, **k):
    return None


def ok():
    return True


def spin(node):
    return None
'''

STUB_QOS = '''"""QoS 桩：只要枚举名存在、QoSProfile 能吃关键字参数即可。"""


class ReliabilityPolicy:
    RELIABLE = 'reliable'
    BEST_EFFORT = 'best_effort'


class HistoryPolicy:
    KEEP_LAST = 'keep_last'
    KEEP_ALL = 'keep_all'


class QoSProfile:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
'''

# ============================================================ 桩：消息
STUB_MSG_COMMON = '''"""消息桩：字段名与 ROS 2 一致，够被测节点读写即可。"""


class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self):
        self.stamp = _Stamp()
        self.frame_id = ''


class _Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Pose:
    def __init__(self):
        self.position = _Vector3()
        self.orientation = _Quaternion()


class _PoseWithCovariance:
    def __init__(self):
        self.pose = _Pose()
        self.covariance = [0.0] * 36


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _TwistWithCovariance:
    def __init__(self):
        self.twist = _Twist()
        self.covariance = [0.0] * 36
'''

STUB_ODOMETRY = '''from ._common import (_Header, _PoseWithCovariance, _TwistWithCovariance)


class Odometry:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ''
        self.pose = _PoseWithCovariance()
        self.twist = _TwistWithCovariance()
'''

STUB_POINTCLOUD2 = '''from ._common import _Header


class PointCloud2:
    def __init__(self):
        self.header = _Header()
        self.height = 1
        self.width = 0
        self.point_step = 24
        self.row_step = 0
        self.is_dense = True
        self.fields = []
        self.data = b''
'''

STUB_PATH = '''from ._common import _Header


class Path:
    def __init__(self):
        self.header = _Header()
        self.poses = []
'''

STUB_POSESTAMPED = '''from ._common import _Header, _Pose


class PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()
'''

STUB_IMU = '''from ._common import _Header, _Quaternion, _Vector3


class Imu:
    def __init__(self):
        self.header = _Header()
        self.orientation = _Quaternion()
        self.angular_velocity = _Vector3()
        self.linear_acceleration = _Vector3()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9
'''


def build_stub(root):
    """把桩包写到 root 下，返回 PYTHONPATH 应加的那个目录。"""
    def w(path, text):
        full = os.path.join(root, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, 'w', encoding='utf-8') as f:
            f.write(text)

    w('rclpy/__init__.py', STUB_RCLPY)
    w('rclpy/node.py', 'from . import Node\n__all__ = ["Node"]\n')
    w('rclpy/qos.py', STUB_QOS)
    # 每个消息包各放一份 _common：桩里的相对 import 必须能解析
    for pkg in ('nav_msgs', 'geometry_msgs', 'sensor_msgs'):
        w(f'{pkg}/__init__.py', '')
        w(f'{pkg}/msg/_common.py', STUB_MSG_COMMON)
    w('nav_msgs/msg/__init__.py', STUB_ODOMETRY + '\n' + STUB_PATH)
    w('geometry_msgs/msg/__init__.py', STUB_POSESTAMPED)
    w('sensor_msgs/msg/__init__.py', STUB_POINTCLOUD2 + '\n' + STUB_IMU)
    return root


def load_module(name, stub_root):
    for mod in list(sys.modules):
        if mod in ('rclpy', 'nav_msgs', 'geometry_msgs', 'sensor_msgs') or \
           mod.startswith(('rclpy.', 'nav_msgs.', 'geometry_msgs.', 'sensor_msgs.')):
            del sys.modules[mod]
    if stub_root not in sys.path:
        sys.path.insert(0, stub_root)
    if BENCH_SCRIPTS not in sys.path:
        sys.path.insert(0, BENCH_SCRIPTS)
    return importlib.import_module(name)


# ============================================================ 小工具
def make_gt(x, y, z, yaw=0.0, roll=0.0, t=0.0):
    """造一条 nav_msgs/Odometry 真值（欧拉角用四元数表示）。"""
    import nav_msgs.msg as navmsg
    m = navmsg.Odometry()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.header.frame_id = 'world'
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.position.z = z
    # 先 roll 再 yaw 的复合旋转
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    m.pose.pose.orientation.x = sr * cy
    m.pose.pose.orientation.y = cr * sy
    m.pose.pose.orientation.z = -sr * sy
    m.pose.pose.orientation.w = cr * cy
    return m


def make_cloud(t):
    import sensor_msgs.msg as smsg
    m = smsg.PointCloud2()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    return m


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ============================================================ 测试
def preset(**kw):
    """在构造节点前塞参数，模拟 ros2 launch 的 parameters= 行为。"""
    import rclpy
    rclpy.PRESETS.clear()
    rclpy.PRESETS.update(kw)


def test_gt_sync(stub_root):
    print('=== 1. gt_sync：插值 + 模型系->雷达系偏移 ===')
    gs = load_module('gt_sync', stub_root)
    preset(lidar_z_offset=0.40, world_frame='world', child_frame='lidar_link',
           path_rate_hz=1.0, buffer_seconds=10.0, gt_topic='/gt/odom',
           cloud_topic='/points', out_topic='/gt/pose_sync', path_topic='/gt/path')
    node = gs.GtSync()

    on_gt = node._subs['/gt/odom']
    on_cloud = node._subs['/points']
    pub = node._pubs['/gt/pose_sync']
    # 真值：沿 +x 以 1 m/s 走，姿态水平，10.00 / 10.10 / 10.20
    for i in range(3):
        on_gt(make_gt(x=0.1 * i, y=0.0, z=0.12, t=10.0 + 0.1 * i))

    # 在 10.05 查：线性插值应给 x=0.05，雷达在模型上方 0.40 -> z=0.52
    on_cloud(make_cloud(10.05))
    check('发了一条真值', len(pub.sent) == 1, f'{len(pub.sent)} 条')
    if pub.sent:
        out = pub.sent[-1]
        check('位置线性插值正确（x=0.05）', abs(out.pose.pose.position.x - 0.05) < 1e-6,
              f'x={out.pose.pose.position.x:.6f}')
        check('雷达偏移加到 z（0.12+0.40=0.52）',
              abs(out.pose.pose.position.z - 0.52) < 1e-6,
              f'z={out.pose.pose.position.z:.6f}')
        check('时间戳沿用点云（10.05）', abs(stamp_of(out) - 10.05) < 1e-9,
              f'{stamp_of(out):.6f}')
        check('frame_id = world / child = lidar_link',
              out.header.frame_id == 'world' and out.child_frame_id == 'lidar_link',
              f'{out.header.frame_id} -> {out.child_frame_id}')

    # 姿态为 roll=90 时，(0,0,0.40) 应被转到 (0,-0.40,0)
    preset(lidar_z_offset=0.40, world_frame='world', child_frame='lidar_link',
           path_rate_hz=1.0, buffer_seconds=10.0, gt_topic='/gt/odom',
           cloud_topic='/points', out_topic='/gt/pose_sync', path_topic='/gt/path')
    gs2 = gs.GtSync()
    on_gt2, on_cloud2 = gs2._subs['/gt/odom'], gs2._subs['/points']
    pub2 = gs2._pubs['/gt/pose_sync']
    for i in range(2):
        on_gt2(make_gt(x=0.0, y=0.0, z=0.0, roll=math.pi / 2, t=20.0 + 0.1 * i))
    on_cloud2(make_cloud(20.05))
    if pub2.sent:
        p = pub2.sent[-1].pose.pose.position
        check('roll=90° 时偏移转到 -y（0,-0.40,0）',
              abs(p.x) < 1e-6 and abs(p.y + 0.40) < 1e-6 and abs(p.z) < 1e-6,
              f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    # 姿态插值：0° -> 90° roll，中点应≈45°
    preset(lidar_z_offset=0.40, world_frame='world', child_frame='lidar_link',
           path_rate_hz=1.0, buffer_seconds=10.0, gt_topic='/gt/odom',
           cloud_topic='/points', out_topic='/gt/pose_sync', path_topic='/gt/path')
    gs3 = gs.GtSync()
    on_gt3, on_cloud3 = gs3._subs['/gt/odom'], gs3._subs['/points']
    pub3 = gs3._pubs['/gt/pose_sync']
    on_gt3(make_gt(0, 0, 0, roll=0.0, t=30.0))
    on_gt3(make_gt(0, 0, 0, roll=math.pi / 2, t=30.1))
    on_cloud3(make_cloud(30.05))
    if pub3.sent:
        p = pub3.sent[-1].pose.pose.position
        # 45° 时 (0,0,0.4) -> (0, -0.4*sin45, 0.4*cos45) = (0, -0.2828, 0.2828)
        check('姿态 slerp 到 45°（偏移应为 (0,-0.283,0.283)）',
              abs(p.y + 0.4 * math.sin(math.pi / 4)) < 5e-3
              and abs(p.z - 0.4 * math.cos(math.pi / 4)) < 5e-3,
              f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    # 早于第一帧真值：用最近值 + 计数
    before = len(pub3.sent)
    on_cloud3(make_cloud(29.0))
    check('早于真值缓存时仍发布（用最近值）', len(pub3.sent) == before + 1)
    check('外插被计数', gs3._n_extrapolated >= 1, f'{gs3._n_extrapolated}')

    # 没有真值时不应发布
    preset(lidar_z_offset=0.40, world_frame='world', child_frame='lidar_link',
           path_rate_hz=1.0, buffer_seconds=10.0, gt_topic='/gt/odom',
           cloud_topic='/points', out_topic='/gt/pose_sync', path_topic='/gt/path')
    gs4 = gs.GtSync()
    gs4._subs['/points'](make_cloud(40.0))
    check('没有真值时一条都不发（而不是发垃圾）',
          len(gs4._pubs['/gt/pose_sync'].sent) == 0)


def test_recorder(stub_root):
    print('\n=== 2. traj_recorder：CSV 内容与 meta ===')
    tr = load_module('traj_recorder', stub_root)
    out_csv = os.path.join(WORK, 'rec.csv')
    preset(algo='loam', est_topic='/aft_mapped_to_init', gt_topic='/gt/pose_sync',
           cloud_topic='/points', out_csv=out_csv, flush_every=1)
    node = tr.TrajRecorder()
    import csv as _csv

    gt = make_gt(x=1.5, y=-2.5, z=0.52, yaw=0.7, t=12.34)
    est = make_gt(x=1.4, y=-2.6, z=0.50, yaw=0.6, t=12.0)
    node._subs['/gt/pose_sync'](gt)
    node._subs['/aft_mapped_to_init'](est)
    node.close()

    check('CSV 已写出', os.path.isfile(out_csv))
    with open(out_csv, encoding='utf-8') as f:
        rows = list(_csv.reader(f))
    check('表头列正确', rows[0] == ['kind', 'stamp', 'node_now', 'x', 'y', 'z',
                                   'qx', 'qy', 'qz', 'qw'], str(rows[0]))
    check('两行数据（gt + est）', len(rows) == 3, f'{len(rows) - 1} 行')
    check('gt 行 kind/stamp 正确', rows[1][0] == 'gt' and abs(float(rows[1][1]) - 12.34) < 1e-6,
          f'{rows[1][0]} @ {rows[1][1]}')
    check('gt 行位置正确', abs(float(rows[1][3]) - 1.5) < 1e-6
          and abs(float(rows[1][4]) + 2.5) < 1e-6, f'({rows[1][3]}, {rows[1][4]})')
    check('est 行 kind 正确', rows[2][0] == 'est')
    check('meta.json 已落盘', os.path.isfile(os.path.splitext(out_csv)[0] + '.meta.json'))
    if os.path.isfile(os.path.splitext(out_csv)[0] + '.meta.json'):
        import json
        meta = json.load(open(os.path.splitext(out_csv)[0] + '.meta.json', encoding='utf-8'))
        check('meta 记录了 algo 与计数',
              meta.get('algo') == 'loam' and meta['counts']['est'] == 1
              and meta['counts']['gt'] == 1, str(meta.get('counts')))


def main():
    keep = '--keep' in sys.argv
    if os.path.isdir(WORK):
        shutil.rmtree(WORK)
    os.makedirs(WORK, exist_ok=True)
    stub_root = build_stub(os.path.join(WORK, 'stub'))
    print(f'stub  : {stub_root}')
    print(f'work  : {WORK}\n')

    test_gt_sync(stub_root)
    test_recorder(stub_root)

    print('\n=== 结论 ===')
    if FAILS:
        print(f'{len(FAILS)} FAILED: {FAILS}')
        print(f'（产物保留在 {WORK}）')
        return 1
    print('ALL PASS —— gt_sync / traj_recorder 的逻辑被真的执行过一遍')
    if not keep:
        shutil.rmtree(WORK, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
