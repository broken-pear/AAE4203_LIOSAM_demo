#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gt_sync.py —— 把 Gazebo 的真值位姿，对齐到"每一帧点云"的时间戳上
===============================================================================

为什么需要这个节点
------------------
评测要回答的问题很朴素：**算法在第 k 帧给出的位姿，和车在第 k 帧的真值差多少？**

难点在于三条时间线的分辨率不同：

  真值 /gt/odom      gz 的 OdometryPublisher 以 100 Hz 发布（时间戳很细）
  点云 /points       10 Hz，每一帧有自己的时间戳（仿真时间）
  算法输出           A-LOAM 的时间戳**只有整数秒**（它把 header.stamp.sec
                     当 double 传来传去，纳秒被丢掉了）；LIO-SAM 的时间戳是准的

如果直接拿算法的时间戳去找"最接近的真值"，A-LOAM 会错配到同一秒里的任意一帧
（10 Hz 下一秒钟有 10 帧，误差可以到 0.1 s → 0.4 m/s 时就是 4 cm 的虚假误差）。

所以这个节点把真值**重采样到点云的时间戳**上：

    /points 到达  ->  用 header.stamp 作为查询时刻 t_c
                     在真值缓冲里线性插值位置、球面插值旋转
                     再把"模型系"位姿换成"雷达系"位姿（见下）
                     以 t_c 为时间戳发 /gt/pose_sync

于是评测侧只需要：每一帧点云对应一条 /gt/pose_sync，时间戳完全一致。

坐标补偿：模型系 -> 雷达系
--------------------------
OdometryPublisher 发的是 worldPose(模型)，也就是 SDF 里 <model> 的原点。
而两条算法估计的都是**雷达**的位姿（LIO-SAM 解的就是 lidar_link；A-LOAM 解的是
雷达坐标系）。model.sdf 里雷达挂在 z = 0.40 m 处，所以：

    p_lidar = p_model + R(q_model) · (0, 0, lidar_z_offset)
    q_lidar = q_model          （雷达相对模型只有平移，没有旋转）

这个偏移是**常量**，ATE 用 SE(3) 对齐后会被吸收掉，但它会让"未对齐的绝对误差"
凭空多出 40 cm —— 所以这里必须补，让真值和算法估计的是同一个物理点。

话题
----
  订阅  /gt/odom        nav_msgs/Odometry   真值（world -> lio_bot/base_link）
  订阅  /points         sensor_msgs/PointCloud2   只用来取时间戳
  发布  /gt/pose_sync   nav_msgs/Odometry   插值后的真值（world -> lidar_link）
  发布  /gt/path        nav_msgs/Path       真值轨迹（RViz 里对照用，1 Hz）

参数
----
  gt_topic         /gt/odom
  cloud_topic      /points
  out_topic        /gt/pose_sync
  path_topic       /gt/path
  lidar_z_offset   0.40     雷达相对模型原点的高度（看 model.sdf）
  world_frame      world    真值所在的世界坐标系名
  child_frame      lidar_link
  path_rate_hz     1.0      轨迹话题的发布频率（10 Hz 太重）
  buffer_seconds   10.0     真值缓存长度
"""

import bisect
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2


# --------------------------------------------------------------------------- #
# 四元数小工具（只用到 slerp，不引第三方库，免得 Jetson 上多装东西）
# --------------------------------------------------------------------------- #
def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / n for c in q)


def _quat_slerp(q0, q1, s):
    """球面线性插值。q0/q1 需已归一化，返回已归一化。"""
    d = sum(a * b for a, b in zip(q0, q1))
    if d < 0.0:                      # 取最短路径
        q1 = tuple(-c for c in q1)
        d = -d
    d = max(-1.0, min(1.0, d))
    if d > 0.9995:                   # 夹角太小，线性插值足够，避免除零
        return _quat_normalize(tuple(a + s * (b - a) for a, b in zip(q0, q1)))
    theta0 = math.acos(d)
    theta = theta0 * s
    sin_theta0 = math.sin(theta0)
    w0 = math.sin(theta0 - theta) / sin_theta0
    w1 = math.sin(theta) / sin_theta0
    return _quat_normalize(tuple(w0 * a + w1 * b for a, b in zip(q0, q1)))


def _quat_rotate(q, v):
    """用四元数旋转向量 v（q 需归一化）。"""
    qv = (v[0], v[1], v[2], 0.0)
    q_conj = (-q[0], -q[1], -q[2], q[3])
    r = _quat_mul(_quat_mul(q, qv), q_conj)
    return (r[0], r[1], r[2])


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _pose_to_tuple(msg_pose):
    p = msg_pose.position
    q = msg_pose.orientation
    return (p.x, p.y, p.z), _quat_normalize((q.x, q.y, q.z, q.w))


# --------------------------------------------------------------------------- #
class GtSync(Node):
    def __init__(self):
        super().__init__('gt_sync')

        self.declare_parameter('gt_topic', '/gt/odom')
        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('out_topic', '/gt/pose_sync')
        self.declare_parameter('path_topic', '/gt/path')
        self.declare_parameter('lidar_z_offset', 0.40)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('child_frame', 'lidar_link')
        self.declare_parameter('path_rate_hz', 1.0)
        self.declare_parameter('buffer_seconds', 10.0)

        gp = self.get_parameter
        self.gt_topic = gp('gt_topic').value
        self.cloud_topic = gp('cloud_topic').value
        self.out_topic = gp('out_topic').value
        self.path_topic = gp('path_topic').value
        self.lidar_z = float(gp('lidar_z_offset').value)
        self.world_frame = gp('world_frame').value
        self.child_frame = gp('child_frame').value
        self.buffer_s = float(gp('buffer_seconds').value)

        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)

        # 真值缓存：三个等长列表，按时间戳升序
        self._t = []
        self._p = []
        self._q = []

        self._n_gt = 0
        self._n_cloud = 0
        self._n_extrapolated = 0
        self._last_path_t = -1e9
        self._path = Path()
        self._path.header.frame_id = self.world_frame

        self.pub_gt = self.create_publisher(Odometry, self.out_topic, pub_qos)
        self.pub_path = self.create_publisher(Path, self.path_topic, pub_qos)

        self.create_subscription(
            Odometry, self.gt_topic, self.on_gt, pub_qos)
        self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud, sensor_qos)

        self.get_logger().info(
            f'gt_sync: {self.gt_topic} -> {self.out_topic} '
            f'(lidar_z_offset={self.lidar_z:.3f} m, frame={self.world_frame})'
        )

    # ------------------------------------------------------------------ 真值
    def on_gt(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)
        p, q = _pose_to_tuple(msg.pose.pose)

        # 乱序/重复时间戳直接丢（bag 回溯、双 /clock 都会造成这个）
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._p.append(p)
        self._q.append(q)
        self._n_gt += 1

        # 裁掉过期缓存
        t_min = t - self.buffer_s
        if len(self._t) > 2 and self._t[0] < t_min:
            k = bisect.bisect_left(self._t, t_min)
            del self._t[:k]
            del self._p[:k]
            del self._q[:k]

    # -------------------------------------------------------------- 点云时间
    def on_cloud(self, msg: PointCloud2):
        if not self._t:
            if self._n_cloud == 0:
                self.get_logger().warning(
                    f'还没收到 {self.gt_topic}，先不发真值（检查桥接/插件）')
            self._n_cloud += 1
            return

        t_c = _stamp_to_sec(msg.header.stamp)
        p, q = self._interpolate(t_c)
        if p is None:
            return

        # 模型系 -> 雷达系（常量平移，跟着车的姿态转）
        offset = _quat_rotate(q, (0.0, 0.0, self.lidar_z))
        p_lidar = (p[0] + offset[0], p[1] + offset[1], p[2] + offset[2])

        out = Odometry()
        out.header.stamp = msg.header.stamp        # ★ 用点云的时间戳
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.child_frame
        out.pose.pose.position.x = p_lidar[0]
        out.pose.pose.position.y = p_lidar[1]
        out.pose.pose.position.z = p_lidar[2]
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]
        self.pub_gt.publish(out)
        self._n_cloud += 1

        # 真值轨迹（低频发，避免 ROS 侧带宽被一条 Path 吃满）
        rate = max(float(self.get_parameter('path_rate_hz').value), 1e-3)
        if t_c - self._last_path_t >= 1.0 / rate:
            self._last_path_t = t_c
            ps = PoseStamped()
            ps.header = out.header
            ps.pose = out.pose.pose
            self._path.header.stamp = out.header.stamp
            self._path.poses.append(ps)
            self.pub_path.publish(self._path)

        if self._n_cloud % 100 == 0:
            self.get_logger().info(
                f'已同步 {self._n_cloud} 帧真值（真值消息 {self._n_gt} 条，'
                f'外插 {self._n_extrapolated} 次）'
            )

    # -------------------------------------------------------------- 插值核心
    def _interpolate(self, t_c):
        ts = self._t
        if t_c <= ts[0]:
            # 早于缓存里最早的真值：用最近的一帧，并计数（评测里要看这个数）
            self._n_extrapolated += 1
            return self._p[0], self._q[0]
        if t_c >= ts[-1]:
            self._n_extrapolated += 1
            return self._p[-1], self._q[-1]

        i = bisect.bisect_right(ts, t_c) - 1
        t0, t1 = ts[i], ts[i + 1]
        dt = t1 - t0
        s = 0.0 if dt <= 0.0 else (t_c - t0) / dt
        p0, p1 = self._p[i], self._p[i + 1]
        p = tuple(a + s * (b - a) for a, b in zip(p0, p1))
        q = _quat_slerp(self._q[i], self._q[i + 1], s)
        return p, q


def main():
    rclpy.init()
    node = GtSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
