#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_prep.py —— 把 Gazebo 的传感器数据"翻译"成 LIO-SAM 能吃的格式
====================================================================

为什么需要这个节点？
--------------------
LIO-SAM 的 imageProjection.cpp 里，点云被当作 Velodyne 的 PointXYZIRT 处理::

    struct VelodynePointXYZIRT {
        float x, y, z, intensity;
        uint16_t ring;   // 第几线
        float    time;   // 该点相对这一帧起始时刻的偏移（秒）
    };

而 Gazebo 的 gpu_lidar 通过 ros_gz_bridge 出来的 PointCloud2 只有
``x y z intensity`` 四个字段，没有 ``ring`` 也没有 ``time``；
并且 header.frame_id 往往是 ``lio_bot/lidar_link/lidar`` 这种 gz 内部名字。

所以本节点做两件事：

  1) /points_raw  (x,y,z,intensity)                --> /points    (x,y,z,intensity,ring,time)
     * ring 由点的垂直角度反算（16 线，±15°，每线 2°）
     * time 由点的方位角反算；默认 time_period=0.0 表示"这一帧是瞬时扫完的"
       （Gazebo 的激光雷达不是真的旋转，一帧就是一瞬间）
     * frame_id 强制写成 lidar_link

  2) /imu_raw (sensor_msgs/Imu)                     --> /imu/data
     * frame_id 强制写成 imu_link
     * 如果 orientation 不是合法四元数（有些仿真器会发全 0），用重力方向补一个
       roll/pitch，保证 LIO-SAM 不会因为 "Invalid quaternion" 直接退出。

参数（都可以用 ros2 param 改）：
  in_cloud_topic   : /points_raw
  out_cloud_topic  : /points
  in_imu_topic     : /imu_raw
  out_imu_topic    : /imu/data
  cloud_frame_id   : lidar_link
  imu_frame_id     : imu_link
  n_scan           : 16       激光线数，必须和 LIO-SAM 的 N_SCAN 一致
  vertical_fov_deg : 30.0     垂直视场角（±15°），用来把垂直角映射成 ring
  time_period      : 0.0      非 0 时按方位角给每点分配 0~period 的相对时间
  min_range        : 0.15     过滤太近/无效的点
  max_range        : 45.0     过滤太远/无效的点
  imu_gravity_mode : auto     auto/on/off —— 是否给 IMU 的线性加速度补上重力
  gravity_magnitude: 9.80511  和 world 的 <gravity> 一致

关于 imu_gravity_mode（重要）
----------------------------
SDF 规范里 `linear_acceleration` 是**含重力**的比力：车静止水平时 z 应该读出 +9.8。
有些仿真器/版本给出的是"去掉重力"的纯运动加速度（静止时读到 0）。这不影响 A-LOAM
（它根本不用 IMU），但 LIO-SAM 的 IMU 预积分要靠重力方向确定姿态：重力缺失时
"加速度计偏置"和"俯仰角"互相不可分辨，因子图直接**秩亏**，表现为
`gtsam::IndeterminantLinearSystemException`（underconstrained variables）。

auto 模式在**第一条** IMU 上判断：|a| < 5 m/s² 就认为重力缺失，按姿态把
世界系重力转到 IMU 系补上；|a| ≈ 9.8 则原样转发。判断只做一次，并在日志里说明，
所以既不会重复加重力，也不需要人工切换。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, PointCloud2, PointField

# 输出的 Velodyne 格式：point_step = 24
_OUT_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring',      offset=16, datatype=PointField.UINT16,  count=1),
    PointField(name='time',      offset=20, datatype=PointField.FLOAT32, count=1),
]

_OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})


def _read_xyz(msg):
    """从 PointCloud2 里直接按 offsets 读出 x / y / z / intensity（用 numpy，零拷贝）。"""
    by_name = {f.name: f for f in msg.fields}
    for axis in ('x', 'y', 'z'):
        if axis not in by_name:
            return None

    n = msg.width * msg.height
    if n == 0:
        return None

    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': ['<f4', '<f4', '<f4'],
        'offsets': [by_name['x'].offset, by_name['y'].offset, by_name['z'].offset],
        'itemsize': msg.point_step,
    })
    arr = np.frombuffer(msg.data, dtype=dtype, count=n)
    x = arr['x'].astype(np.float32, copy=True)
    y = arr['y'].astype(np.float32, copy=True)
    z = arr['z'].astype(np.float32, copy=True)

    if 'intensity' in by_name:
        dtype_i = np.dtype({'names': ['i'], 'formats': ['<f4'],
                            'offsets': [by_name['intensity'].offset],
                            'itemsize': msg.point_step})
        intensity = np.frombuffer(msg.data, dtype=dtype_i, count=n)['i'].copy()
    else:
        intensity = np.zeros(n, dtype=np.float32)
    return x, y, z, intensity


class SensorPrep(Node):
    def __init__(self):
        super().__init__('lio_sam_sensor_prep')

        self.declare_parameter('in_cloud_topic', '/points_raw')
        self.declare_parameter('out_cloud_topic', '/points')
        self.declare_parameter('in_imu_topic', '/imu_raw')
        self.declare_parameter('out_imu_topic', '/imu/data')
        self.declare_parameter('cloud_frame_id', 'lidar_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('n_scan', 16)
        self.declare_parameter('vertical_fov_deg', 30.0)
        self.declare_parameter('time_period', 0.0)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 45.0)
        self.declare_parameter('imu_gravity_mode', 'auto')
        self.declare_parameter('gravity_magnitude', 9.80511)

        gp = self.get_parameter
        self.in_cloud = gp('in_cloud_topic').value
        self.out_cloud = gp('out_cloud_topic').value
        self.in_imu = gp('in_imu_topic').value
        self.out_imu = gp('out_imu_topic').value
        self.cloud_frame = gp('cloud_frame_id').value
        self.imu_frame = gp('imu_frame_id').value
        self.n_scan = int(gp('n_scan').value)
        self.half_fov = float(gp('vertical_fov_deg').value) * 0.5
        self.time_period = float(gp('time_period').value)
        self.min_range = float(gp('min_range').value)
        self.max_range = float(gp('max_range').value)
        self.gravity_mode = str(gp('imu_gravity_mode').value).lower()
        self.gravity_mag = float(gp('gravity_magnitude').value)
        # None = 还没判断；True/False = 已决定是否补重力
        self._add_gravity = None

        # gz -> ROS 的桥接通常发 best_effort，这里用 SensorData 级别订阅最保险
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # 发布端用 reliable：LIO-SAM 订阅 best_effort 能收到，RViz2 默认 reliable 也能收到
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub_cloud = self.create_publisher(PointCloud2, self.out_cloud, pub_qos)
        self.pub_imu = self.create_publisher(Imu, self.out_imu, pub_qos)
        self.create_subscription(PointCloud2, self.in_cloud, self.on_cloud, sensor_qos)
        self.create_subscription(Imu, self.in_imu, self.on_imu, sensor_qos)

        self._n_cloud = 0
        self._n_imu = 0
        self._warned_imu = False
        self._last_imu_t = None       # 上一条已转发 IMU 的时间戳（用于丢重复）
        self._n_imu_dropped = 0
        self._warned_fields = False
        self.get_logger().info(
            f'point cloud: {self.in_cloud} -> {self.out_cloud} '
            f'(frame={self.cloud_frame}, n_scan={self.n_scan})'
        )
        self.get_logger().info(
            f'imu        : {self.in_imu} -> {self.out_imu} (frame={self.imu_frame})'
        )

    # ------------------------------------------------------------------ 点云
    def on_cloud(self, msg: PointCloud2):
        data = _read_xyz(msg)
        if data is None:
            if not self._warned_fields:
                self._warned_fields = True
                self.get_logger().error(
                    '点云缺少 x/y/z 字段！实际字段 = '
                    f'{[f.name for f in msg.fields]}'
                )
            return
        x, y, z, intensity = data

        r = np.sqrt(x * x + y * y + z * z)
        good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        good &= (r > self.min_range) & (r < self.max_range)
        if not np.any(good):
            return
        x, y, z, intensity = x[good], y[good], z[good], intensity[good]

        # 垂直角度 -> 线号 ring
        elev_deg = np.degrees(np.arctan2(z, np.hypot(x, y)))
        step = self.half_fov * 2.0 / max(self.n_scan - 1, 1)
        ring = np.rint((elev_deg + self.half_fov) / step)
        ring = np.clip(ring, 0, self.n_scan - 1).astype(np.uint16)

        # 方位角 -> 相对时间 time（time_period = 0 时全 0，即"瞬时扫描"）
        if self.time_period > 0.0:
            az = np.arctan2(y, x)                   # [-pi, pi]
            time = ((az + math.pi) / (2.0 * math.pi) * self.time_period)
        else:
            time = np.zeros_like(x)

        n = x.shape[0]
        out = np.empty(n, dtype=_OUT_DTYPE)
        out['x'] = x
        out['y'] = y
        out['z'] = z
        out['intensity'] = intensity
        out['ring'] = ring
        out['time'] = time.astype(np.float32)

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self.cloud_frame
        cloud.height = 1
        cloud.width = n
        cloud.fields = _OUT_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = 24
        cloud.row_step = 24 * n
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.pub_cloud.publish(cloud)

        self._n_cloud += 1
        if self._n_cloud % 50 == 0:
            self.get_logger().info(
                f'已转发 {self._n_cloud} 帧点云（最新 {n} 点/帧）'
            )

    # ------------------------------------------------------------------- IMU
    def on_imu(self, msg: Imu):
        # 时间戳必须严格递增：GTSAM 的预积分因子用 dt 计算，
        # 出现 dt <= 0 的样本会让那段区间的信息量为零，进而使
        # lio_sam_imuPreintegration 抛 IndeterminantLinearSystemException
        # （underconstrained variables）。仿真里传感器更新率和物理步长不同步时
        # 确实会出现重复时间戳，所以这里直接丢掉不平滑的样本并计数。
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_imu_t is not None and t <= self._last_imu_t + 1e-9:
            self._n_imu_dropped += 1
            if self._n_imu_dropped in (1, 10, 100) or self._n_imu_dropped % 500 == 0:
                self.get_logger().warning(
                    f'丢弃时间戳非递增的 IMU（第 {self._n_imu_dropped} 条，'
                    f't={t:.6f} <= {self._last_imu_t:.6f}）')
            return
        self._last_imu_t = t

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.imu_frame
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        o = msg.orientation
        norm2 = o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z
        if norm2 < 0.1:
            # 退化情况：用加速度计的重力方向补 roll / pitch，yaw 置 0
            a = msg.linear_acceleration
            an = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
            if an > 1e-3:
                roll = math.atan2(a.y, a.z)
                pitch = math.atan2(-a.x, math.hypot(a.y, a.z))
                cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
                cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
                out.orientation.x = sr * cp
                out.orientation.y = cr * sp
                out.orientation.z = -sr * sp
                out.orientation.w = cr * cp
            else:
                out.orientation.w = 1.0
            if not self._warned_imu:
                self._warned_imu = True
                self.get_logger().warning(
                    'Gazebo 发来的 IMU 没有合法的 orientation，已用重力方向补 roll/pitch。'
                )

        # ---- 重力补偿（只判断一次）----
        # 见文件顶部说明：重力缺失时 LIO-SAM 的因子图会秩亏。
        if self._add_gravity is None:
            a0 = msg.linear_acceleration
            an0 = math.sqrt(a0.x * a0.x + a0.y * a0.y + a0.z * a0.z)
            if self.gravity_mode == 'on':
                self._add_gravity = True
            elif self.gravity_mode == 'off':
                self._add_gravity = False
            else:                                   # auto
                self._add_gravity = an0 < 5.0
            if self._add_gravity:
                self.get_logger().warning(
                    f'IMU 首帧 |a| = {an0:.2f} m/s² —— 这条 IMU 的线性加速度**不含重力**，'
                    f'已按姿态补上 {self.gravity_mag:.3f} m/s²。'
                    f'（不补的话 LIO-SAM 的姿态不可观测，会抛 '
                    f'IndeterminantLinearSystemException）')
            else:
                self.get_logger().info(
                    f'IMU 首帧 |a| = {an0:.2f} m/s² —— 已含重力，原样转发')

        if self._add_gravity:
            o = out.orientation
            # 先归一化：非单位四元数会让重力模长偏差 norm^2 倍（已用数值测试确认过）
            nq = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
            if nq < 1e-6:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx, qy, qz, qw = o.x / nq, o.y / nq, o.z / nq, o.w / nq
            # R^T * (0, 0, g)：把世界系的重力转到 IMU 系
            gx = 2.0 * (qx * qz - qw * qy)
            gy = 2.0 * (qy * qz + qw * qx)
            gz = qw * qw - qx * qx - qy * qy + qz * qz
            g = self.gravity_mag
            out.linear_acceleration.x = msg.linear_acceleration.x + g * gx
            out.linear_acceleration.y = msg.linear_acceleration.y + g * gy
            out.linear_acceleration.z = msg.linear_acceleration.z + g * gz

        self.pub_imu.publish(out)

        self._n_imu += 1
        if self._n_imu % 1000 == 0:
            self.get_logger().info(f'已转发 {self._n_imu} 条 IMU')


def main():
    rclpy.init()
    node = SensorPrep()
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
