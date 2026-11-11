#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traj_recorder.py —— 把"算法位姿"和"同一帧的真值"逐帧写成 CSV
===============================================================================

设计原则：**录制器只做搬运，不做任何配对/对齐判断。**

配对是个容易出错、也容易被指责"调参"的环节，所以这里刻意保持笨：
  * 来一条 /gt/pose_sync  -> 写一行 kind=gt
  * 来一条算法的里程计    -> 写一行 kind=est
  * 每行都带自己的时间戳，以及节点收到它时的仿真时刻 node_now

真正的配对交给离线评测器 evaluate.py 做，那里有显式的关联规则和"关联合格率"
输出（matched_fraction），任何人都能复核。

CSV 格式（列固定，方便 pandas / Excel 直接读）
------------------------------------------------
  kind,stamp,node_now,x,y,z,qx,qy,qz,qw
    gt   真值，stamp = 该帧点云的仿真时间（由 gt_sync 保证）
    est  算法输出，stamp = 算法自己给的（LIO-SAM 精确；A-LOAM 只有整数秒）
    node_now = 节点收到消息时的仿真时刻，用来算延迟与吞吐

同步写一份同名 .meta.json，记录参数、条数、首末时间戳、算法名 —— 复核用。

参数
----
  algo            必需，loam / liosam（只用于打标签，不影响录制）
  est_topic       必需，要录的位姿话题
  gt_topic        /gt/pose_sync
  cloud_topic     /points（只用来计数，确认帧数对得上）
  out_csv         输出 CSV 路径（目录不存在会自动创建）
  flush_every     每写多少行落一次盘
"""

import csv
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TrajRecorder(Node):
    def __init__(self):
        super().__init__('traj_recorder')

        self.declare_parameter('algo', 'unknown')
        self.declare_parameter('est_topic', '')
        self.declare_parameter('gt_topic', '/gt/pose_sync')
        self.declare_parameter('cloud_topic', '/points')
        self.declare_parameter('out_csv', 'traj.csv')
        self.declare_parameter('flush_every', 50)

        gp = self.get_parameter
        self.algo = gp('algo').value
        self.est_topic = gp('est_topic').value
        self.gt_topic = gp('gt_topic').value
        self.cloud_topic = gp('cloud_topic').value
        self.out_csv = gp('out_csv').value
        self.flush_every = int(gp('flush_every').value)

        if not self.est_topic:
            raise RuntimeError('参数 est_topic 必须给（要录哪个位姿话题）')

        out_dir = os.path.dirname(os.path.abspath(self.out_csv))
        os.makedirs(out_dir, exist_ok=True)

        self._fh = open(self.out_csv, 'w', newline='', encoding='utf-8')
        self._w = csv.writer(self._fh)
        self._w.writerow(['kind', 'stamp', 'node_now',
                          'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        self._n_gt = 0
        self._n_est = 0
        self._n_cloud = 0
        self._since_flush = 0
        self._gt_t = [None, None]         # 首/末时间戳
        self._est_t = [None, None]
        self._closed = False

        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50)
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- 估计位姿必须用 BEST_EFFORT 订阅，原因很具体 ----
        # LIO-SAM 发 /lio_sam/mapping/odometry 用的是 BEST_EFFORT（上游 utility.hpp
        # 里 qos_profile 就写着 RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT，depth=1）。
        # DDS 的兼容规则是"发布者提供 ≥ 订阅者要求"：
        #   RELIABLE 订阅者 + BEST_EFFORT 发布者 => **不兼容，一条消息都收不到**。
        # 这个坑在真机上表现为"LIO-SAM 录到 0 帧"，而算法其实一直在正常发布 ——
        # 因为 A-LOAM 的 /aft_mapped_to_init 是 RELIABLE 发的，同一份代码就收得到。
        # BEST_EFFORT 订阅者与两种发布者都兼容，所以这里统一用它。
        est_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                             reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Odometry, self.est_topic, self.on_est, est_qos)
        self.create_subscription(Odometry, self.gt_topic, self.on_gt, est_qos)
        self.create_subscription(PointCloud2, self.cloud_topic,
                                 self.on_cloud, sensor_qos)

        self.get_logger().info(f'[{self.algo}] recording {self.est_topic} '
                               f'+ {self.gt_topic}  ->  {self.out_csv}')

    # ---------------------------------------------------------------- 落盘
    def _write(self, kind, stamp, msg_pose):
        p = msg_pose.position
        q = msg_pose.orientation
        now = _stamp_to_sec(self.get_clock().now().to_msg())
        self._w.writerow([kind, f'{stamp:.9f}', f'{now:.9f}',
                          f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
                          f'{q.x:.7f}', f'{q.y:.7f}', f'{q.z:.7f}', f'{q.w:.7f}'])
        self._since_flush += 1
        if self._since_flush >= self.flush_every:
            self._fh.flush()
            self._since_flush = 0

    def on_est(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)
        self._write('est', t, msg.pose.pose)
        self._n_est += 1
        if self._est_t[0] is None:
            self._est_t[0] = t
        self._est_t[1] = t

    def on_gt(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)
        self._write('gt', t, msg.pose.pose)
        self._n_gt += 1
        if self._gt_t[0] is None:
            self._gt_t[0] = t
        self._gt_t[1] = t

    def on_cloud(self, _msg: PointCloud2):
        self._n_cloud += 1

    # ---------------------------------------------------------------- 收尾
    def close(self):
        if self._closed:
            return
        self._closed = True
        meta = {
            'algo': self.algo,
            'est_topic': self.est_topic,
            'gt_topic': self.gt_topic,
            'cloud_topic': self.cloud_topic,
            'out_csv': os.path.abspath(self.out_csv),
            'counts': {
                'est': self._n_est,
                'gt': self._n_gt,
                'cloud': self._n_cloud,
            },
            'stamp_range': {
                'est': self._est_t,
                'gt': self._gt_t,
            },
        }
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:                              # noqa: BLE001
            pass
        meta_path = os.path.splitext(self.out_csv)[0] + '.meta.json'
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        self.get_logger().info(
            f'recorder done: est={self._n_est} gt={self._n_gt} '
            f'cloud={self._n_cloud}  ->  {self.out_csv}'
        )
        if self._n_est == 0:
            self.get_logger().error(
                f'一条 {self.est_topic} 都没录到 —— 算法没起来，或话题名不对')


def main():
    try:
        rclpy.init()
    except Exception:
        # rcl 只会说 "Failed to parse global arguments"，不说是哪个参数。
        # 把 sys.argv 打出来，下次能直接定位。
        import sys
        print('[traj_recorder] rclpy.init() 失败，sys.argv =', sys.argv, flush=True)
        raise
    node = None
    try:
        node = TrajRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
