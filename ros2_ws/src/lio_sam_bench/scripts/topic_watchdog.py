#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
topic_watchdog.py —— 录制阶段的话题看门狗：没数据就早点死，别让大家干等 102 秒
===============================================================================

它解决的具体问题
----------------
`ros2 bag record` 遇到"话题一直没数据"时不会报错，它会安静地等到超时。
于是 run_bench.sh 要等满整个 `record_seconds`（102 s）才发现包里没有 .db3，
而且日志里只有一句 "Waiting for topics"，看不出到底是 Gazebo 没起来、
渲染没拿到 GL 上下文、还是 sensor_prep 没转发。

watchdog 在启动后盯着三件事：
  * 被录制的话题有没有**发布者**（/points /imu/data /gt/odom）
  * 它们**收到了多少条消息**（0 条 = 桥接/节点在，但上游没数据）
  * 上游的原始话题有没有发布者（/points_raw /imu_raw）—— 用来区分
    "Gazebo 没起来" 和 "sensor_prep 没转发"

到 give_up_s 秒（墙钟时间，不是仿真时间 —— 仿真时间在 Gazebo 起来前是冻住的）
还不满足就打印一张诊断表并以退出码 1 结束；launch 里配了 on_exit=Shutdown，
所以整条录制会立刻收尾，run_bench.sh 马上就能报错，不用等 102 秒。

参数
----
  required       要录的话题（默认三个）
  upstream       上游原始话题（只查发布者，用于归因）
  give_up_s      多少秒后放弃（默认 45）
  min_messages   至少收到几条才算"活着"（默认 3）
  report_every_s 每隔多久打一次进度（默认 10）
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry


# 话题 -> (消息类型, 说明)
WATCH = {
    '/points': (PointCloud2, '16 线点云（sensor_prep 输出，两条算法都吃它）'),
    '/imu/data': (Imu, 'IMU（sensor_prep 输出，LIO-SAM 用）'),
    '/gt/odom': (Odometry, '真值（gz OdometryPublisher -> 桥接；评测全靠它）'),
}

# 上游原始话题：只用来判断"卡在哪一段"
UPSTREAM = {
    '/points_raw': 'gz gpu_lidar -> 桥接（它没发布者 = Gazebo 的雷达没起）',
    '/imu_raw': 'gz Imu 系统插件 -> 桥接（它没发布者 = world 里缺 imu 插件）',
}


class TopicWatchdog(Node):
    def __init__(self):
        super().__init__('topic_watchdog')

        self.declare_parameter('required', list(WATCH.keys()))
        self.declare_parameter('upstream', list(UPSTREAM.keys()))
        self.declare_parameter('give_up_s', 45.0)
        self.declare_parameter('min_messages', 3)
        self.declare_parameter('report_every_s', 10.0)

        gp = self.get_parameter
        self.give_up = float(gp('give_up_s').value)
        self.min_msgs = int(gp('min_messages').value)
        self.report_every = float(gp('report_every_s').value)
        self.required = list(gp('required').value)
        self.upstream = list(gp('upstream').value)

        self.counts = {t: 0 for t in self.required}
        self.t0 = time.monotonic()      # 墙钟：仿真时间在 Gazebo 起来前是冻住的
        self._last_report = 0.0
        self._done = False

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic in self.required:
            msg_type = WATCH.get(topic, (None,))[0]
            if msg_type is None:
                continue
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_msg(t), qos)

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'topic_watchdog: 盯着 {self.required}，'
            f'{self.give_up:.0f} s 内没数据就退出（墙钟计时）')

    def _on_msg(self, topic):
        self.counts[topic] = self.counts.get(topic, 0) + 1

    # ------------------------------------------------------------------ 检查
    def _alive(self, topic):
        return (self.count_publishers(topic) > 0
                and self.counts.get(topic, 0) >= self.min_msgs)

    def _report(self, elapsed, give_up_now):
        lines = []
        for topic in self.required:
            pubs = self.count_publishers(topic)
            n = self.counts.get(topic, 0)
            mark = 'OK ' if (pubs > 0 and n >= self.min_msgs) else 'BAD'
            desc = WATCH.get(topic, ('', ''))[1] if topic in WATCH else ''
            lines.append(f'  [{mark}] {topic:12s} publishers={pubs:<2d} '
                         f'messages={n:<5d} {desc}')
        for topic in self.upstream:
            pubs = self.count_publishers(topic)
            mark = 'OK ' if pubs > 0 else 'BAD'
            lines.append(f'  [{mark}] {topic:12s} publishers={pubs:<2d} '
                         f'（上游）{UPSTREAM.get(topic, "")}')
        self.get_logger().info(
            f'topic_watchdog {elapsed:.0f}s/{self.give_up:.0f}s\n' + '\n'.join(lines))

        if give_up_now:
            bad = [t for t in self.required if not self._alive(t)]
            bad_up = [t for t in self.upstream if self.count_publishers(t) == 0]
            self.get_logger().error(
                '录制拿不到数据，提前收工，免得白等。缺：' + ', '.join(bad))
            # 给出"卡在哪一段"的判断
            if '/points_raw' in bad_up and '/gt/odom' in bad:
                self.get_logger().error(
                    '根因更像 **Gazebo 根本没起来/没渲染**：连 /points_raw 都没有发布者。'
                    '常见于没有 DISPLAY 的 SSH 会话（gpu_lidar 需要 GL 上下文）。'
                    '先单独跑 gazebo_lab1.launch.py 看 Gazebo 窗口能不能出来。')
            elif '/points_raw' in bad_up:
                self.get_logger().error(
                    '根因更像 **gz 传感器/插件**：/points_raw 没有发布者，'
                    '检查 world 里有没有 gz-sim-sensors-system 和 <render_engine>ogre2。')
            elif '/gt/odom' in bad:
                self.get_logger().error(
                    '根因更像 **真值插件或桥接**：/gt/odom 没有数据。'
                    '检查 model.sdf 的 OdometryPublisher 与 gz_bridge.yaml 里的 /gt/odom 条目。')
            else:
                self.get_logger().error(
                    '话题有发布者但没消息：多半是上游节点在跑却没收到数据'
                    '（例如 sensor_prep 收到了 /points_raw 但过滤后全丢）。看上面那张表。')

    def _tick(self):
        if self._done:
            return
        elapsed = time.monotonic() - self.t0

        if all(self._alive(t) for t in self.required):
            if not self._done:
                self._done = True
                self._report(elapsed, give_up_now=False)
                self.get_logger().info(
                    'topic_watchdog: 三路数据都到了，录制可以继续（本节点转入静默）')
                # 注意：成功时**不退出**。launch 侧的 on_exit 处理器只在
                # 退出码非 0 时才有意义，成功就退出会误杀整条录制流程。
            return

        if elapsed >= self.give_up:
            self._done = True
            self._report(elapsed, give_up_now=True)
            raise SystemExit(1)

        if elapsed - self._last_report >= self.report_every:
            self._last_report = elapsed
            self._report(elapsed, give_up_now=False)


def main():
    rclpy.init()
    node = TopicWatchdog()
    rc = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        rc = int(e.code or 0)
    except KeyboardInterrupt:
        rc = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
