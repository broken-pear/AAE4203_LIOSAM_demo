#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patrol_driver.py —— 让小车自己绕圈跑，方便"零操作"观察 LIO-SAM 建图

它只是以固定频率往 /cmd_vel 发 geometry_msgs/Twist：

    linear.x  = linear_speed   (m/s)
    angular.z = angular_speed  (rad/s)

转弯半径 = linear_speed / angular_speed。默认 0.40 / 0.32 ≈ 1.25 m，
正好绕着 world 中间那根方柱转圈 —— 跑一两圈就能触发闭环。

参数：
  linear_speed  : 0.40
  angular_speed : 0.32
  rate_hz       : 20.0
  ramp_time     : 2.0   起步的斜坡时间（秒），避免一开始轮胎打滑
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PatrolDriver(Node):
    def __init__(self):
        super().__init__('patrol_driver')

        self.declare_parameter('linear_speed', 0.40)
        self.declare_parameter('angular_speed', 0.32)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('ramp_time', 2.0)

        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.ramp_time = max(float(self.get_parameter('ramp_time').value), 1e-3)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t0 = None
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'patrol_driver: v={self.v} m/s, w={self.w} rad/s, 转弯半径≈{self.v / self.w:.2f} m'
        )

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return                       # 仿真时钟还没起来
        if self.t0 is None:
            self.t0 = now

        k = min((now - self.t0) / self.ramp_time, 1.0)

        msg = Twist()
        msg.linear.x = self.v * k
        msg.angular.z = self.w * k
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PatrolDriver()
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
