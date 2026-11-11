#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripted_driver.py —— 按"脚本"开车：一串 (v, ω, 持续时间) 段依次执行
===============================================================================
patrol_driver.py 只会匀速绕圈，够 Demo 用；但对比实验需要不同的运动模式来暴露
两条算法的差异（急转 / 高速直行 / 走廊往返）。所以这里给一个可编排的驱动器：

    linear_speed  : [0.4, 0.0, 0.4, ...]     m/s   （每段一个）
    angular_speed : [0.0, 1.2, 0.0, ...]     rad/s
    duration      : [12.0, 2.0, 12.0, ...]   s

三个数组长度必须一致，第 i 段执行 duration[i] 秒，之后就切下一段。
全部段跑完后发 0 速度（车停住），保持不动直到录制结束。

为什么用"分段常量"而不是闭环控制
--------------------------------
对比实验要的是**可复现**，不是"跑得漂亮"。
分段常量速度 → 同一份 bag 可以喂给两条算法 → 两条算法看到的运动完全相同。
开环 + 同一仿真器，重复录制之间的差异只有物理引擎的微小数值差别。

参数
----
  linear_speed   必需，double 数组
  angular_speed  必需，double 数组
  duration       必需，double 数组
  rate_hz        20.0   发布频率
  ramp_time      1.0    第一段的软启动时间（s），避免起步轮胎打滑
  hold_final     true   跑完后继续发 0 速度（而不是退出节点）

用法（配合 experiments/scenarios/*.yaml）：
  ros2 run lio_sam_gz_bringup scripted_driver.py \
       --ros-args --params-file experiments/scenarios/s1_fast_yaw.yaml \
                 -p use_sim_time:=true
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ScriptedDriver(Node):
    def __init__(self):
        super().__init__('scripted_driver')

        self.declare_parameter('linear_speed', [0.4])
        self.declare_parameter('angular_speed', [0.0])
        self.declare_parameter('duration', [10.0])
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('ramp_time', 1.0)
        self.declare_parameter('hold_final', True)

        gp = self.get_parameter
        self.v = [float(x) for x in gp('linear_speed').value]
        self.w = [float(x) for x in gp('angular_speed').value]
        self.d = [float(x) for x in gp('duration').value]
        rate = float(gp('rate_hz').value)
        self.ramp = max(float(gp('ramp_time').value), 1e-3)
        self.hold_final = bool(gp('hold_final').value)

        if not (len(self.v) == len(self.w) == len(self.d)):
            raise RuntimeError(
                f'linear_speed({len(self.v)}) / angular_speed({len(self.w)}) / '
                f'duration({len(self.d)}) 长度必须一致')

        # 每段的结束时刻（相对 t0）
        self.edges = []
        acc = 0.0
        for t in self.d:
            acc += t
            self.edges.append(acc)
        self.total = acc

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t0 = None
        self.seg = -1
        self.done = False
        self.timer = self.create_timer(1.0 / rate, self.tick)

        plan = ', '.join(f'{v:+.2f}m/s {w:+.2f}rad/s {t:.1f}s'
                         for v, w, t in zip(self.v, self.w, self.d))
        self.get_logger().info(f'scripted_driver: {len(self.d)} 段，共 '
                               f'{self.total:.1f} s  [{plan}]')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return                        # 仿真时钟还没起来
        if self.t0 is None:
            self.t0 = now

        t = now - self.t0

        if t >= self.total:
            if not self.done:
                self.done = True
                self.get_logger().info(f'脚本跑完（{self.total:.1f} s），停车')
            if not self.hold_final:
                return
            self._publish(0.0, 0.0)
            return

        seg = 0
        while seg < len(self.edges) - 1 and t >= self.edges[seg]:
            seg += 1
        if seg != self.seg:
            self.seg = seg
            self.get_logger().info(
                f'段 {seg + 1}/{len(self.d)}: v={self.v[seg]:+.2f} m/s, '
                f'w={self.w[seg]:+.2f} rad/s, 持续 {self.d[seg]:.1f} s')

        # 只在第一段做软启动
        k = 1.0
        if seg == 0:
            k = min(t / self.ramp, 1.0)
        self._publish(self.v[seg] * k, self.w[seg] * k)

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScriptedDriver()
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
