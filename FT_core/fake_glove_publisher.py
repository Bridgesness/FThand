#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假手套数据发布器（开发联调用，不是真节点的一部分）

用途：没有真手套时，用它定时发假的 /glove/data，验证整条 ROS2 链路通不通。
跑法：python3 fake_glove_publisher.py   （雷达/手套都不用接）
停止：Ctrl+C

数据布局（与改造方案的 /glove/data 一致，共 90 个 float）：
  data[0..44]   = 左手 15 个向量，每个(x,y,z) 摊平（这里全置 0）
  data[45..89]  = 右手 15 个向量
模拟右手食指来回弯曲，方便后续 retargeter 接上后能看到食指动。
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class FakeGloveNode(Node):
    def __init__(self):
        super().__init__('fake_glove')
        self.pub = self.create_publisher(Float64MultiArray, '/glove/data', 10)
        self.timer = self.create_timer(1.0 / 60.0, self._publish)   # 60Hz
        self.t0 = time.time()
        self.get_logger().info('假手套已启动，发 /glove/data @60Hz（模拟右手食指弯曲）。Ctrl+C 停。')

    def _publish(self):
        t = time.time() - self.t0
        # 食指弯曲量 0 → -100 来回（负=向内弯，和 glove_mapping.yaml 的 range 一致）
        flex = -50.0 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.4 * t))   # 0~-50，0.4Hz

        data = [0.0] * 90
        # 左右手食指都动（左手 offset 0，右手 offset 45），这样 retargeter 选哪只都有动作
        for offset in (0, 45):
            data[offset + 3 * 3 + 0] = flex   # Index1.x (食指 MCP)
            data[offset + 4 * 3 + 0] = flex   # Index2.x (食指 PIP)
            data[offset + 5 * 3 + 0] = flex   # Index3.x (食指 DIP)

        msg = Float64MultiArray(data=data)
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(FakeGloveNode())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
