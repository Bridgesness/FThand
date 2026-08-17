#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glove_driver 节点（真手套，替代 fake_glove）

UDP 8000 收真手套数据（UDEGloveSDK）→ 发 /glove/data（Float64MultiArray，90 float）
布局与 fake_glove 完全一致：左手45(15向量×xyz) + 右手45，retargeter 直接能用。

跑法：
  ros2 run orcahand_teleop_ros2 glove_driver
  （或加端口：--ros-args -p glove_port:=8000）
停止：Ctrl+C

前提：手套的 UDP 包要能到 WSL（mirrored 网络 + 实测能收到）。
"""
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

FT_CORE = os.environ.get('FT_CORE_PATH', os.path.expanduser('~/FThand/FThand/FT_core'))
if FT_CORE not in sys.path:
    sys.path.insert(0, FT_CORE)

from teleoperation import UDEGloveSDK  # noqa: E402  复用你现成的 UDP 收+解析


class GloveDriverNode(Node):
    def __init__(self):
        super().__init__('glove_driver')
        port = self.declare_parameter('glove_port', 8000).value
        # 发布频率默认 30Hz：hand_controller 每次 set_joint_pos 串口耗时 ~15ms，
        # 60Hz(16.6ms/帧) 几乎没余量会排队卡顿；30Hz 是顺滑的安全值（可 -p publish_rate:=40 调快）
        self.rate = self.declare_parameter('publish_rate', 30.0).value
        self.pub = self.create_publisher(Float64MultiArray, '/glove/data', 10)

        self.glove = UDEGloveSDK(port=port)
        if not self.glove.initialize():
            self.get_logger().error(f'手套 SDK 初始化失败（端口 {port} 被占？）')
            raise SystemExit(1)
        self.glove.start_listening()

        self.create_timer(1.0 / self.rate, self._publish)   # 30Hz 发最新手套数据
        self.get_logger().info(f'glove_driver 已启动，监听 UDP {port}，发 /glove/data @{self.rate:.0f}Hz')
        self.get_logger().info('提示：is_connected=False 说明手套 UDP 还没进来（查 mirrored 网络/手套目标IP）')

    def _publish(self):
        left = self.glove.get_finger_data('left')    # List[Vector3Float]×15
        right = self.glove.get_finger_data('right')
        data = []
        for v in left:
            data += [v.x, v.y, v.z]
        for v in right:
            data += [v.x, v.y, v.z]
        self.pub.publish(Float64MultiArray(data=data))


def main():
    rclpy.init()
    try:
        rclpy.spin(GloveDriverNode())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
