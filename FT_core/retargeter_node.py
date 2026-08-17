#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
retargeter 节点（第二步，可插拔的中间节点）

订阅 /glove/data（Float64MultiArray，90个float）
  → 用 teleoperation.py 里现成的 GloveToHandMapper + SmoothController 算出 17 个关节角度
  → 发布 /hand/joint_targets（sensor_msgs/JointState）

跑法（假手套在另一个终端开着）：
  cd ~/FThand/FThand/FT_core
  python3 retargeter_node.py
停止：Ctrl+C

注：这里直接 import teleoperation.py 里现成的类（快速联调）。
   后续建正式 ROS2 包时，会把这几个类抽到共享模块 teleop_lib.py。
"""
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# —— 把 FT_core 加进路径，才能 import teleoperation 和 orca_core ——
FT_CORE = os.environ.get('FT_CORE_PATH', os.path.expanduser('~/FThand/FThand/FT_core'))
if FT_CORE not in sys.path:
    sys.path.insert(0, FT_CORE)

from teleoperation import GloveToHandMapper, SmoothController, Vector3Float  # noqa: E402
from orca_core.core import OrcaHand                                          # noqa: E402


class RetargeterNode(Node):
    def __init__(self):
        super().__init__('retargeter')

        # 参数（都能 ros2 param set 动态改，对应原 stdin 的 s/m 命令）
        hand = self.declare_parameter('hand', 'right').value
        model_path = os.path.expanduser(self.declare_parameter(
            'model_path', os.path.join(FT_CORE, 'orca_core/models/orcahand_v1_right')).value)
        motion_scale = self.declare_parameter('motion_scale', 0.8).value
        smoothing = self.declare_parameter('smoothing', 0.3).value

        # OrcaHand 只为加载 joint_roms/neutral/model_path（不 connect，不碰硬件）
        ref = OrcaHand(model_path=model_path)
        self.mapper = GloveToHandMapper(ref, hand=hand)
        self.mapper.motion_scale = motion_scale
        self.smoother = SmoothController(smoothing)

        self.create_subscription(Float64MultiArray, '/glove/data', self._cb, 10)
        self.pub = self.create_publisher(JointState, '/hand/joint_targets', 10)

        # 启动自诊断：确认手套映射有没有加载
        self.hand = hand
        n = len(self.mapper.glove_mapping)
        idx_cfg = self.mapper.glove_mapping.get('index_mcp', {})
        self.get_logger().info(
            f'retargeter 已启动 hand={hand} scale={motion_scale} smooth={smoothing}')
        self.get_logger().info(
            f'  映射关节数={n}  index_mcp配置={"✓" if idx_cfg else "❌缺失(映射没加载!)"}  {idx_cfg}')

    def _cb(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < 90:
            return

        # 把 90 个 float reshape 回 List[Vector3Float]×2
        def to_vecs(offset):
            return [Vector3Float(d[offset + i * 3], d[offset + i * 3 + 1], d[offset + i * 3 + 2])
                    for i in range(15)]
        left = to_vecs(0)
        right = to_vecs(45)
        glove_data = right if self.mapper.hand == 'right' else left

        # 复用现成映射 + 平滑
        target = self.mapper.map_glove_to_hand(glove_data)
        smooth = self.smoother.smooth(target)

        # 自诊断：index_mcp 的 raw→norm→angle，一眼看出映射有没有生效
        diag = self.mapper.last_diagnostics.get('index_mcp', {})
        self.get_logger().info(
            f'[diag] index_mcp  raw={diag.get("raw", 0):6.1f}  '
            f'norm={diag.get("normalized", 0):.2f}  '
            f'angle={diag.get("angle", 0):5.1f}°  ->  publish={smooth.get("index_mcp", 0):5.1f}°',
            throttle_duration_sec=1.0)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(smooth.keys())
        js.position = list(smooth.values())
        self.pub.publish(js)


def main():
    rclpy.init()
    try:
        rclpy.spin(RetargeterNode())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
