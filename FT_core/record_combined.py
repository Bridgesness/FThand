#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_combined 节点：合并录制 机械臂(OpenARM) + 灵巧手(FThand) 的动作轨迹。

订阅：
  /joint_states        机械臂实际关节角（OpenARM ros2_control）
  /hand/joint_targets  手的目标关节角（FThand 遥操）
按固定频率（默认30Hz）采样，合并成一行 CSV：
  timestamp_ms, 左臂j1..j7, 左夹爪, 右臂j1..j7, 右夹爪, 手j0..j15

用法（机械臂模拟 + 手链跑着时）：
  ros2 run orcahand_teleop_ros2 record_combined
  # 固定时长：-p duration:=10
  # Ctrl+C 停并保存到 ~/fthand_ws/data/traj_combined_<时间>.csv
"""
import os
import sys
import time
import csv

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

ARM_LEFT = [f'openarm_left_joint{i}' for i in range(1, 8)]
ARM_RIGHT = [f'openarm_right_joint{i}' for i in range(1, 8)]
ARM_GRIP = ['openarm_left_finger_joint1', 'openarm_right_finger_joint1']
HAND = ['thumb_abd', 'thumb_mcp', 'thumb_pip', 'thumb_dip',
        'index_abd', 'index_mcp', 'index_pip',
        'middle_abd', 'middle_mcp', 'middle_pip',
        'ring_abd', 'ring_mcp', 'ring_pip',
        'pinky_abd', 'pinky_mcp', 'pinky_pip']


class RecordCombinedNode(Node):
    def __init__(self):
        super().__init__('record_combined')
        # Jazzy 参数类型严格：rate/duration 用整数类型声明（CLI 传 duration:=8 是 int）
        self.rate = float(self.declare_parameter('rate', 30).value)
        self.duration = float(self.declare_parameter('duration', 0).value)  # 0=直到Ctrl+C
        self.outdir = os.path.expanduser(self.declare_parameter('outdir', '~/fthand_ws/data').value)

        self._arm = None
        self._hand = None
        self._saved = False
        self.rows = []
        self.headers = (['timestamp_ms'] + ARM_LEFT + [ARM_GRIP[0]] +
                        ARM_RIGHT + [ARM_GRIP[1]] +
                        [f'hand_j{i}' for i in range(16)])

        self.create_subscription(JointState, '/joint_states', self._arm_cb, 10)
        self.create_subscription(JointState, '/hand/joint_targets', self._hand_cb, 10)
        self.create_timer(1.0 / self.rate, self._sample)
        if self.duration > 0:
            self._timer = self.create_timer(self.duration, self._auto_stop)
        self.get_logger().info(f'record_combined 启动 rate={self.rate:.0f}Hz 录 机械臂+手 → {self.outdir}')
        self.get_logger().info('等待 /joint_states 和 /hand/joint_targets ...')

    def _arm_cb(self, msg: JointState):
        self._arm = dict(zip(msg.name, msg.position))

    def _hand_cb(self, msg: JointState):
        self._hand = dict(zip(msg.name, msg.position))

    def _sample(self):
        if self._arm is None or self._hand is None:
            return
        row = [int(time.time() * 1000)]
        for j in ARM_LEFT:
            row.append(self._arm.get(j, 0.0))
        row.append(self._arm.get(ARM_GRIP[0], 0.0))
        for j in ARM_RIGHT:
            row.append(self._arm.get(j, 0.0))
        row.append(self._arm.get(ARM_GRIP[1], 0.0))
        for j in HAND:
            row.append(self._hand.get(j, 0.0))
        self.rows.append(row)

    def _auto_stop(self):
        self._timer.cancel()
        self.save()
        rclpy.shutdown()

    def save(self):
        if self._saved:
            return
        self._saved = True
        if not self.rows:
            self.get_logger().info('没有录到数据（确认机械臂模拟 + 手链都在跑）')
            return
        os.makedirs(self.outdir, exist_ok=True)
        from datetime import datetime
        path = os.path.join(self.outdir, f"traj_combined_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# 合并录制: 机械臂(/joint_states) + 手(/hand/joint_targets)'])
            w.writerow(['# rate_hz: ' + str(int(self.rate))])
            w.writerow(['# rows: ' + str(len(self.rows))])
            w.writerow(self.headers)
            for r in self.rows:
                w.writerow(r)
        self.get_logger().info(f'已保存 {len(self.rows)} 帧 → {path}')
        print('--- 表头 ---')
        print(','.join(self.headers))


def main():
    rclpy.init()
    node = RecordCombinedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
