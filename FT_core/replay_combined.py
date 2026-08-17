#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
replay_combined 节点：合并回放 record_combined 录的轨迹（机械臂 + 灵巧手）。

读合并 CSV（record_combined 格式），同时：
  - 机械臂：发 JointTrajectory 给 OpenARM 的 4 个控制器（左右臂 + 左右夹爪）
  - 灵巧手：发 JointState 给 /hand/joint_targets（hand_controller 驱动真手）

用法（OpenARM 模拟 + 全新 hand_controller 跑着时）：
  ros2 run orcahand_teleop_ros2 replay_combined --ros-args \
    -p file:=~/fthand_ws/data/traj_combined_xxx.csv
  # -p loop:=true 循环
"""
import os
import sys
import time
import csv

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

LEFT_ARM = [f'openarm_left_joint{i}' for i in range(1, 8)]
RIGHT_ARM = [f'openarm_right_joint{i}' for i in range(1, 8)]
HAND = ['thumb_abd', 'thumb_mcp', 'thumb_pip', 'thumb_dip',
        'index_abd', 'index_mcp', 'index_pip',
        'middle_abd', 'middle_mcp', 'middle_pip',
        'ring_abd', 'ring_mcp', 'ring_pip',
        'pinky_abd', 'pinky_mcp', 'pinky_pip']


def load_combined(path):
    """返回 [(t_ms, left[8], right[8], hand[16])]"""
    frames = []
    with open(path, newline='') as f:
        for row in csv.reader(f):
            if not row or not row[0] or not row[0][0].isdigit():
                continue
            t = int(row[0])
            left = [float(x) for x in row[1:9]]
            right = [float(x) for x in row[9:17]]
            hand = [float(x) for x in row[17:33]]
            frames.append((t, left, right, hand))
    return frames


class ReplayCombinedNode(Node):
    def __init__(self):
        super().__init__('replay_combined')
        self.file = os.path.expanduser(self.declare_parameter('file', '').value)
        self.loop = self.declare_parameter('loop', False).value
        if not self.file or not os.path.exists(self.file):
            self.get_logger().error(f'CSV 不存在: {self.file}')
            raise SystemExit(1)
        self.frames = load_combined(self.file)
        if not self.frames:
            self.get_logger().error('CSV 无数据')
            raise SystemExit(1)

        # 安全限速：每帧最大关节变化
        self.max_step = float(self.declare_parameter('max_step', 3.0).value)      # 手：度/帧
        self.arm_max_step = float(self.declare_parameter('arm_max_step', 0.05).value)  # 臂：弧度/帧
        self._prev = None

        self.left_arm_pub = self.create_publisher(JointTrajectory, '/left_joint_trajectory_controller/joint_trajectory', 10)
        self.left_grip_pub = self.create_publisher(JointTrajectory, '/left_gripper_controller/joint_trajectory', 10)
        self.right_arm_pub = self.create_publisher(JointTrajectory, '/right_joint_trajectory_controller/joint_trajectory', 10)
        self.right_grip_pub = self.create_publisher(JointTrajectory, '/right_gripper_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(JointState, '/hand/joint_targets', 10)
        self.get_logger().info(f'replay_combined 加载 {len(self.frames)} 帧, loop={self.loop}')

    def _clamp(self, prev, target, max_step):
        if prev is None:
            return list(target)
        return [p + max(-max_step, min(max_step, t - p)) for p, t in zip(prev, target)]

    def _build_traj(self, names, positions):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 20_000_000  # 0.02s
        msg.points = [pt]
        return msg

    def _publish_frame(self, left, right, hand):
        # 安全限速：每帧最大关节变化，防抽搐
        if self._prev is not None:
            hand = self._clamp(self._prev[0], hand, self.max_step)
            left = self._clamp(self._prev[1], left, self.arm_max_step)
            right = self._clamp(self._prev[2], right, self.arm_max_step)
        self._prev = (list(hand), list(left), list(right))
        self.left_arm_pub.publish(self._build_traj(LEFT_ARM, left[:7]))
        self.left_grip_pub.publish(self._build_traj(['openarm_left_finger_joint1'], [left[7]]))
        self.right_arm_pub.publish(self._build_traj(RIGHT_ARM, right[:7]))
        self.right_grip_pub.publish(self._build_traj(['openarm_right_finger_joint1'], [right[7]]))
        js = JointState()
        js.name = list(HAND)
        js.position = [float(x) for x in hand]
        self.hand_pub.publish(js)

    def replay(self):
        self.get_logger().info('开始合并回放（机械臂+手）...')
        t0 = self.frames[0][0]
        start = time.time()
        n = 0
        try:
            while True:
                for t, left, right, hand in self.frames:
                    target = start + (t - t0) / 1000.0
                    now = time.time()
                    if now < target:
                        time.sleep(target - now)
                    self._publish_frame(left, right, hand)
                    n += 1
                if not self.loop:
                    break
        except KeyboardInterrupt:
            pass
        self.get_logger().info(f'回放完成，共 {n} 帧')


def main():
    rclpy.init()
    node = ReplayCombinedNode()
    try:
        node.replay()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
