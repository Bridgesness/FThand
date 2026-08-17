#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
replay_sync 节点：同步回放「录好的灵巧手 CSV + 合成的机械臂摆动」。

用途：没有机械臂的真实录制数据时，用脚本化的正弦摆动驱动机械臂（模拟键盘输入），
同时回放录好的灵巧手轨迹，两者按同一时间轴同步。

用法（OpenARM 模拟 + 全新 hand_controller 跑着时）：
  ros2 run orcahand_teleop_ros2 replay_sync --ros-args \
    -p file:=~/fthand_ws/data/traj_2026xxxx.csv     # 手部 CSV（record_targets 格式）
    -p amp:=0.8 -p freq:=0.5                          # 机械臂摆动幅度/频率
  # -p loop:=true 循环
"""
import os
import sys
import time
import csv
import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

HAND = ['thumb_abd', 'thumb_mcp', 'thumb_pip', 'thumb_dip',
        'index_abd', 'index_mcp', 'index_pip',
        'middle_abd', 'middle_mcp', 'middle_pip',
        'ring_abd', 'ring_mcp', 'ring_pip',
        'pinky_abd', 'pinky_mcp', 'pinky_pip']
LEFT_ARM = [f'openarm_left_joint{i}' for i in range(1, 8)]
RIGHT_ARM = [f'openarm_right_joint{i}' for i in range(1, 8)]


def load_hand_csv(path):
    """record_targets 格式: 注释头 + timestamp_ms,j0..j15"""
    frames = []
    with open(path, newline='') as f:
        for row in csv.reader(f):
            if not row or not row[0] or not row[0][0].isdigit():
                continue
            t = int(row[0])
            hand = [float(x) for x in row[1:17]]
            frames.append((t, hand))
    return frames


class ReplaySyncNode(Node):
    def __init__(self):
        super().__init__('replay_sync')
        self.file = os.path.expanduser(self.declare_parameter('file', '').value)
        self.loop = self.declare_parameter('loop', False).value
        self.amp = float(self.declare_parameter('amp', 0.8).value)   # 臂摆动幅度(rad)
        self.freq = float(self.declare_parameter('freq', 0.5).value)  # 摆动频率(Hz)
        if not self.file or not os.path.exists(self.file):
            self.get_logger().error(f'CSV 不存在: {self.file}')
            raise SystemExit(1)
        self.frames = load_hand_csv(self.file)
        if not self.frames:
            self.get_logger().error('CSV 无数据')
            raise SystemExit(1)

        # ---- 安全限速：每帧最大关节变化（防抽搐/暴力动作，护硬件）----
        self.max_step = float(self.declare_parameter('max_step', 3.0).value)      # 手：度/帧
        self.arm_max_step = float(self.declare_parameter('arm_max_step', 0.05).value)  # 臂：弧度/帧
        self._prev_hand = None
        self._prev_left = None
        self._prev_right = None

        self.left_arm_pub = self.create_publisher(JointTrajectory, '/left_joint_trajectory_controller/joint_trajectory', 10)
        self.left_grip_pub = self.create_publisher(JointTrajectory, '/left_gripper_controller/joint_trajectory', 10)
        self.right_arm_pub = self.create_publisher(JointTrajectory, '/right_joint_trajectory_controller/joint_trajectory', 10)
        self.right_grip_pub = self.create_publisher(JointTrajectory, '/right_gripper_controller/joint_trajectory', 10)
        self.hand_pub = self.create_publisher(JointState, '/hand/joint_targets', 10)
        self.get_logger().info(f'replay_sync 加载 {len(self.frames)} 帧手数据, 臂摆动 amp={self.amp} freq={self.freq}Hz, loop={self.loop}')
        self.get_logger().info(f'安全限速: 手每帧≤{self.max_step}°, 臂每帧≤{self.arm_max_step}rad')

    def _clamp_step(self, prev, target, max_step):
        """相邻帧关节变化超过 max_step 就削峰，防暴力动作"""
        if prev is None:
            return list(target)
        return [p + max(-max_step, min(max_step, t - p)) for p, t in zip(prev, target)]

    def _arm_sweep(self, t_rel):
        """按相对时间合成机械臂正弦摆动（模拟键盘输入）"""
        left = [self.amp * math.sin(2 * math.pi * self.freq * t_rel + i * 0.5) for i in range(7)]
        right = [self.amp * math.sin(2 * math.pi * self.freq * t_rel + i * 0.5 + 1.0) for i in range(7)]
        return left, right

    def _traj(self, names, positions):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in positions]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 20_000_000
        msg.points = [pt]
        return msg

    def _publish_frame(self, t_rel, hand):
        left, right = self._arm_sweep(t_rel)
        # 限速：削峰到每帧最大变化，防抽搐
        hand = self._clamp_step(self._prev_hand, hand, self.max_step)
        left = self._clamp_step(self._prev_left, left, self.arm_max_step)
        right = self._clamp_step(self._prev_right, right, self.arm_max_step)
        self._prev_hand, self._prev_left, self._prev_right = list(hand), list(left), list(right)
        self.left_arm_pub.publish(self._traj(LEFT_ARM, left))
        self.left_grip_pub.publish(self._traj(['openarm_left_finger_joint1'], [0.044]))
        self.right_arm_pub.publish(self._traj(RIGHT_ARM, right))
        self.right_grip_pub.publish(self._traj(['openarm_right_finger_joint1'], [0.044]))
        js = JointState()
        js.name = list(HAND)
        js.position = [float(x) for x in hand]
        self.hand_pub.publish(js)

    def replay(self):
        self.get_logger().info('开始同步回放（臂摆动 + 手轨迹）...')
        t0 = self.frames[0][0]
        start = time.time()
        n = 0
        try:
            while True:
                for t, hand in self.frames:
                    target = start + (t - t0) / 1000.0
                    now = time.time()
                    if now < target:
                        time.sleep(target - now)
                    self._publish_frame((t - t0) / 1000.0, hand)
                    n += 1
                if not self.loop:
                    break
        except KeyboardInterrupt:
            pass
        self.get_logger().info(f'回放完成，共 {n} 帧')


def main():
    rclpy.init()
    node = ReplaySyncNode()
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
