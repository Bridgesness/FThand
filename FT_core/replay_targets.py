#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
replay_targets 节点：回放录制的关节角轨迹 CSV → /hand/joint_targets。

读 record_targets 生成的 CSV（解析 # joint_names 注释头），
按原采样率把每一帧 publish 到 /hand/joint_targets。
配合 hand_controller 运行即可让手重做动作。

用法（先停掉 retargeter / glove_driver，只留 hand_controller）：
  ros2 run orcahand_teleop_ros2 replay_targets --ros-args \
    -p file:=/home/ubuntu/fthand_ws/data/traj_xxx.csv
  # 可选：-p loop:=true 循环回放
"""
import os
import sys
import time
import csv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def load_csv(path):
    """解析 CSV，返回 (joint_names, [(timestamp_ms, angles)])。"""
    joint_names = []
    rows = []
    data_started = False
    with open(path, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            first = row[0]
            if first.startswith('# joint_names'):
                joint_names = [s.strip() for s in first.split(':', 1)[1].split(',')]
            elif first.startswith('#'):
                continue  # 其他注释行
            elif not data_started and first == 'timestamp_ms':
                data_started = True  # 表头
            elif data_started:
                try:
                    t = int(row[0])
                    ang = [float(x) for x in row[1:]]
                    rows.append((t, ang))
                except ValueError:
                    continue
    return joint_names, rows


class ReplayTargetsNode(Node):
    def __init__(self):
        super().__init__('replay_targets')
        # 展开 ~/ 家目录（launch 参数里的 ~ 不会自动展开）
        self.file = os.path.expanduser(self.declare_parameter('file', '').value)
        self.loop = self.declare_parameter('loop', False).value
        if not self.file or not os.path.exists(self.file):
            self.get_logger().error(f'CSV 文件不存在: {self.file}')
            raise SystemExit(1)
        self.joint_names, self.rows = load_csv(self.file)
        if not self.rows:
            self.get_logger().error('CSV 里没有数据行')
            raise SystemExit(1)
        # 安全限速：每帧最大关节变化（度/帧），防抽搐/暴力动作护硬件
        self.max_step = float(self.declare_parameter('max_step', 3.0).value)
        self._prev = None
        self.pub = self.create_publisher(JointState, '/hand/joint_targets', 10)
        self.get_logger().info(
            f'replay_targets 加载 {len(self.rows)} 帧, 关节={len(self.joint_names)}, loop={self.loop}, 限速≤{self.max_step}°/帧')

    def _publish(self, angles):
        if self._prev is not None:
            angles = [p + max(-self.max_step, min(self.max_step, t - p))
                      for p, t in zip(self._prev, angles)]
        self._prev = list(angles)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = list(angles)
        self.pub.publish(js)

    def replay(self):
        self.get_logger().info('开始回放...')
        t0 = self.rows[0][0]
        start = time.time()
        frame = 0
        try:
            while True:
                for ts, ang in self.rows:
                    target = start + (ts - t0) / 1000.0
                    now = time.time()
                    if now < target:
                        time.sleep(target - now)
                    self._publish(ang)
                    frame += 1
                if not self.loop:
                    break
        except KeyboardInterrupt:
            pass
        self.get_logger().info(f'回放完成，共 {frame} 帧')


def main():
    rclpy.init()
    node = ReplayTargetsNode()
    try:
        node.replay()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
