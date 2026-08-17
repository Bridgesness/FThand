#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_targets 节点：录制遥操时下发到手的关节角轨迹 → CSV（VLA 数采用）。

订阅 /hand/joint_targets（JointState），定时采样（默认 30Hz）写入 CSV：
  # joint_names: ...   (元信息注释行)
  # rate_hz: 30
  timestamp_ms, j0, j1, ..., j16
  <ms>, <angle0>, ...

用法：
  ros2 run orcahand_teleop_ros2 record_targets
  # 可选：-p rate:=30  -p outdir:=~/fthand_ws/data
  Ctrl+C 停止并保存（文件在 ~/fthand_ws/data/traj_<时间>.csv）
"""
import os
import sys
import time
import csv

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


class RecordTargetsNode(Node):
    def __init__(self):
        super().__init__('record_targets')
        # Jazzy 参数类型严格：rate/duration 用整数类型声明（CLI 传 duration:=8 是 int）
        self.rate = float(self.declare_parameter('rate', 30).value)
        self.duration = float(self.declare_parameter('duration', 0).value)  # 0 = 直到 Ctrl+C
        self.outdir = os.path.expanduser(self.declare_parameter('outdir', '~/fthand_ws/data').value)

        self.joint_names = []
        self.rows = []          # [(timestamp_ms, [angles])]
        self._last = None
        self._saved = False

        self.create_subscription(JointState, '/hand/joint_targets', self._cb, 10)
        self.create_timer(1.0 / self.rate, self._sample)
        if self.duration > 0:
            self._timer = self.create_timer(self.duration, self._auto_stop)
            self.get_logger().info(
                f'record_targets 已启动 rate={self.rate:.0f}Hz，duration={self.duration}s → {self.outdir}')
        else:
            self.get_logger().info(
                f'record_targets 已启动 rate={self.rate:.0f}Hz，录制 /hand/joint_targets → {self.outdir}')
            self.get_logger().info('Ctrl+C 停止并保存')

    def _cb(self, msg: JointState):
        self._last = msg

    def _sample(self):
        msg = self._last
        if msg is None:
            return
        if not self.joint_names:
            self.joint_names = list(msg.name)
        self.rows.append((int(time.time() * 1000), list(msg.position)))

    def _auto_stop(self):
        self._timer.cancel()
        self.get_logger().info(f'duration 到，自动保存')
        self.save()
        rclpy.shutdown()

    def save(self):
        if self._saved:
            return
        self._saved = True
        if not self.rows:
            self.get_logger().info('没有录到数据（确认遥操链在跑、/hand/joint_targets 有数据）')
            return
        os.makedirs(self.outdir, exist_ok=True)
        from datetime import datetime
        path = os.path.join(self.outdir, f"traj_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# joint_names: ' + ','.join(self.joint_names)])
            w.writerow(['# rate_hz: ' + str(int(self.rate))])
            w.writerow(['# rows: ' + str(len(self.rows))])
            w.writerow(['timestamp_ms'] + [f'j{i}' for i in range(len(self.joint_names))])
            for t, ang in self.rows:
                w.writerow([t] + ang)
        self.get_logger().info(f'已保存 {len(self.rows)} 帧 → {path}')
        # 打印前 3 行预览
        with open(path) as f:
            preview = ''.join(f.readlines()[:5])
        print('--- 文件头预览 ---')
        print(preview)

    def destroy_node(self):
        # rclpy spin 退出（Ctrl+C）时 save 已在 main 里调用；这里兜底
        super().destroy_node()


def main():
    rclpy.init()
    node = RecordTargetsNode()
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
