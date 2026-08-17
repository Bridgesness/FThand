#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_controller 节点（链路最后一环 + 手部维护服务）

订阅 /hand/joint_targets → set_joint_pos(num_steps=1)  [teleop]

另外暴露 4 个维护服务（仅真手模式 mock:=false 时可用）：
  /hand_controller/neutral    归中性位
  /hand_controller/zero       归零位
  /hand_controller/calibrate  自动标定（耗时，后台执行）
  /hand_controller/tension    张紧（耗时，后台执行）
都用 std_srvs/Trigger：ros2 service call /hand_controller/neutral std_srvs/srv/Trigger

注意：calibrate/tension 期间别同时跑 teleop（先停 fake_glove/retargeter），否则指令打架。
"""
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

FT_CORE = os.environ.get('FT_CORE_PATH', os.path.expanduser('~/FThand/FThand/FT_core'))
if FT_CORE not in sys.path:
    sys.path.insert(0, FT_CORE)


class HandControllerNode(Node):
    def __init__(self):
        super().__init__('hand_controller')

        self.mock = self.declare_parameter('mock', True).value
        self.hand = None
        # 跳过不命令的关节（逗号分隔），如断线/未标定的 pinky_abd，避免电机乱动+刷警告。
        # 必须在 mock 判断外声明：_cb 无论 mock 与否都会读取它。
        self.declare_parameter('skip_joints', '')

        if not self.mock:
            from orca_core.core import OrcaHand  # noqa: E402
            model_path = os.path.expanduser(self.declare_parameter(
                'model_path', os.path.join(FT_CORE, 'orca_core/models/orcahand_v1_right')).value)
            # 归中/归零速度参数（可 ros2 param set 动态调；越小/越短越快）
            self.declare_parameter('neutral_steps', 50)        # 插值步数，原 teleop 用 50
            self.declare_parameter('neutral_step_size', 0.001) # 每步 sleep 秒
            self.hand = OrcaHand(model_path=model_path)
            ok, msg = self.hand.connect()
            if not ok:
                self.get_logger().error(f'连接失败: {msg}')
                raise SystemExit(1)
            self.hand.enable_torque()
            self.hand.set_control_mode('current_based_position')
            self.get_logger().info('真手已连接，扭矩已使能')

            # 4 个维护服务（仅真手模式）
            self.create_service(Trigger, '~/neutral', self._srv_neutral)
            self.create_service(Trigger, '~/zero', self._srv_zero)
            self.create_service(Trigger, '~/calibrate', self._srv_calibrate)
            self.create_service(Trigger, '~/tension', self._srv_tension)
            self.get_logger().info(
                '维护服务已开放: /hand_controller/{neutral,zero,calibrate,tension}')
        else:
            self.get_logger().info('Mock 模式：只打印目标角度，不碰硬件/无维护服务')

        self.create_subscription(JointState, '/hand/joint_targets', self._cb, 10)
        self.get_logger().info('hand_controller 已启动，等待 /hand/joint_targets ...')

    # ---- teleop 回调 ----
    def _cb(self, msg: JointState):
        angles = dict(zip(msg.name, msg.position))
        # 过滤掉要跳过的关节（断线/未标定），不再命令对应电机
        skip = [s.strip() for s in self.get_parameter('skip_joints').value.split(',') if s.strip()]
        if skip:
            angles = {j: v for j, v in angles.items() if j not in skip}
        if not self.mock:
            try:
                self.hand.set_joint_pos(angles, num_steps=1)   # ★绝不 >1(会阻塞)
            except Exception as e:
                self.get_logger().warn(f'set_joint_pos 失败: {e}', throttle_duration_sec=1.0)
                return
        idx = angles.get('index_mcp', 0.0)
        tag = '(mock仅显示)' if self.mock else '(已下发真手)'
        self.get_logger().info(f'收到目标 index_mcp={idx:.1f}° {tag}', throttle_duration_sec=2.0)

    # ---- 维护服务：在后台线程跑，回调立即返回，不卡 executor ----
    def _kick(self, fn, name):
        def task():
            try:
                self.get_logger().info(f'[{name}] 开始执行 ...')
                fn()
                self.get_logger().info(f'[{name}] 完成')
            except Exception as e:
                self.get_logger().error(f'[{name}] 失败: {e}')
        threading.Thread(target=task, daemon=True).start()

    def _srv_neutral(self, req, resp):
        n = self.get_parameter('neutral_steps').value
        s = self.get_parameter('neutral_step_size').value
        self._kick(lambda n=n, s=s: self.hand.set_neutral_position(num_steps=n, step_size=s), 'neutral')
        resp.success, resp.message = True, f'neutral 启动 steps={n} step_size={s}（越大/越慢越平滑）'
        return resp

    def _srv_zero(self, req, resp):
        n = self.get_parameter('neutral_steps').value
        s = self.get_parameter('neutral_step_size').value
        self._kick(lambda n=n, s=s: self.hand.set_zero_position(num_steps=n, step_size=s), 'zero')
        resp.success, resp.message = True, f'zero 启动 steps={n} step_size={s}'
        return resp

    def _srv_calibrate(self, req, resp):
        self.get_logger().warn('标定耗时较长，且期间请勿同时 teleop（先停 fake_glove/retargeter）')
        self._kick(lambda: self.hand.calibrate(), 'calibrate')
        resp.success, resp.message = True, 'calibrate 已在后台启动（按 config.yaml 的 calib_sequence 执行，耗时数分钟）'
        return resp

    def _srv_tension(self, req, resp):
        self.get_logger().warn('张紧耗时较长，且期间请勿同时 teleop')
        self._kick(lambda: self.hand.tension(), 'tension')
        resp.success, resp.message = True, 'tension 已在后台启动'
        return resp


def main():
    rclpy.init()
    try:
        rclpy.spin(HandControllerNode())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
