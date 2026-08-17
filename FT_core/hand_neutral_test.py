#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
真手通信 + 中性位测试（不依赖 ROS2，单独验证手能不能动）

做三件事：
  1. 连手，打印端口/校准状态
  2. 读当前关节位置（验证通信通不通）
  3. 缓慢移到 neutral_position（num_steps 大 = 慢 = 安全）

跑法：
  cd ~/FThand/FThand/FT_core
  python3 hand_neutral_test.py
随时 Ctrl+C 中断。

注意：如果 is_calibrated=False，中性位可能不准（关节→电机换算缺校准数据），
      那就要先跑 scripts/calibrate.py 校准。
"""
import os
import sys
import time

FT_CORE = os.path.expanduser('~/FThand/FThand/FT_core')
if FT_CORE not in sys.path:
    sys.path.insert(0, FT_CORE)

from orca_core.core import OrcaHand  # noqa: E402

# 第一参数 left/right，默认 left（你现在接的是左手）
hand_type = sys.argv[1] if len(sys.argv) > 1 else 'left'
MODEL = os.path.join(FT_CORE, f'orca_core/models/orcahand_v1_{hand_type}')


def main():
    hand = OrcaHand(model_path=MODEL)
    print(f'端口: {hand.port}   波特率: {hand.baudrate}   client: {hand.client_type}')
    print(f'已校准? {hand.is_calibrated(verbose=True)}')

    ok, msg = hand.connect()
    print(f'connect: {ok} {msg}')
    if not ok:
        print('连不上，检查 /dev/ttyACM0 和权限')
        return

    hand.enable_torque()
    hand.set_control_mode('current_based_position')

    try:
        cur = hand.get_joint_pos(as_list=False)
        print('当前关节位置(度):')
        for j, v in cur.items():
            print(f'  {j:12s}: {v:8.2f}')
    except Exception as e:
        print(f'读位置失败: {e}（通信可能没通——CDC ACM 下 1000000 波特率的问题？）')

    print('\n缓慢移到中性位（约2秒）... 如果手要撞/拧，立刻 Ctrl+C！')
    time.sleep(1)
    try:
        hand.set_neutral_position(num_steps=100, step_size=0.02)
    except Exception as e:
        print(f'移中性位失败: {e}')
    time.sleep(0.5)

    try:
        now = hand.get_joint_pos(as_list=False)
        print('到位后位置(度):')
        for j, v in now.items():
            print(f'  {j:12s}: {v:8.2f}')
    except Exception as e:
        print(f'读位置失败: {e}')

    print('\n保持 3 秒...')
    time.sleep(3)
    hand.disable_torque()
    hand.disconnect()
    print('完成，已断开。')


if __name__ == '__main__':
    main()
