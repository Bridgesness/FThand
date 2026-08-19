# ==============================================================================
# 对指位姿录制工具
# 戴手套摆出"拇指对某指尖"的姿势, 按键存入对应槽位, 最后写 opposition_poses.yaml。
# 供 teleoperation.py 的"对指吸附"模式使用。
#
# 用法:
#   python scripts/record_opposition.py --model orca_core/models/orcahand_v1_left
#
# 按键:
#   1-4  存当前平滑角度到 拇指对食指/中指/无名指/小指
#   p    打印已存槽位
#   w    合并写 opposition_poses.yaml(不覆盖已有槽位) 并退出
#   q    退出(不写)
#
# 流程: 戴好手套, 把拇指和食指对在一起(像 OK 手势), 等手稳定后按 1;
#       换中指对指按 2, 依次 3/4; 最后按 w 保存。
# ==============================================================================

import sys
import os
import time
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml

from orca_core import OrcaHand
from teleoperation import UDEGloveSDK, GloveToHandMapper, SmoothController

FINGERS = ['index', 'middle', 'ring', 'pinky']
FINGER_LABEL = {'index': '食指', 'middle': '中指', 'ring': '无名指', 'pinky': '小指'}


def calibrated_angles(hand, angles):
    """剔除未标定关节(肌腱断/未校准), 只保存能驱动的关节."""
    out = {}
    for j, a in angles.items():
        mid = hand.joint_to_motor_map.get(j)
        limits = hand.motor_limits_dict.get(mid) if mid is not None else None
        if limits is None or any(l is None for l in limits):
            continue
        out[j] = float(a)
    return out


def print_table(angles, roms):
    """紧凑打印当前 16 关节角度 + 占 ROM 百分比条."""
    lines = []
    for j, a in angles.items():
        rom = roms.get(j)
        if rom and rom[1] > rom[0]:
            pct = (a - rom[0]) / (rom[1] - rom[0]) * 100.0
            pct = max(0.0, min(100.0, pct))
            bar = '#' * int(round(pct / 10)) + '-' * (10 - int(round(pct / 10)))
            lines.append(f"  {j:<14}{a:>7.1f}  {bar}")
    print("\n" + "=" * 40)
    print("  当前平滑角度 (#=弯曲程度)")
    print("-" * 40)
    print("\n".join(lines))
    print("=" * 40)


def write_poses(slots, model_path):
    path = os.path.join(model_path, "opposition_poses.yaml")
    existing = {}
    if os.path.exists(path):
        with open(path, 'r', encoding='utf-8') as f:
            existing = yaml.safe_load(f) or {}
    existing.update({k: v for k, v in slots.items() if v})
    with open(path, 'w', encoding='utf-8') as f:
        yaml.dump(existing, f, allow_unicode=True, default_flow_style=False)
    print(f"\n[写] 已保存 {len(slots)} 组对指位姿 -> {path}")
    for f, pose in slots.items():
        if pose:
            print(f"     {f}({FINGER_LABEL[f]}): {len(pose)} 关节")


def main():
    parser = argparse.ArgumentParser(description="对指位姿录制工具")
    parser.add_argument("--model", default="orca_core/models/orcahand_v1_left",
                        help="手模型路径")
    parser.add_argument("--port", type=int, default=8000, help="手套 UDP 端口")
    args = parser.parse_args()

    hand = OrcaHand(model_path=args.model)
    status = hand.connect()
    if not status[0]:
        print(f"[错误] 手连接失败: {status[1]}")
        return
    hand.enable_torque()
    hand.set_control_mode('current_based_position')
    print(f"[连接] {status[1]}")

    glove = UDEGloveSDK(port=args.port)
    if not glove.initialize():
        print("[错误] 手套 SDK 初始化失败")
        return
    glove.start_listening()
    print("[手套] 等待数据...")
    for _ in range(50):
        time.sleep(0.1)
        if glove.is_connected:
            print("[手套] 已连接!")
            break
    else:
        print("[警告] 未检测到手套数据, 将使用默认值")

    side = hand.type or "left"
    mapper = GloveToHandMapper(hand, hand=side)
    smoother = SmoothController(smoothing_factor=0.5)

    state = {'current': {}, 'slots': {}, 'last_print': 0.0}
    stop = threading.Event()

    def cmd_thread():
        print("[录制] 按键: 1-4=存对指位姿  p=查看已存  w=保存并退出  q=退出不写")
        while not stop.is_set():
            try:
                cmd = input().strip().lower()
            except EOFError:
                break
            if not cmd:
                continue
            if cmd in ('1', '2', '3', '4'):
                f = FINGERS[int(cmd) - 1]
                if state['current']:
                    state['slots'][f] = dict(state['current'])
                    print(f"[存] {f}({FINGER_LABEL[f]}): {len(state['slots'][f])} 关节")
                else:
                    print("[存] 还没收到手套数据, 等一帧再按")
            elif cmd == 'p':
                filled = [f for f in FINGERS if state['slots'].get(f)]
                print(f"[已存] {filled if filled else '无'}")
            elif cmd == 'w':
                write_poses(state['slots'], hand.model_path)
                stop.set()
            elif cmd == 'q':
                stop.set()

    threading.Thread(target=cmd_thread, daemon=True).start()

    print("[录制] 开始... 把拇指和指尖对在一起, 稳定后按对应数字键")
    try:
        while not stop.is_set():
            data = glove.get_finger_data(side)
            target = mapper.map_glove_to_hand(data)
            smoothed = smoother.smooth(target)
            filtered = calibrated_angles(hand, smoothed)
            state['current'] = filtered
            hand.set_joint_pos(filtered, num_steps=1)

            if time.time() - state['last_print'] > 0.4:
                state['last_print'] = time.time()
                print_table(filtered, mapper.joint_roms)

            time.sleep(0.01)
    finally:
        stop.set()
        glove.end_listening()
        hand.disable_torque()
        hand.disconnect()
        print("\n[录制] 已停止")


if __name__ == "__main__":
    main()
