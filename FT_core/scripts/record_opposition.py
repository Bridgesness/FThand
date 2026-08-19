# ==============================================================================
# 对指位姿录制工具（手动掰手版）
# 断开扭矩, 手动把灵巧手的拇指和某指尖掰成对指姿势, 按对应数字键记录。
# 记录的是手部**实际关节角度**(get_joint_pos), 不是手套映射值——
# 因为手套映射够不到真正的对指位姿。
# 供 teleoperation.py 的"对指吸附"模式使用。
#
# 用法:
#   python scripts/record_opposition.py --model orca_core/models/orcahand_v1_left
#
# 按键:
#   1-4  把当前手部实际关节角度存到 拇指对食指/中指/无名指/小指
#   p    打印已存槽位
#   w    合并写 opposition_poses.yaml(不覆盖已有槽位) 并退出
#   q    退出(不写)
#
# 流程: 启动后扭矩自动断开, 手可自由掰动。
#       手动把拇指和食指对在一起(像 OK 手势), 稳定后按 1;
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

FINGERS = ['index', 'middle', 'ring', 'pinky']
FINGER_LABEL = {'index': '食指', 'middle': '中指', 'ring': '无名指', 'pinky': '小指'}


def read_current_angles(hand):
    """读手部实际关节角度(度), 剔除未标定(None)关节."""
    positions = hand.get_joint_pos(as_list=False) or {}
    return {j: float(v) for j, v in positions.items() if v is not None}


def print_table(angles, roms):
    """紧凑打印当前手部角度 + 占 ROM 百分比条."""
    lines = []
    for j, a in angles.items():
        rom = roms.get(j)
        if rom and rom[1] > rom[0]:
            pct = max(0.0, min(100.0, (a - rom[0]) / (rom[1] - rom[0]) * 100.0))
            bar = '#' * int(round(pct / 10)) + '-' * (10 - int(round(pct / 10)))
            lines.append(f"  {j:<14}{a:>7.1f}  {bar}")
    print("\n" + "=" * 40)
    print("  手部实际角度 (#=弯曲程度)")
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
    parser = argparse.ArgumentParser(description="对指位姿录制工具(手动掰手)")
    parser.add_argument("--model", default="orca_core/models/orcahand_v1_left",
                        help="手模型路径")
    args = parser.parse_args()

    hand = OrcaHand(model_path=args.model)
    status = hand.connect()
    if not status[0]:
        print(f"[错误] 手连接失败: {status[1]}")
        return

    # 断开扭矩: 手松软, 可手动掰成对指姿势
    hand.disable_torque()
    print(f"[连接] {status[1]}")
    print("[提示] 扭矩已断开, 手可自由掰动。")
    print("[提示] 手动把拇指和某指尖对在一起, 稳定后按 1/2/3/4 记录, 最后按 w 保存。")

    state = {'current': {}, 'slots': {}, 'last_print': 0.0}
    stop = threading.Event()

    def cmd_thread():
        print("[录制] 按键: 1-4=存当前手部角度到对指槽位  p=查看已存  w=保存并退出  q=退出不写")
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
                    print("[存] 还没读到手部位置, 等一帧再按")
            elif cmd == 'p':
                filled = [f for f in FINGERS if state['slots'].get(f)]
                print(f"[已存] {filled if filled else '无'}")
            elif cmd == 'w':
                write_poses(state['slots'], hand.model_path)
                stop.set()
            elif cmd == 'q':
                stop.set()

    threading.Thread(target=cmd_thread, daemon=True).start()

    try:
        while not stop.is_set():
            state['current'] = read_current_angles(hand)
            if time.time() - state['last_print'] > 0.4:
                state['last_print'] = time.time()
                print_table(state['current'], hand.joint_roms_dict)
            time.sleep(0.02)
    finally:
        stop.set()
        hand.enable_torque()   # 恢复扭矩, 别让手一直软着
        hand.disconnect()
        print("\n[录制] 已停止(扭矩已恢复)")


if __name__ == "__main__":
    main()
