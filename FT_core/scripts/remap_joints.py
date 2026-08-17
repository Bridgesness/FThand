import argparse
import shutil
import time
from orca_core import OrcaHand
from orca_core.utils.utils import read_yaml

# 软件侧调整:逐个电机动一下,人工核对它物理上接的是哪个关节,
# 然后把正确的映射写回 config.yaml 的 joint_to_motor_map 段(保留注释)。
#
# 用法(在 FT_core 目录下):
#   python scripts\remap_joints.py orca_core\models\orcahand_v1_left

FLEX_POSITIVE = {"f", "flex"}      # 正向移动时关节弯折/分开
EXTEND_POSITIVE = {"e", "extend"}  # 正向移动时关节伸直/靠拢
UNSURE = {"u", "unsure", ""}


def build_map_block(joint_ids, new_map):
    """Build the text lines for the joint_to_motor_map block (with trailing blank line)."""
    lines = ["joint_to_motor_map:\n"]
    for joint in joint_ids:
        lines.append(f"  {joint}: {new_map[joint]}\n")
    lines.append("\n")  # keep the blank line before the next top-level key
    return lines


def replace_map_in_config(config_path, joint_ids, new_map):
    """Replace only the joint_to_motor_map block, preserving every other line/comment."""
    with open(config_path, encoding="utf-8") as f:
        lines = f.readlines()

    start = next(i for i, l in enumerate(lines) if l.strip() == "joint_to_motor_map:")
    end = start + 1
    while end < len(lines):
        l = lines[end]
        # next top-level key: non-empty, no leading whitespace, ends with ':'
        if l.strip() and not l.startswith((" ", "\t")) and l.rstrip().endswith(":"):
            break
        end += 1

    new_lines = lines[:start] + build_map_block(joint_ids, new_map) + lines[end:]
    with open(config_path, "w", encoding="utf-8") as f:
        f.writelines(new_lines)


def main():
    parser = argparse.ArgumentParser(
        description="Discover the true motor->joint wiring and write it back to config.yaml."
    )
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the orcahand model folder")
    parser.add_argument("--move", type=float, default=0.4,
                        help="Relative move size in rad (default 0.4)")
    parser.add_argument("--hold", type=float, default=1.2,
                        help="Seconds to hold at each position (default 1.2)")
    parser.add_argument("--current", type=int, default=150,
                        help="Max current during the test (default 150)")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    config_path = hand.config_path
    joint_ids = hand.joint_ids

    # current signed map -> default sign per joint name (used when user is unsure)
    cfg = read_yaml(config_path)
    cur_map = cfg.get("joint_to_motor_map", {})
    default_sign = {j: (-1 if int(v) < 0 else 1) for j, v in cur_map.items()}

    status = hand.connect()
    print(status)
    if not status[0]:
        print("连接失败。")
        return

    hand.enable_torque()
    hand.set_control_mode("current_based_position")
    hand.set_max_current(args.current)

    print("\n可关节名列表:", ", ".join(joint_ids))
    input("\n即将【逐个电机】小幅往复运动,请确保手指前方无阻挡。按回车开始(随时 Ctrl+C 中止)...\n")

    # motor_id -> (joint_name, direction_on_positive)
    discovered = {}
    try:
        for motor_id in hand.motor_ids:
            start = hand.get_motor_pos(as_dict=True)[motor_id]
            print(f"\n>>> 驱动 MOTOR ID {motor_id} ... 盯紧看哪个手指 / 哪一节在动")

            hand._set_motor_pos({motor_id: start + args.move})   # 正向
            time.sleep(args.hold)
            hand._set_motor_pos({motor_id: start - args.move})   # 反向
            time.sleep(args.hold)
            hand._set_motor_pos({motor_id: start})               # 回原位
            time.sleep(0.4)

            # 1) 哪个关节
            while True:
                joint = input(f"    motor {motor_id} 实际带的是哪个关节? ").strip()
                if joint in joint_ids:
                    break
                print(f"    无效。请从列表里选: {', '.join(joint_ids)}")

            # 2) 方向:正向(第一次)移动时关节怎么动
            d = input("    第一次(正向)移动时,该关节是 弯折/分开 [f] 还是 伸直/靠拢 [e]?(不确定按 u): ").strip().lower()
            if d in FLEX_POSITIVE:
                direction = "flex"
            elif d in EXTEND_POSITIVE:
                direction = "extend"
            else:
                direction = "unsure"

            discovered[motor_id] = (joint, direction)
            print(f"    -> 已记录: motor {motor_id} = {joint} ({direction})")

    except KeyboardInterrupt:
        print("\n已中止,不写配置。")
        return
    finally:
        try:
            hand.disable_torque()
            hand.disconnect()
        except Exception:
            pass

    # ---- 校验:每个关节必须恰好出现一次 ----
    assigned = [j for j, _ in discovered.values()]
    missing = [j for j in joint_ids if j not in assigned]
    dupes = sorted({j for j in assigned if assigned.count(j) > 1})
    if missing or dupes:
        print("\n映射不完整,未写入配置:")
        if missing:
            print("  缺少关节:", ", ".join(missing))
        if dupes:
            print("  重复关节:", ", ".join(dupes))
        print("  请重新运行本脚本,把每个关节都指定一次。")
        return

    # ---- 组装新的 joint_to_motor_map ----
    new_map = {}
    for joint in joint_ids:
        motor_id = next(m for m, (j, _) in discovered.items() if j == joint)
        _, direction = discovered[motor_id]
        if direction == "flex":
            sign = 1
        elif direction == "extend":
            sign = -1
        else:
            sign = default_sign.get(joint, 1)
        new_map[joint] = sign * motor_id

    print("\n========== 新的 joint_to_motor_map ==========")
    for joint in joint_ids:
        print(f"  {joint}: {new_map[joint]}")

    ans = input("\n确认写入 config.yaml?(会先备份为 config.yaml.bak) [y/N]: ").strip().lower()
    if ans not in ("y", "yes"):
        print("已取消,未修改。")
        return

    shutil.copy(config_path, config_path + ".bak")
    replace_map_in_config(config_path, joint_ids, new_map)
    print(f"已备份 -> {config_path}.bak")
    print(f"已写入 -> {config_path}")

    print("\n下一步:")
    print("  1) 重新运行校准:  python scripts\\calibrate.py orca_core\\models\\orcahand_v1_left")
    print("     (校准会清掉旧的限位并按新映射重新标定)")
    print("  2) 校准时盯一下每个关节的 flex 步骤:如果某关节该弯却伸了,"
          "把 config 里该关节的电机 ID 加/去负号,再重跑校准。")


if __name__ == "__main__":
    main()
