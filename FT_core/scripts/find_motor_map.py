import argparse
import time
from orca_core import OrcaHand

# 逐个电机小幅往复运动,用来人工核对"每个 motor_id 物理上接的是哪个关节"。
# 用法(在 FT_core 目录下):
#   python scripts\find_motor_map.py orca_core\models\orcahand_v1_left

def main():
    parser = argparse.ArgumentParser(
        description="Find which physical joint each motor ID actually drives."
    )
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the orcahand model folder")
    parser.add_argument("--move", type=float, default=0.5,
                        help="Relative move size in rad (default 0.5)")
    parser.add_argument("--hold", type=float, default=1.5,
                        help="Seconds to hold at moved position (default 1.5)")
    parser.add_argument("--current", type=int, default=150,
                        help="Max current during the test (default 150, gentle)")
    parser.add_argument("--port", default=None,
                        help="串口覆盖(如 COM7)，不指定则用 config.yaml 的 port")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path, port=args.port)
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.enable_torque()
    hand.set_control_mode("current_based_position")
    hand.set_max_current(args.current)

    input("\n即将【逐个电机】小幅往复运动。请确保手部前方无阻挡、手指别被卡住。"
          "\n按回车开始(随时可 Ctrl+C 中止)...\n")

    results = {}
    try:
        for motor_id in hand.motor_ids:
            start = hand.get_motor_pos(as_dict=True)[motor_id]
            print(f"\n>>> 正在驱动 MOTOR ID {motor_id} ... 盯紧看哪个手指/哪一节在动")

            hand._set_motor_pos({motor_id: start + args.move})
            time.sleep(args.hold)
            hand._set_motor_pos({motor_id: start - args.move})
            time.sleep(args.hold)
            hand._set_motor_pos({motor_id: start})  # 回到起点
            time.sleep(0.5)

            ans = input(f"    motor {motor_id} 实际动的是哪个关节?"
                        f"(例如 thumb_mcp / index_pip / wrist,直接回车跳过): ").strip()
            results[motor_id] = ans or "(skipped)"
            print(f"    -> 已记录: motor {motor_id} = {results[motor_id]}")

    except KeyboardInterrupt:
        print("\n已中止。")
    finally:
        try:
            hand.disable_torque()
            hand.disconnect()
        except Exception:
            pass

    print("\n========== 汇总(把右侧填回 config.yaml 的 joint_to_motor_map)==========")
    for motor_id, joint in results.items():
        print(f"  motor {motor_id:>2}  ->  {joint}")


if __name__ == "__main__":
    main()
