#!/usr/bin/env python3
"""
手动姿态校准脚本 (适配飞特舵机)。

自动校准 calibrate.py 在飞特位置伺服模式下不可靠(电流限制不生效, 极限误判)。
本脚本改用「整体姿态」: 你把整只手物理掰到几个姿态, 脚本记录电机位置并换算。

姿态:
  1) OPEN   全张开伸直  -> 弯曲关节的 extend 极限 + abd 关节的 flex(分开)极限
  2) FIST   全握紧      -> 弯曲关节的 flex 极限   + abd 关节的 extend(并拢)极限

注意: 手腕(wrist)不在此手动校准范围内 —— 它是高减速比舵机(ratio≈0.18,
远大于手指的~0.015), 手动反驱动会损坏齿轮, 且型号不同(4106)。手腕保留
calibration.yaml 里的现有数据, 后续若控制异常再用专门驱动方式单独处理。

用法:
  python scripts/manual_calibrate.py orca_core/models/orcahand_v1_right
"""

import argparse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand
from orca_core.utils.utils import update_yaml

# 每个关节的 flex / extend 极限分别在哪个姿态下达到
# 弯曲关节: 伸直=extend(张开), 弯曲=flex(握紧)
# abd关节:  分开=flex(张开), 并拢=extend(握紧)
JOINT_POSE_MAP = {
    # 拇指
    "thumb_mcp": {"flex": "FIST", "extend": "OPEN"},
    "thumb_abd": {"flex": "OPEN", "extend": "FIST"},
    "thumb_pip": {"flex": "FIST", "extend": "OPEN"},
    "thumb_dip": {"flex": "FIST", "extend": "OPEN"},
    # 食指
    "index_abd": {"flex": "OPEN", "extend": "FIST"},
    "index_mcp": {"flex": "FIST", "extend": "OPEN"},
    "index_pip": {"flex": "FIST", "extend": "OPEN"},
    # 中指
    "middle_abd": {"flex": "OPEN", "extend": "FIST"},
    "middle_mcp": {"flex": "FIST", "extend": "OPEN"},
    "middle_pip": {"flex": "FIST", "extend": "OPEN"},
    # 无名指
    "ring_abd": {"flex": "OPEN", "extend": "FIST"},
    "ring_mcp": {"flex": "FIST", "extend": "OPEN"},
    "ring_pip": {"flex": "FIST", "extend": "OPEN"},
    # 小指
    "pinky_abd": {"flex": "OPEN", "extend": "FIST"},
    "pinky_mcp": {"flex": "FIST", "extend": "OPEN"},
    "pinky_pip": {"flex": "FIST", "extend": "OPEN"},
    # 注意: 手腕(wrist)不在此手动校准 —— 高减速比, 手动反驱动会损坏齿轮。
}

POSE_INSTRUCTIONS = {
    "OPEN": (
        "【姿态① 全张开】\n"
        "  四指完全伸直, 并**尽量向左右四周分开** (这是手指左右扭动/外展 abd 的极限);\n"
        "  拇指完全伸直并向外(远离掌心)张开。\n"
        "  -> 手指既要伸到「伸不直」, 也要分到「分不开」。"
    ),
    "FIST": (
        "【姿态② 全握紧】\n"
        "  四指完全握紧弯曲, 指尖压向掌心, 四指**尽量并拢** (abd 的另一极限);\n"
        "  拇指扣过来捏向食指方向(对掌)。\n"
        "  -> 每根手指弯到「弯不下」, 同时并到「并不拢」。"
    ),
}

POSE_ORDER = ["OPEN", "FIST"]


def read_pose(hand, pose_name):
    """提示用户掰到指定姿态, 按回车后记录所有电机位置。"""
    print("\n" + "=" * 64)
    print(POSE_INSTRUCTIONS[pose_name])
    print("-" * 64)
    print("掰好后, 保持不动, 然后按 回车 记录...")
    input(">>> 按回车记录当前位置 <<<")
    pos = hand.get_motor_pos(as_dict=True)
    preview = ", ".join(f"{mid}:{pos[mid]:+.2f}" for mid in list(hand.motor_ids)[:4])
    print(f"  已记录 {pose_name}  | 部分电机位置: {preview} ...")
    return pos


def main():
    parser = argparse.ArgumentParser(
        description="手动姿态校准 (飞特舵机). 把手掰到几个整体姿态完成校准。"
    )
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="orcahand model 文件夹路径")
    args = parser.parse_args()

    hand = OrcaHand(model_path=args.model_path)
    ok, msg = hand.connect()
    if not ok:
        print(f"连接失败: {msg}")
        return 1
    print("连接成功。")

    # 关闭扭矩 -> 手自由, 你可以随意掰动 (电机编码器仍可读位置)
    hand.disable_torque()
    print("扭矩已关闭, 现在可以自由掰动手部 (会读取电机位置)。\n")

    poses = {}
    try:
        print("=" * 64)
        print("开始姿态校准。每个姿态掰到位后按回车。")
        print("(随时可按 Ctrl+C 中止, 已读取的数据不会被保存)\n")
        for pname in POSE_ORDER:
            poses[pname] = read_pose(hand, pname)

        # ---- 计算手指关节的 motor_limits 与 joint_to_motor_ratios ----
        # 严格复用 core.py _calibrate 的 sign 判定, 保证与正反向关节自洽。
        # 手腕不在 JOINT_POSE_MAP 中, 保留 calibration.yaml 现有值不动。
        motor_limits = {mid: list(hand.motor_limits_dict[mid]) for mid in hand.motor_ids}
        ratios = dict(hand.joint_to_motor_ratios_dict)
        for joint in JOINT_POSE_MAP.keys():
            motor_id = hand.joint_to_motor_map[joint]
            pm = JOINT_POSE_MAP[joint]
            flex_pos = poses[pm["flex"]][motor_id]
            extend_pos = poses[pm["extend"]][motor_id]
            rom = hand.joint_roms_dict[joint]

            for direction, limit in [("flex", flex_pos), ("extend", extend_pos)]:
                sign = 1 if direction == "flex" else -1
                if hand.joint_inversion_dict.get(joint, False):
                    sign = -sign
                if sign == 1:
                    motor_limits[motor_id][1] = float(limit)
                else:
                    motor_limits[motor_id][0] = float(limit)

            ratios[motor_id] = float(
                (motor_limits[motor_id][1] - motor_limits[motor_id][0])
                / (rom[1] - rom[0])
            )

        # ---- 展示结果供你确认 ----
        print("\n" + "=" * 64)
        print("校准计算结果 (请检查 Δ电机 是否合理: 手指关节 Δ 通常在 1~4 rad)")
        print(f"{'关节':<12}{'电机ID':>6}{'Δ电机rad':>10}{'Δ≈度':>8}{'ratio':>10}")
        print("-" * 64)
        for joint in JOINT_POSE_MAP.keys():
            mid = hand.joint_to_motor_map[joint]
            ml = motor_limits[mid]
            dm = abs(ml[1] - ml[0])
            print(f"{joint:<12}{mid:>6}{dm:>10.3f}{dm*57.3:>8.1f}{ratios[mid]:>10.5f}")
        print("  (wrist 保留旧数据, 未改动)")

        print("-" * 64)
        print("按 回车 保存到 calibration.yaml  (或 Ctrl+C 放弃不保存)")
        input(">>> 按回车保存 <<<")

        update_yaml(hand.calib_path, "motor_limits", motor_limits)
        update_yaml(hand.calib_path, "joint_to_motor_ratios", ratios)
        update_yaml(hand.calib_path, "calibrated", True)
        print("已保存到 calibration.yaml")

        # ---- 验证: 使能扭矩后读取关节角度, 应在 ROM 内 ----
        hand.enable_torque()
        hand._wrap_offsets_dict = None  # 强制重算 wrap offset
        hand.get_motor_pos()
        jpos = hand.get_joint_pos(as_list=False)
        print("\n" + "=" * 64)
        print("校准后当前关节角度 (应在你掰到的姿态附近, 且都在 ROM 内):")
        for j in JOINT_POSE_MAP.keys():
            rom = hand.joint_roms_dict[j]
            v = jpos[j]
            vs = f"{v:+7.1f}°" if v is not None else "    NA "
            flag = "OK" if (v is not None and rom[0] - 5 <= v <= rom[1] + 5) else "??"
            print(f"  {j:<12}{vs}   ROM[{rom[0]:+.0f},{rom[1]:+.0f}]  {flag}")
        print("\n如果某个关节角度明显不对, 可能是没掰到位, 可重新运行本脚本。")

    except KeyboardInterrupt:
        print("\n已中止, 不保存。")
    finally:
        try:
            hand.disable_torque()
            hand.disconnect()
            print("已断开, 扭矩已释放。")
        except Exception as e:
            print(f"断开异常: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
