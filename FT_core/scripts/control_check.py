#!/usr/bin/env python3
"""
控制链路自检 + 平滑归中位 (飞特舵机, 手指装了视触觉装置)。

先【只读】打印每个关节的当前角度 vs ROM, 用来验证校准数据 / wrap offset
在控制链路里是否正常 —— 这是从校准进入实际控制前的关键检查点。
你确认读数合理后按回车, 才会平滑移动到 neutral, 全程无意外运动。

用法:
  python scripts/control_check.py orca_core/models/orcahand_v1_right
"""
import argparse
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand

# 校准数据里 ratio 明显偏小的关节(上次没掰到位)。控制时这几个的实际行程
# 会很小, 是预期内的现象, 不是 bug。
SUSPECT = {"middle_mcp", "thumb_mcp", "index_pip", "pinky_pip", "middle_pip"}


def main():
    parser = argparse.ArgumentParser(description="控制自检 + 平滑归中位")
    parser.add_argument("model_path", nargs="?", default=None,
                        help="orcahand model 文件夹路径")
    args = parser.parse_args()

    hand = OrcaHand(model_path=args.model_path)
    ok, msg = hand.connect()
    if not ok:
        print(f"连接失败: {msg}")
        return 1
    print("连接成功。")

    # 规范初始化(比 neutral.py 多了 control_mode / max_current 的设置)
    hand.enable_torque()
    try:
        hand.set_control_mode(hand.control_mode)
    except Exception as e:
        print(f"(set_control_mode 跳过: {e})")
    try:
        hand.set_max_current(hand.max_current)
    except Exception as e:
        print(f"(set_max_current 跳过: {e})")

    # ---------- 只读: 验证换算与 wrap offset ----------
    print("\n" + "=" * 70)
    print("当前关节读数 (只读, 不运动)。应都在 ROM 内, 且接近手现在的姿态。")
    print(f"{'关节':<12}{'当前°':>9}{'ROM':>16}{'状态':>14}")
    print("-" * 70)
    jpos = hand.get_joint_pos(as_list=False)
    bad = 0
    for j in hand.joint_ids:
        rom = hand.joint_roms_dict[j]
        v = jpos[j]
        if v is None:
            print(f"{j:<12}{'NA':>9}{f'[{rom[0]:+.0f},{rom[1]:+.0f}]':>16}{'NULL':>14}")
            bad += 1
            continue
        inside = rom[0] - 5 <= v <= rom[1] + 5
        flag = "OK" if inside else "越界?"
        if j in SUSPECT:
            flag += " *偏小"
        if not inside:
            bad += 1
        print(f"{j:<12}{v:>+9.1f}{f'[{rom[0]:+.0f},{rom[1]:+.0f}]':>16}{flag:>14}")
    print("-" * 70)
    print("* = 该关节校准时没掰到位, 控制时实际动幅偏小(预期内)。")
    if bad:
        print(f"!! 有 {bad} 个关节读数异常(越界/NULL), 可能 wrap offset 错位,")
        print("   建议 先排查 再运动, 不要按下面的回车。")

    # ---------- 确认后才运动 ----------
    print("\n确认读数合理后, 按回车 平滑归中位 (Ctrl+C 中止, 不会运动):")
    try:
        input(">>> 按回车继续 <<<")
    except KeyboardInterrupt:
        print("\n已中止, 未运动。")
        hand.disable_torque()
        hand.disconnect()
        return 0

    print("平滑移动到 neutral ...")
    hand.set_joint_pos(hand.neutral_position, num_steps=150, step_size=0.015)

    print("已到 neutral。读回验证 (实际应接近目标):")
    jpos2 = hand.get_joint_pos(as_list=False)
    for j in hand.joint_ids:
        tgt = hand.neutral_position.get(j)
        v = jpos2[j]
        vs = f"{v:+.1f}" if v is not None else "NA"
        print(f"  {j:<12} 目标 {tgt:+.0f}°   实际 {vs}°")

    hand.disable_torque()
    hand.disconnect()
    print("\n完成。链路正常的话, 可继续跑 slider_joint.py 逐关节交互测试。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
