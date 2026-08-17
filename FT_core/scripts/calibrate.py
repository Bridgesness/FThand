import argparse
from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the ORCA Hand. Specify the path to the orcahand model folder."
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    parser.add_argument(
        "--joint",
        type=str,
        default=None,
        help="Only calibrate this joint (comma-separated for several), e.g. --joint pinky_pip. "
             "Other joints keep their existing calibration."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="串口覆盖(如 COM7)，不指定则用 config.yaml 的 port"
    )
    parser.add_argument(
        "--skip",
        type=str,
        default=None,
        help="跳过这些关节不标定（逗号分隔），如 --skip pinky_abd。用于肌腱断/不可用的关节。"
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path, port=args.port)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    if args.joint or args.skip:
        wanted = [j.strip() for j in args.joint.split(",") if j.strip()] if args.joint else None
        skip = [j.strip() for j in args.skip.split(",") if j.strip()] if args.skip else []
        for j in (wanted or []) + skip:
            if j not in hand.joint_ids:
                print(f"Unknown joint: {j}")
                print(f"Valid joints: {list(hand.joint_ids)}")
                exit(1)
        kept = []
        for step in hand.calib_sequence:
            step_joints = step.get("joints", {})
            sub = {}
            for j, d in step_joints.items():
                if wanted is not None and j not in wanted:
                    continue
                if j in skip:
                    continue
                sub[j] = d
            if sub:
                kept.append({"step": len(kept) + 1, "joints": sub})
        if not kept:
            print("No calibration steps left after filter.")
            exit(1)
        hand.calib_sequence = kept
        msg = []
        if wanted is not None:
            msg.append(f"only={wanted}")
        if skip:
            msg.append(f"skip={skip}")
        print(f"Calibration filtered: {' '.join(msg)}  (其余关节保留旧标定)")

    hand.calibrate()

if __name__ == "__main__":
    main()