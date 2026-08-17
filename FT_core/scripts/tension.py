import argparse
from orca_core import OrcaHand
import time
import numpy as np

def main():
    parser = argparse.ArgumentParser(
        description="Enable torque and hold tension on the ORCA Hand. "
                    "Specify the path to the orcahand model folder."
    )
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    parser.add_argument('--move_motors', action='store_true', help='If set, move motors 1-16 continuously positively for 3 seconds with calibration current.')
    parser.add_argument('--continuous', action='store_true', help='If set, keep winding the motors to tension the tendons until Ctrl+C (then hold position).')
    parser.add_argument('--port', default=None, help='串口覆盖(如 COM7)，不指定则用 config.yaml 的 port')

    args = parser.parse_args()

    hand = OrcaHand(args.model_path, port=args.port)
    status = hand.connect()
    if not status[0]:
        print(f"Failed to connect to the hand. Reason: {status[1]}")
        exit(1)

    hand.enable_torque()

    hand.tension(args.move_motors, continuous=args.continuous)
    
    hand.disconnect()

if __name__ == "__main__":
    main()