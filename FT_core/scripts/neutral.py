#!/usr/bin/env python3

import argparse
import sys
import os
import time

# Add the parent directory to the Python path so we can import orca_core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(description='Move OrcaHand to neutral position.')
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)"
    )
    parser.add_argument(
        "--port",
        default=None,
        help="串口覆盖(如 COM7)，不指定则用 config.yaml 的 port"
    )

    args = parser.parse_args()

    try:
        # Initialize the hand
        hand = OrcaHand(model_path=args.model_path, port=args.port)
            
        # Connect to the hand
        success, message = hand.connect()
        if not success:
            print(f"Failed to connect: {message}")
            return 1
            
        print("Connected to hand successfully")
        
        # Enable torque
        hand.enable_torque()
        print("Torque enabled")
        print("Available motor IDs:", hand.motor_ids)
        # Move to neutral position
        print("Moving to neutral position...")
        hand.set_neutral_position()
        print("Reached neutral position. Holding (torque on). Press Ctrl+C to release and disconnect.")

        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            hand.disable_torque()
            hand.disconnect()
            print("Disconnected from hand")

        return 0
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 