import argparse
import logging
import os
from orca_core.hardware.hl_client import HLClient, COMM_SUCCESS
from orca_core.utils.utils import get_model_path, read_yaml

# 扫描电机总线:逐个 ID 发 PING,看哪些电机真的在线。
# 只读、不启用扭矩,所以不会卡在死掉的电机上。
#
# 用法(在 FT_core 目录下):
#   python scripts\scan_bus.py orca_core\models\orcahand_v1_left

def main():
    logging.disable(logging.CRITICAL)  # 屏蔽 SDK 的逐包报错噪音

    parser = argparse.ArgumentParser(description="Scan the motor bus by PINGing each ID.")
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the orcahand model folder")
    parser.add_argument("--max-id", type=int, default=40,
                        help="Scan motor IDs 0..max-id (default 40)")
    args = parser.parse_args()

    model_path = get_model_path(args.model_path)
    cfg = read_yaml(os.path.join(model_path, "config.yaml"))
    port = cfg.get("port")
    baud = cfg.get("baudrate", 1000000)
    expected = set(cfg.get("motor_ids", []))

    print(f"端口: {port}   波特率: {baud}")
    print(f"config 期望的 motor_ids: {sorted(expected)}")

    # 直接构造 client,但不调用 connect()(connect() 会给所有电机 enable torque,死电机会卡死)
    client = HLClient(list(range(0, args.max_id + 1)), port, baud)
    if not client.port_handler.openPort():
        print(f"\n无法打开端口 {port} —— 端口被占或驱动异常。")
        return
    if not client.port_handler.setBaudRate(baud):
        print(f"\n无法设置波特率 {baud}。")
        return
    client.packetHandler = client.ft.hls(client.port_handler)

    print(f"\n扫描 ID 0..{args.max_id}(每个无应答 ID 会等一个超时,稍慢)...\n")
    alive = []
    for mid in range(0, args.max_id + 1):
        model, result, err = client.packetHandler.ping(mid)
        if result == COMM_SUCCESS:
            print(f"  ID {mid:>3} : 在线  (model={model}, err={err})")
            alive.append(mid)

    print("\n================ 结果 ================")
    if alive:
        print(f"应答的电机 ID: {alive}")
    else:
        print("没有任何电机应答! 整条总线都是哑的 -> 几乎肯定是供电/主线问题。")

    missing = sorted(expected - set(alive))
    extra = sorted(set(alive) - expected)
    if missing:
        print(f"配置要求但没应答(死/掉线): {missing}")
    if extra:
        print(f"应答了但不在配置里(意外电机): {extra}")

    client.port_handler.closePort()


if __name__ == "__main__":
    main()
