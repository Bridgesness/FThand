import argparse
import socket
import json
import sys

# 实时查看手套 UDP 变量(调参用)。
# 用法(在 FT_core 目录下):
#   python scripts\watch_glove_vars.py                    # 默认看 5 个手指的外展变量
#   python scripts\watch_glove_vars.py l11                # 只看 l11
#   python scripts\watch_glove_vars.py l8 l9 l10 l11 -p 8000

def main():
    parser = argparse.ArgumentParser(description="Live-view selected glove UDP variables (tuning helper).")
    parser.add_argument("vars", nargs="*", default=["l3", "l7", "l11", "l15", "l19"],
                        help="要查看的变量名(默认五个手指的外展变量)")
    parser.add_argument("-p", "--port", type=int, default=8000, help="UDP 端口(默认 8000)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)

    print(f"监听 UDP {args.port},实时显示: {args.vars}")
    print("动手指观察数值范围,Ctrl+C 退出。\n")

    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024 * 1024)
                val = json.loads(data.decode("utf-8"))
                params = {}
                for dev in val.values():
                    for p in dev.get("Parameter", []):
                        params[p.get("Name")] = p.get("Value", 0.0)
                line = "  ".join(f"{v}={params.get(v, 0.0):7.2f}" for v in args.vars)
                sys.stdout.write("\r " + line + "        ")
                sys.stdout.flush()
            except socket.timeout:
                sys.stdout.write("\r (没有收到手套数据)        ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
