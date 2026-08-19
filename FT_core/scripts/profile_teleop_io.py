# ==============================================================================
# 遥操控制链路 I/O 微基准
#
# 测量 set_joint_pos 每帧的串口往返分解:
#   - 内部真实调用了多少次 read / write
#   - 单次读 / 写耗时
#   - 等效控制频率 (1000 / 平均帧间隔)
#   - 周期抖动 (相邻帧间隔 max/min/std)
#   - 每帧吞吐字节数 (按协议包长估算)
#
# 用法:
#   python scripts/profile_teleop_io.py --model orca_core/models/orcahand_v1_left --iters 200
#   python scripts/profile_teleop_io.py --mock        # 不接硬件, 纯验证脚本跑通
#
# 输出会同时给出 [优化前(强制 2 读+1 写)] 与 [优化后(只写)] 两列, 直接可对比。
# ==============================================================================

import sys
import os
import time
import argparse

import numpy as np

# Windows 控制台默认 GBK, 这里统一成 UTF-8, 避免中文/符号打印报错
if sys.platform == 'win32':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand, MockOrcaHand

# 协议包长估算 (FeiTech HLS @ 1Mbaud, 8N1):
#   同步读请求: 2+1+1+1+1+1 + 16*1 + 1 = 24 字节
#   同步读响应: 16 * 10 = 160 字节
#   同步写请求: 2+1+1+1+1+1 + 16*3 + 1 = 56 字节
READ_BYTES = 24 + 160      # 每读往返
WRITE_BYTES = 56           # 每写


class SerialProfiler:
    """Monkeypatch 包住 client 的读/写方法, 统计调用次数与耗时."""

    def __init__(self, hand):
        self.hand = hand
        self.read_times = []
        self.write_times = []
        self.read_count = 0
        self.write_count = 0

        client = hand._dxl_client
        self._orig_read = client.read_pos_vel_cur
        self._orig_write = client.write_desired_pos
        client.read_pos_vel_cur = self._wrap_read(self._orig_read)
        client.write_desired_pos = self._wrap_write(self._orig_write)

    def _wrap_read(self, fn):
        def w(*args, **kwargs):
            t = time.perf_counter()
            r = fn(*args, **kwargs)
            self.read_times.append((time.perf_counter() - t) * 1000.0)
            self.read_count += 1
            return r
        return w

    def _wrap_write(self, fn):
        def w(*args, **kwargs):
            t = time.perf_counter()
            r = fn(*args, **kwargs)
            self.write_times.append((time.perf_counter() - t) * 1000.0)
            self.write_count += 1
            return r
        return w

    def restore(self):
        client = self.hand._dxl_client
        client.read_pos_vel_cur = self._orig_read
        client.write_desired_pos = self._orig_write


def stat(vals):
    if len(vals) == 0:  # 优化后 0 次读是正常情况, 不能判 np.array 的真值
        return "  -  "
    a = np.array(vals)
    return f"avg={a.mean():6.2f}  max={a.max():6.2f}  std={a.std():5.2f}"


def bench(hand, iters, legacy):
    """跑 iters 次 set_joint_pos, 返回 (profiler, 帧间隔列表)."""
    prof = SerialProfiler(hand)
    intervals = []
    t_last = time.perf_counter()

    for _ in range(iters):
        if legacy:
            # 复现"优化前"热路径: _joint_to_motor_pos 的 len(get_motor_pos()) +
            # _set_motor_pos 的无条件读 = 2 次全量读, 再加 1 次写
            hand.get_motor_pos()
            hand.get_motor_pos()
        hand.set_joint_pos(hand.neutral_position)

        now = time.perf_counter()
        intervals.append((now - t_last) * 1000.0)
        t_last = now

    prof.restore()
    return prof, intervals


def report(label, prof, intervals, n):
    per_frame = np.array(intervals)
    read_ms = prof.read_times
    write_ms = prof.write_times
    reads_per_frame = prof.read_count / n
    writes_per_frame = prof.write_count / n
    bytes_per_frame = reads_per_frame * READ_BYTES + writes_per_frame * WRITE_BYTES

    print(f"\n  [{label}]")
    print(f"    每帧串口调用       : 读 {reads_per_frame:.2f} 次 / 写 {writes_per_frame:.2f} 次  "
          f"({bytes_per_frame:.0f} B/帧)")
    print(f"    单次读耗时(ms)     : {stat(read_ms)}")
    print(f"    单次写耗时(ms)     : {stat(write_ms)}")
    print(f"    帧间隔(ms)         : {stat(per_frame)}")
    print(f"    等效控制频率       : {1000.0 / per_frame.mean():6.1f} Hz"
          f"   (周期峰值可达 {1000.0 / per_frame.min():.0f} Hz)")
    return {
        "label": label,
        "frame_avg_ms": float(per_frame.mean()),
        "hz": 1000.0 / per_frame.mean(),
        "bytes": float(bytes_per_frame),
    }


def main():
    parser = argparse.ArgumentParser(description="遥操控制链路 I/O 微基准")
    parser.add_argument("--model", default="orca_core/models/orcahand_v1_left",
                        help="手模型路径")
    parser.add_argument("--iters", type=int, default=200, help="每档迭代次数")
    parser.add_argument("--mock", action="store_true", help="用 Mock 手(不接硬件)")
    args = parser.parse_args()

    print("=" * 66)
    print(" 遥操控制链路 I/O 微基准  (1Mbaud, 8N1 = 10us/字节)")
    print("=" * 66)

    if args.mock:
        hand = MockOrcaHand(model_path=args.model)
    else:
        hand = OrcaHand(model_path=args.model)
    status = hand.connect()
    if not status[0]:
        print(f"[错误] 连接失败: {status[1]}")
        print("  - 接上串口(COM)后重试, 或用 --mock 跑纯验证")
        return
    print(f"[连接] {status[1]}")

    # 热机: 触发一次 wrap_offsets 计算(首次会多一次读), 不进入统计
    hand._compute_wrap_offsets_dict()
    hand.set_joint_pos(hand.neutral_position)
    time.sleep(0.2)

    print(f"\n跑 {args.iters} 帧 / 档 ...")
    p_after, iv_after = bench(hand, args.iters, legacy=False)
    p_before, iv_before = bench(hand, args.iters, legacy=True)

    r_after = report("优化后 (只写, 绝对位置)", p_after, iv_after, args.iters)
    r_before = report("优化前 (强制 2 读 + 1 写)", p_before, iv_before, args.iters)

    print("\n" + "=" * 66)
    print(" 对比")
    print("=" * 66)
    print(f"  每帧平均耗时 : {r_before['frame_avg_ms']:.2f} ms -> {r_after['frame_avg_ms']:.2f} ms"
          f"   ({-(1 - r_after['frame_avg_ms'] / r_before['frame_avg_ms']) * 100:+.0f}%)")
    print(f"  等效频率     : {r_before['hz']:.1f} Hz -> {r_after['hz']:.1f} Hz"
          f"   ({r_after['hz'] / r_before['hz']:.1f}×)")
    print(f"  每帧吞吐字节 : {r_before['bytes']} B -> {r_after['bytes']} B"
          f"   ({-(1 - r_after['bytes'] / r_before['bytes']) * 100:+.0f}%)")
    print("=" * 66)


if __name__ == "__main__":
    main()
