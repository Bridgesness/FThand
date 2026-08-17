# ==============================================================================
# 手套 UDP 数据监控脚本
# 用于接收和显示数据手套的原始数据，不涉及任何硬件控制
#
# 功能:
#   - 通过 UDP 直接接收数据手套的关节数据
#   - 解析并打印关节数据 (Vector3Float)
#   - 解析并打印控制器数据 (摇杆、按键)
#   - 实时显示数据更新状态
#
# 用途: 调试和验证手套数据传输
# ==============================================================================

import sys
import os
import time
import socket
import json
import threading
import argparse
from typing import List, Dict, Optional

# 添加父目录到路径以使用共享模块
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ============================================================================
# 数据类
# ============================================================================

class Vector3Float:
    """3D 向量类"""
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"({self.x:7.2f}, {self.y:7.2f}, {self.z:7.2f})"


# ============================================================================
# 手套数据监控器
# ============================================================================

class GloveDataMonitor:
    """
    手套 UDP 数据监控器
    接收并显示手套数据，不涉及硬件控制
    """

    # 手套关节数据头 (30 个关节，每个为 Vector3Float)
    GLOVE_DATA_HEADERS = [
        "LeftThumb1", "LeftThumb2", "LeftThumb3",
        "LeftIndex1", "LeftIndex2", "LeftIndex3",
        "LeftMiddle1", "LeftMiddle2", "LeftMiddle3",
        "LeftRing1", "LeftRing2", "LeftRing3",
        "LeftPinky1", "LeftPinky2", "LeftPinky3",
        "RightThumb1", "RightThumb2", "RightThumb3",
        "RightIndex1", "RightIndex2", "RightIndex3",
        "RightMiddle1", "RightMiddle2", "RightMiddle3",
        "RightRing1", "RightRing2", "RightRing3",
        "RightPinky1", "RightPinky2", "RightPinky3"
    ]

    # 控制器数据头 (12 个值)
    CONTROLLER_HEADERS = [
        "Left Joy X", "Left Joy Y", "Left A Button", "Left B Button",
        "Left Joy Button", "Left Menu Button",
        "Right Joy X", "Right Joy Y", "Right A Button", "Right B Button",
        "Right Joy Button", "Right Menu Button"
    ]

    def __init__(self, udp_port: int = 8000, verbose: bool = True):
        """
        Args:
            udp_port: UDP 监听端口
            verbose: 是否详细输出
        """
        self.udp_port = udp_port
        self.verbose = verbose
        self.sock = None
        self.running = False
        self.recv_thread: Optional[threading.Thread] = None

        # 数据存储
        self.data_lock = threading.Lock()
        self.raw_data: Dict = {}
        self.glove_data: Dict = {}
        self.controller_data: Dict = {}

        # 解析后的关节数据
        self.left_finger_data: List[Vector3Float] = [Vector3Float(0, 0, 0)] * 15
        self.right_finger_data: List[Vector3Float] = [Vector3Float(0, 0, 0)] * 15
        self.left_controller: List[float] = [0.0] * 6
        self.right_controller: List[float] = [0.0] * 6

        # 统计信息
        self.packet_count = 0
        self.last_packet_time = 0.0
        self.start_time = 0.0
        self.packets_per_second = 0

    def initialize(self) -> bool:
        """初始化 UDP 套接字"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", self.udp_port))
            self.sock.settimeout(1.0)
            print(f"[Monitor] UDP 套接字已绑定到端口 {self.udp_port}")
            return True
        except Exception as e:
            print(f"[Monitor] 初始化失败: {e}")
            return False

    def start(self):
        """启动监控"""
        self.running = True
        self.start_time = time.time()

        print("[Monitor] 开始监控手套数据...")
        print("[Monitor] 按 Ctrl+C 停止")
        print("=" * 80)

        try:
            # 接收数据循环
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024 * 1024)
                    self._process_data(data, addr)
                except socket.timeout:
                    # 检查超时
                    if time.time() - self.last_packet_time > 2.0 and self.packet_count > 0:
                        print(f"[Monitor] 警告: 超过2秒未收到数据")
                    continue
                except Exception as e:
                    if self.running:
                        print(f"[Monitor] 接收错误: {e}")

        except KeyboardInterrupt:
            print("\n[Monitor] 收到停止信号")
        finally:
            self.stop()

    def stop(self):
        """停止监控"""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass

        elapsed = time.time() - self.start_time
        print("\n" + "=" * 80)
        print(f"[Monitor] 监控已停止")
        print(f"[Monitor] 总接收数据包: {self.packet_count}")
        print(f"[Monitor] 运行时间: {elapsed:.1f} 秒")
        if elapsed > 0:
            print(f"[Monitor] 平均频率: {self.packet_count / elapsed:.1f} 包/秒")
        print("=" * 80)

    def _process_data(self, data: bytes, addr: tuple):
        """处理接收到的数据"""
        self.packet_count += 1
        self.last_packet_time = time.time()

        try:
            # 解析 JSON
            json_str = data.decode("utf-8")
            value = json.loads(json_str)

            with self.data_lock:
                self.raw_data = value

                # 解析手套数据和控制器数据
                for role_name, device in value.items():
                    parameters = device.get("Parameter", [])

                    for param in parameters:
                        name = param.get("Name", "")
                        val = param.get("Value", 0.0) if "Value" in param else 0.0

                        # 判断是控制器数据还是手套数据
                        if len(name) >= 2 and name[1] == '_' and (name[0] == 'l' or name[0] == 'r'):
                            if name not in self.controller_data:
                                self.controller_data[name] = val
                            else:
                                self.controller_data[name] = val
                        else:
                            if name not in self.glove_data:
                                self.glove_data[name] = val
                            else:
                                self.glove_data[name] = val

                # 解析关节数据
                self._parse_finger_data()
                self._parse_controller_data()

            # 打印数据
            self._print_data(addr, json_str)

        except json.JSONDecodeError as e:
            print(f"[Monitor] JSON 解析错误: {e}")
        except Exception as e:
            print(f"[Monitor] 处理数据错误: {e}")

    def _parse_finger_data(self):
        """解析手指关节数据"""
        # 左手 15 个关节
        self.left_finger_data = [
            Vector3Float(self.glove_data.get("l2", 0), self.glove_data.get("l3", 0), self.glove_data.get("l20", 0)),
            Vector3Float(self.glove_data.get("l1", 0), 0, 0),
            Vector3Float(self.glove_data.get("l0", 0), 0, 0),
            Vector3Float(self.glove_data.get("l6", 0), self.glove_data.get("l7", 0), self.glove_data.get("l21", 0)),
            Vector3Float(self.glove_data.get("l5", 0), 0, 0),
            Vector3Float(self.glove_data.get("l4", 0), 0, 0),
            Vector3Float(self.glove_data.get("l10", 0), self.glove_data.get("l11", 0), 0),
            Vector3Float(self.glove_data.get("l9", 0), 0, 0),
            Vector3Float(self.glove_data.get("l8", 0), 0, 0),
            Vector3Float(self.glove_data.get("l14", 0), self.glove_data.get("l15", 0), 0),
            Vector3Float(self.glove_data.get("l13", 0), 0, 0),
            Vector3Float(self.glove_data.get("l12", 0), 0, 0),
            Vector3Float(self.glove_data.get("l18", 0), self.glove_data.get("l19", 0), self.glove_data.get("l22", 0)),
            Vector3Float(self.glove_data.get("l17", 0), 0, 0),
            Vector3Float(self.glove_data.get("l16", 0), 0, 0),
        ]

        # 右手 15 个关节
        self.right_finger_data = [
            Vector3Float(self.glove_data.get("r2", 0), self.glove_data.get("r3", 0), self.glove_data.get("r20", 0)),
            Vector3Float(self.glove_data.get("r1", 0), 0, 0),
            Vector3Float(self.glove_data.get("r0", 0), 0, 0),
            Vector3Float(self.glove_data.get("r6", 0), self.glove_data.get("r7", 0), self.glove_data.get("r21", 0)),
            Vector3Float(self.glove_data.get("r5", 0), 0, 0),
            Vector3Float(self.glove_data.get("r4", 0), 0, 0),
            Vector3Float(self.glove_data.get("r10", 0), self.glove_data.get("r11", 0), 0),
            Vector3Float(self.glove_data.get("r9", 0), 0, 0),
            Vector3Float(self.glove_data.get("r8", 0), 0, 0),
            Vector3Float(self.glove_data.get("r14", 0), self.glove_data.get("r15", 0), 0),
            Vector3Float(self.glove_data.get("r13", 0), 0, 0),
            Vector3Float(self.glove_data.get("r12", 0), 0, 0),
            Vector3Float(self.glove_data.get("r18", 0), self.glove_data.get("r19", 0), self.glove_data.get("r22", 0)),
            Vector3Float(self.glove_data.get("r17", 0), 0, 0),
            Vector3Float(self.glove_data.get("r16", 0), 0, 0),
        ]

    def _parse_controller_data(self):
        """解析控制器数据"""
        self.left_controller = [
            self.controller_data.get("l_joyX", 0.0),
            self.controller_data.get("l_joyY", 0.0),
            self.controller_data.get("l_aButton", 0.0),
            self.controller_data.get("l_bButton", 0.0),
            self.controller_data.get("l_joyButton", 0.0),
            self.controller_data.get("l_menu", 0.0),
        ]

        self.right_controller = [
            self.controller_data.get("r_joyX", 0.0),
            self.controller_data.get("r_joyY", 0.0),
            self.controller_data.get("r_aButton", 0.0),
            self.controller_data.get("r_bButton", 0.0),
            self.controller_data.get("r_joyButton", 0.0),
            self.controller_data.get("r_menu", 0.0),
        ]

    def _print_data(self, addr: tuple, raw_json: str):
        """打印接收到的数据"""
        # 清屏（可选，注释掉以保留历史）
        # os.system('cls' if os.name == 'nt' else 'clear')

        print("\n" + "=" * 80)
        print(f"数据包 #{self.packet_count} | 来自: {addr[0]}:{addr[1]} | 时间: {time.strftime('%H:%M:%S')}")
        print("=" * 80)

        # 打印左手数据
        print("\n【左手数据】")
        print("-" * 80)
        print(f"{'关节':<12} {'X':>10} {'Y':>10} {'Z':>10}")
        print("-" * 80)
        for i, (header, vec) in enumerate(zip(self.GLOVE_DATA_HEADERS[:15], self.left_finger_data)):
            print(f"{header:<12} {vec}")

        # 打印右手数据
        print("\n【右手数据】")
        print("-" * 80)
        print(f"{'关节':<12} {'X':>10} {'Y':>10} {'Z':>10}")
        print("-" * 80)
        for i, (header, vec) in enumerate(zip(self.GLOVE_DATA_HEADERS[15:], self.right_finger_data)):
            print(f"{header:<12} {vec}")

        # 打印控制器数据
        print("\n【控制器数据】")
        print("-" * 80)
        print("左手控制器:")
        for i, (header, val) in enumerate(zip(self.CONTROLLER_HEADERS[:6], self.left_controller)):
            print(f"  {header:<20} : {val:7.3f}")

        print("右手控制器:")
        for i, (header, val) in enumerate(zip(self.CONTROLLER_HEADERS[6:], self.right_controller)):
            print(f"  {header:<20} : {val:7.3f}")

        # 打印原始手套数据
        if self.verbose:
            print("\n【原始手套数据】")
            print("-" * 80)
            for key, val in sorted(self.glove_data.items()):
                print(f"  {key:<8} : {val:8.2f}")

        # 打印原始控制器数据
        if self.verbose:
            print("\n【原始控制器数据】")
            print("-" * 80)
            for key, val in sorted(self.controller_data.items()):
                print(f"  {key:<12} : {val:8.3f}")

        print("=" * 80)

    def get_current_data(self) -> Dict:
        """获取当前数据（用于其他模块访问）"""
        with self.data_lock:
            return {
                "left_finger": self.left_finger_data.copy(),
                "right_finger": self.right_finger_data.copy(),
                "left_controller": self.left_controller.copy(),
                "right_controller": self.right_controller.copy(),
                "glove_data": self.glove_data.copy(),
                "controller_data": self.controller_data.copy(),
            }


# ============================================================================
# 简易监控模式（仅显示关键信息）
# ============================================================================

class SimpleGloveMonitor:
    """简易手套监控器 - 显示右手所有数据"""

    def __init__(self, udp_port: int = 8000):
        self.udp_port = udp_port
        self.sock = None
        self.running = False
        self.packet_count = 0
        self.last_data = None

    def start(self):
        """启动简易监控"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", self.udp_port))
            self.sock.settimeout(1.0)
        except Exception as e:
            print(f"[错误] 绑定端口失败: {e}")
            return

        self.running = True
        print(f"[简易监控] 监听端口 {self.udp_port}")
        print("[简易监控] 按 Ctrl+C 停止\n")

        try:
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024 * 1024)
                    self.packet_count += 1

                    # 快速解析并显示
                    try:
                        value = json.loads(data.decode("utf-8"))

                        # 提取关键数据
                        all_params = {}
                        for device in value.values():
                            for param in device.get("Parameter", []):
                                name = param.get("Name", "")
                                val = param.get("Value", 0.0)
                                all_params[name] = val

                        # 显示右手所有数据 (r0-r22)
                        timestamp = time.strftime("%H:%M:%S")

                        # 显示全部 23 个变量
                        print(f"[{timestamp}] 包#{self.packet_count:05d}")
                        print(f"  r0 ={all_params.get('r0', 0):7.2f}  r1 ={all_params.get('r1', 0):7.2f}  r2 ={all_params.get('r2', 0):7.2f}  r3 ={all_params.get('r3', 0):7.2f}")
                        print(f"  r4 ={all_params.get('r4', 0):7.2f}  r5 ={all_params.get('r5', 0):7.2f}  r6 ={all_params.get('r6', 0):7.2f}  r7 ={all_params.get('r7', 0):7.2f}")
                        print(f"  r8 ={all_params.get('r8', 0):7.2f}  r9 ={all_params.get('r9', 0):7.2f}  r10={all_params.get('r10', 0):7.2f}  r11={all_params.get('r11', 0):7.2f}")
                        print(f"  r12={all_params.get('r12', 0):7.2f}  r13={all_params.get('r13', 0):7.2f}  r14={all_params.get('r14', 0):7.2f}  r15={all_params.get('r15', 0):7.2f}")
                        print(f"  r16={all_params.get('r16', 0):7.2f}  r17={all_params.get('r17', 0):7.2f}  r18={all_params.get('r18', 0):7.2f}  r19={all_params.get('r19', 0):7.2f}")
                        print(f"  r20={all_params.get('r20', 0):7.2f}  r21={all_params.get('r21', 0):7.2f}  r22={all_params.get('r22', 0):7.2f}")

                    except json.JSONDecodeError:
                        print(f"[{time.strftime('%H:%M:%S')}] 包#{self.packet_count:05d} | JSON 解析失败")

                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[错误] {e}")

        except KeyboardInterrupt:
            print(f"\n[简易监控] 停止，共接收 {self.packet_count} 个数据包")
        finally:
            if self.sock:
                self.sock.close()


# ============================================================================
# 主函数
# ============================================================================

def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='手套 UDP 数据监控工具 - 不涉及硬件控制',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 完整数据输出（左右手所有数据）
  python monitor_glove_udp.py

  # 简易模式（仅显示右手所有数据）
  python monitor_glove_udp.py --simple

  # 指定端口
  python monitor_glove_udp.py -p 5555

  # 静默模式（不显示原始数据）
  python monitor_glove_udp.py -q
        """
    )

    parser.add_argument(
        '-p', '--port',
        type=int,
        default=8000,
        help='UDP 监听端口 (默认: 8000)'
    )
    parser.add_argument(
        '--simple',
        action='store_true',
        help='简易模式，仅显示右手所有数据'
    )
    parser.add_argument(
        '-q', '--quiet',
        action='store_true',
        help='静默模式，不显示原始数据'
    )

    args = parser.parse_args()

    print("=" * 80)
    print(" 手套 UDP 数据监控工具")
    print(" (不涉及硬件控制)")
    print("=" * 80)
    print()

    if args.simple:
        # 简易模式
        monitor = SimpleGloveMonitor(udp_port=args.port)
        monitor.start()
    else:
        # 完整模式
        monitor = GloveDataMonitor(udp_port=args.port, verbose=not args.quiet)

        if not monitor.initialize():
            print("[错误] 初始化失败")
            return 1

        monitor.start()

    return 0


if __name__ == "__main__":
    exit(main())
