# ==============================================================================
# OrcaHand Teleoperation System
# 遥操作系统 - 整合数据手套与 FT_core 灵巧手
#
# 功能:
#   - 通过 UDP 接收数据手套的关节数据 (HandDriver_Linux_Py_Angle)
#   - 实时映射并控制 OrcaHand 灵巧手的 17 个关节
#   - 支持双手遥操作 (可选)
#   - 提供平滑控制和可调参数
#
# 数据手套: UDEGloveSDK (端口 8000)
# 灵巧手: OrcaHand (17 关节)
#
# 关节映射规则:
#   - ABD 关节 (外展): 使用手套 y 分量 (外展/内收角度)
#   - 其他关节 (MCP/PIP/DIP): 使用手套 x 分量 (主要弯曲角度)
# ==============================================================================

import sys
import os
import time
import socket
import json
import threading
import math
import argparse
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from collections import deque

# 添加 orca_core 模块到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from orca_core.core import OrcaHand


# ============================================================================
# 数据手套 SDK (基于 HandDriver_Linux_Py_Angle_202504021812.py)
# ============================================================================

class Vector3Float:
    """3D 向量类"""
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


class ServerStatus:
    """服务器状态枚举"""
    NO_INIT = 0
    READY = 1
    IN_LISTENING = 2
    END = 3


class UDEGloveSDK:
    """
    UDE 数据手套 SDK
    通过 UDP 端口 8000 接收手套关节数据
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

    def __init__(self, port: int = 8000):
        self.port = port
        self.sock = None
        self.server_addr = ("0.0.0.0", self.port)
        self.cur_status = ServerStatus.NO_INIT
        self.recv_thread: Optional[threading.Thread] = None

        # 数据存储
        self.glove_data_list: List[Dict] = []
        self.controller_data_list: List[Dict] = []
        self.data_lock = threading.Lock()

        # 解析后的关节数据 (左右手各 15 个 Vector3Float)
        self.left_finger_data: List[Vector3Float] = [Vector3Float(0, 0, 0)] * 15
        self.right_finger_data: List[Vector3Float] = [Vector3Float(0, 0, 0)] * 15

        # 控制器数据
        self.left_controller: List[float] = [0.0] * 6
        self.right_controller: List[float] = [0.0] * 6

        # 状态
        self.last_packet_time = 0.0
        self.packet_count = 0
        self.is_connected = False

    def initialize(self) -> bool:
        """初始化 UDP 服务器"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(self.server_addr)
            self.sock.settimeout(2)
            self.cur_status = ServerStatus.READY
            print(f"[GloveSDK] SDK 已初始化，监听端口 {self.port}")
            return True
        except Exception as e:
            print(f"[GloveSDK] 初始化失败: {e}")
            self.cur_status = ServerStatus.NO_INIT
            return False

    def start_listening(self):
        """开始监听数据"""
        if self.cur_status != ServerStatus.READY:
            print("[GloveSDK] SDK 未就绪，无法开始监听")
            return
        self.cur_status = ServerStatus.IN_LISTENING
        print("[GloveSDK] 开始监听手套数据...")
        self.recv_thread = threading.Thread(target=self._recv_func, daemon=True)
        self.recv_thread.start()

    def end_listening(self):
        """停止监听"""
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=2)
        self.cur_status = ServerStatus.END
        self.is_connected = False
        print("[GloveSDK] 已停止监听")

    def _recv_func(self):
        """接收线程函数"""
        while self.cur_status == ServerStatus.IN_LISTENING:
            try:
                data, addr = self.sock.recvfrom(1024 * 1024)
                self._process_data(data.decode("utf-8"))
                self.is_connected = True
                self.last_packet_time = time.time()
                self.packet_count += 1
            except socket.timeout:
                # 检查超时
                if self.is_connected and time.time() - self.last_packet_time > 1.0:
                    self.is_connected = False
                continue
            except Exception as e:
                if self.cur_status == ServerStatus.IN_LISTENING:
                    print(f"[GloveSDK] 接收数据错误: {e}")

    def _process_data(self, data: str):
        """处理接收到的 JSON 数据"""
        try:
            value = json.loads(data)

            with self.data_lock:
                self.glove_data_list.clear()
                self.controller_data_list.clear()

                for role_name, device in value.items():
                    glove_data = {"roleName": role_name, "handDatas": {}}
                    controller_data = {"roleName": role_name, "controllerDatas": {}}

                    parameters = device.get("Parameter", [])
                    for param in parameters:
                        name = param.get("Name", "")
                        val = param.get("Value", 0.0) if "Value" in param else 0.0

                        # 判断是控制器数据还是手套数据
                        if len(name) >= 2 and name[1] == '_' and (name[0] == 'l' or name[0] == 'r'):
                            controller_data["controllerDatas"][name] = val
                        else:
                            glove_data["handDatas"][name] = val

                    self.glove_data_list.append(glove_data)
                    self.controller_data_list.append(controller_data)

                # 解析关节数据
                self._parse_finger_data()

        except Exception as e:
            print(f"[GloveSDK] 处理数据错误: {e}")

    def _parse_finger_data(self):
        """解析手指关节数据为 Vector3Float 列表"""
        # 解析左手数据
        left_data = {}
        for glove in self.glove_data_list:
            left_data.update(glove["handDatas"])

        # 左手 15 个关节 (每个关节 3 个分量)
        self.left_finger_data = [
            Vector3Float(left_data.get("l2", 0), left_data.get("l3", 0), left_data.get("l20", 0)),   # Thumb1
            Vector3Float(left_data.get("l1", 0), 0, 0),                                                    # Thumb2
            Vector3Float(left_data.get("l0", 0), 0, 0),                                                    # Thumb3
            Vector3Float(left_data.get("l6", 0), left_data.get("l7", 0), left_data.get("l21", 0)),    # Index1
            Vector3Float(left_data.get("l5", 0), 0, 0),                                                    # Index2
            Vector3Float(left_data.get("l4", 0), 0, 0),                                                    # Index3
            Vector3Float(left_data.get("l10", 0), left_data.get("l11", 0), 0),                       # Middle1
            Vector3Float(left_data.get("l9", 0), 0, 0),                                                     # Middle2
            Vector3Float(left_data.get("l8", 0), 0, 0),                                                     # Middle3
            Vector3Float(left_data.get("l14", 0), left_data.get("l15", 0), 0),                       # Ring1
            Vector3Float(left_data.get("l13", 0), 0, 0),                                                    # Ring2
            Vector3Float(left_data.get("l12", 0), 0, 0),                                                    # Ring3
            Vector3Float(left_data.get("l18", 0), left_data.get("l19", 0), left_data.get("l22", 0)), # Pinky1
            Vector3Float(left_data.get("l17", 0), 0, 0),                                                   # Pinky2
            Vector3Float(left_data.get("l16", 0), 0, 0),                                                   # Pinky3
        ]

        # 解析右手数据
        right_data = {}
        for glove in self.glove_data_list:
            right_data.update(glove["handDatas"])

        self.right_finger_data = [
            Vector3Float(right_data.get("r2", 0), right_data.get("r3", 0), right_data.get("r20", 0)),
            Vector3Float(right_data.get("r1", 0), 0, 0),
            Vector3Float(right_data.get("r0", 0), 0, 0),
            Vector3Float(right_data.get("r6", 0), right_data.get("r7", 0), right_data.get("r21", 0)),
            Vector3Float(right_data.get("r5", 0), 0, 0),
            Vector3Float(right_data.get("r4", 0), 0, 0),
            Vector3Float(right_data.get("r10", 0), right_data.get("r11", 0), 0),
            Vector3Float(right_data.get("r9", 0), 0, 0),
            Vector3Float(right_data.get("r8", 0), 0, 0),
            Vector3Float(right_data.get("r14", 0), right_data.get("r15", 0), 0),
            Vector3Float(right_data.get("r13", 0), 0, 0),
            Vector3Float(right_data.get("r12", 0), 0, 0),
            Vector3Float(right_data.get("r18", 0), right_data.get("r19", 0), right_data.get("r22", 0)),
            Vector3Float(right_data.get("r17", 0), 0, 0),
            Vector3Float(right_data.get("r16", 0), 0, 0),
        ]

        # 解析控制器数据
        for controller in self.controller_data_list:
            ctrl_data = controller["controllerDatas"]
            if controller["roleName"].startswith("left") or "l_" in str(ctrl_data):
                self.left_controller = [
                    ctrl_data.get("l_joyX", 0.0),
                    ctrl_data.get("l_joyY", 0.0),
                    ctrl_data.get("l_aButton", 0.0),
                    ctrl_data.get("l_bButton", 0.0),
                    ctrl_data.get("l_joyButton", 0.0),
                    ctrl_data.get("l_menu", 0.0),
                ]
            else:
                self.right_controller = [
                    ctrl_data.get("r_joyX", 0.0),
                    ctrl_data.get("r_joyY", 0.0),
                    ctrl_data.get("r_aButton", 0.0),
                    ctrl_data.get("r_bButton", 0.0),
                    ctrl_data.get("r_joyButton", 0.0),
                    ctrl_data.get("r_menu", 0.0),
                ]

    def get_finger_data(self, hand: str = "right") -> List[Vector3Float]:
        """
        获取手指关节数据

        Args:
            hand: "left" 或 "right"

        Returns:
            15 个 Vector3Float 的列表，分别对应:
            [Thumb1, Thumb2, Thumb3, Index1, Index2, Index3,
             Middle1, Middle2, Middle3, Ring1, Ring2, Ring3,
             Pinky1, Pinky2, Pinky3]
        """
        with self.data_lock:
            if hand == "left":
                return self.left_finger_data.copy()
            else:
                return self.right_finger_data.copy()

    def get_controller_data(self, hand: str = "right") -> List[float]:
        """获取控制器数据"""
        with self.data_lock:
            if hand == "left":
                return self.left_controller.copy()
            else:
                return self.right_controller.copy()

    def get_role_name_list(self) -> List[str]:
        """获取角色名称列表"""
        return [glove["roleName"] for glove in self.glove_data_list]


# ============================================================================
# 关节映射器 - 手套数据映射到 OrcaHand 关节
# ============================================================================

def load_glove_mapping_config(model_path: str, hand: str = "right") -> Dict:
    """
    加载手套映射配置文件

    Args:
        model_path: OrcaHand 模型路径
        hand: "left" 或 "right"

    Returns:
        配置字典 {joint_name: {glove_var, component, range_min, range_max, neutral}}
    """
    import yaml

    mapping_file = os.path.join(model_path, "glove_mapping.yaml")

    if not os.path.exists(mapping_file):
        print(f"[警告] 未找到手套映射配置文件: {mapping_file}")
        return {}

    with open(mapping_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    mapping_key = f"{hand}_glove_mapping"
    return config.get(mapping_key, {})


def load_full_glove_config(model_path: str) -> Dict:
    """
    加载手套映射配置文件，统一返回扁平格式:
    {global_config, left_glove_mapping, right_glove_mapping}

    自动兼容两种 yaml 写法:
      - 扁平格式(右手模型): 顶层有 global_config / left_glove_mapping / right_glove_mapping
      - 紧凑格式(左手模型): 顶层有 global / left / right,
        每个关节写成 "finger: joint: [glove_var, component, range_min, range_max, factor, direction, invert]"

    Args:
        model_path: OrcaHand 模型路径

    Returns:
        统一格式的配置字典
    """
    import yaml

    mapping_file = os.path.join(model_path, "glove_mapping.yaml")
    if not os.path.exists(mapping_file):
        print(f"[警告] 未找到手套映射配置文件: {mapping_file}")
        return {"global_config": {}, "left_glove_mapping": {}, "right_glove_mapping": {}}

    with open(mapping_file, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}

    # 已是扁平格式 -> 直接按结构返回
    if {"global_config", "left_glove_mapping", "right_glove_mapping"} & set(raw.keys()):
        return {
            "global_config": raw.get("global_config", {}),
            "left_glove_mapping": raw.get("left_glove_mapping", {}),
            "right_glove_mapping": raw.get("right_glove_mapping", {}),
        }

    # 紧凑格式 -> 展开成扁平结构
    result = {
        "global_config": raw.get("global", {}),
        "left_glove_mapping": {},
        "right_glove_mapping": {},
    }
    for hand in ("left", "right"):
        hand_cfg = raw.get(hand, {}) or {}
        flat = {}
        for finger, joints in hand_cfg.items():
            for joint_type, vals in joints.items():
                joint_name = f"{finger}_{joint_type}"
                vals = list(vals)
                flat[joint_name] = {
                    "glove_var": vals[0],
                    "component": vals[1],
                    "range_min": vals[2],
                    "range_max": vals[3],
                    "factor": vals[4],
                    "direction": vals[5] if len(vals) > 5 else "inward",
                    "invert": bool(vals[6]) if len(vals) > 6 else False,
                    "neutral": vals[7] if len(vals) > 7 else 0,
                }
        result[f"{hand}_glove_mapping"] = flat

    return result


class GloveToHandMapper:
    """
    将手套关节数据映射到 OrcaHand 关节

    OrcaHand 有 17 个关节:
    - thumb: thumb_mcp, thumb_abd, thumb_pip, thumb_dip
    - index: index_abd, index_mcp, index_pip
    - middle: middle_abd, middle_mcp, middle_pip
    - ring: ring_abd, ring_mcp, ring_pip
    - pinky: pinky_abd, pinky_mcp, pinky_pip
    - wrist: wrist
    """

    def __init__(self, orca_hand: OrcaHand, hand: str = "right"):
        self.orca_hand = orca_hand
        self.hand = hand
        self.joint_ids = orca_hand.joint_ids
        self.joint_roms = orca_hand.joint_roms_dict
        self.neutral_position = orca_hand.neutral_position
        self.model_path = orca_hand.model_path

        # 加载手套映射配置
        full_config = load_full_glove_config(self.model_path)
        self.glove_mapping = full_config.get(f"{hand}_glove_mapping", {})
        self.global_config = full_config.get("global_config", {})
        self.motion_scale = self.global_config.get("motion_scale", 0.8)

        # 诊断信息: 每帧记录 {joint: {raw, normalized, angle, ...}}, 供控制器周期性打印
        self.last_diagnostics: Dict[str, Dict] = {}

        if not self.glove_mapping:
            print(f"[警告] 使用默认手套映射配置")

        # 手套关节索引 (每指 3 个关节)
        # [Thumb1, Thumb2, Thumb3, Index1, Index2, Index3,
        #  Middle1, Middle2, Middle3, Ring1, Ring2, Ring3,
        #  Pinky1, Pinky2, Pinky3]
        self.GLOVE_INDICES = {
            'thumb': (0, 1, 2),
            'index': (3, 4, 5),
            'middle': (6, 7, 8),
            'ring': (9, 10, 11),
            'pinky': (12, 13, 14),
        }

        # 手套关节到 OrcaHand 关节的映射 (顺序很重要!)
        # 注意：这个顺序必须与 glove_mapping.yaml 中的顺序一致
        self.JOINT_MAPPING = {
            'thumb': ['thumb_abd', 'thumb_mcp', 'thumb_pip', 'thumb_dip'],
            'index': ['index_abd', 'index_mcp', 'index_pip'],
            'middle': ['middle_abd', 'middle_mcp', 'middle_pip'],
            'ring': ['ring_abd', 'ring_mcp', 'ring_pip'],
            'pinky': ['pinky_abd', 'pinky_mcp', 'pinky_pip'],
        }

    def map_glove_to_hand(
        self,
        glove_data: List[Vector3Float],
        motion_scale: Optional[float] = None,
        smoothing_factor: float = 0.3
    ) -> Dict[str, float]:
        """
        将手套数据映射到 OrcaHand 关节角度

        使用 glove_mapping.yaml 配置文件中的量程进行正确的归一化

        Args:
            glove_data: 15 个 Vector3Float 的列表
            motion_scale: 运动缩放因子 (可选，默认使用配置文件中的值)
            smoothing_factor: 平滑系数 (0.0-1.0)

        Returns:
            {joint_name: angle_in_degrees}
        """
        # 如果没有提供 motion_scale，使用配置文件中的值
        if motion_scale is None:
            motion_scale = self.motion_scale
        joint_angles = {}
        diagnostics = {}  # 本帧每个关节的 raw→normalized→angle 诊断数据

        # 获取手套原始数据字典（用于直接查找变量）
        glove_dict = {}
        prefix = 'l' if self.hand == 'left' else 'r'
        for i in range(23):
            var_name = f"{prefix}{i}"
            # 从 glove_data 中提取 - 需要根据解析逻辑重建
        # 由于 glove_data 是 Vector3Float 列表，我们需要直接使用索引

        # 使用新的映射方法：直接遍历配置中的每个关节
        for finger, orca_joints in self.JOINT_MAPPING.items():
            glove_indices = self.GLOVE_INDICES.get(finger, [])

            for i, orca_joint in enumerate(orca_joints):
                if orca_joint not in self.joint_roms:
                    continue

                rom = self.joint_roms[orca_joint]
                rom_min, rom_max = rom
                neutral = self.neutral_position.get(orca_joint, 0)

                # 获取该关节的配置
                joint_config = self.glove_mapping.get(orca_joint, {})

                if joint_config:
                    # 使用配置文件
                    glove_var = joint_config.get('glove_var', '')
                    component = joint_config.get('component', 'x')
                    range_min = joint_config.get('range_min', -100)
                    range_max = joint_config.get('range_max', 0)
                    config_neutral = joint_config.get('neutral', 0)
                    factor = joint_config.get('factor', 1.0)
                    direction = joint_config.get('direction', 'inward')

                    # 从 glove_data 中找到对应的手套变量值
                    glove_value = self._get_glove_value(glove_data, glove_var, component)
                    raw_glove_value = glove_value                      # 原始值(诊断/探测用)
                    glove_value = glove_value - config_neutral         # 中性偏移: 手套静止位(config_neutral)对齐 normalized=0

                    # 根据方向类型归一化
                    if component == 'y':
                        # y轴外展
                        if direction == 'bidirectional':
                            # 双向范围
                            if glove_value > 0:
                                normalized = glove_value / range_max      # [0, 1]
                            else:
                                normalized = glove_value / abs(range_min)  # [-1, 0]
                        elif direction == 'inward':
                            # 向内/向右（负值范围）
                            if glove_value < 0:
                                normalized = glove_value / abs(range_min)  # [-1, 0]
                            else:
                                normalized = 0
                        elif direction == 'outward':
                            # 向外/向左（正值范围）
                            if glove_value > 0:
                                normalized = glove_value / range_max      # [0, 1]
                            else:
                                normalized = 0
                        else:
                            normalized = 0
                    else:
                        # x轴弯曲
                        if direction == 'bidirectional':
                            # 双向范围
                            if glove_value > 0:
                                normalized = glove_value / range_max
                            else:
                                normalized = abs(glove_value) / abs(range_min)
                        elif direction == 'inward':
                            # 向内弯曲（负值范围）: 0到-60 → [0,1]
                            normalized = abs(glove_value) / abs(range_min)
                        elif direction == 'outward':
                            # 向外伸展（正值范围）
                            if glove_value > 0:
                                normalized = glove_value / range_max
                            else:
                                normalized = 0
                        else:
                            normalized = 0

                else:
                    # 使用默认逻辑（兼容旧代码）
                    direction = 'inward'  # 默认方向
                    factor = 1.0  # 默认 factor

                    if i >= len(glove_indices):
                        continue
                    glove_idx = glove_indices[i]
                    if glove_idx >= len(glove_data):
                        continue

                    glove_joint = glove_data[glove_idx]
                    if orca_joint.endswith('_abd'):
                        glove_value = glove_joint.y
                    else:
                        glove_value = glove_joint.x
                    raw_glove_value = glove_value

                    # 默认归一化: 假设范围是 -100 到 0
                    normalized = abs(glove_value) / 100.0

                # 反转标志(紧凑格式 glove_mapping.yaml 的第7个字段;扁平格式/默认没有则为 False)
                if joint_config.get('invert', False):
                    normalized = -normalized

                # 根据关节类型映射到 OrcaHand 角度
                # 使用配置中的 factor
                if orca_joint.endswith('_abd'):
                    # 外展关节: 双向运动，使用配置的 factor
                    if 'factor' in joint_config:
                        factor = joint_config['factor']
                    else:
                        factor = 1.0
                    # ABD 关节: angle 增大 = 向左，angle 减小 = 向右
                    # 根据 normalized 符号使用对应方向的范围
                    if normalized < 0:
                        # 向右运动：用 neutral 到 rom_min 的距离
                        range_to_use = neutral - rom_min
                        angle = neutral + normalized * range_to_use * factor * motion_scale
                    else:
                        # 向左运动：用 rom_max 到 neutral 的距离
                        range_to_use = rom_max - neutral
                        angle = neutral + normalized * range_to_use * factor * motion_scale

                else:
                    # 弯曲关节: 向内弯曲，使用配置的 factor
                    if 'factor' in joint_config:
                        factor = joint_config['factor']
                    else:
                        factor = 1.0
                    # 手套数据负值表示向内弯曲，但关节控制正值表示向内弯曲
                    # 所以从 neutral 向 rom_max 方向运动（正值方向）
                    angle = neutral + normalized * (rom_max - neutral) * factor * motion_scale
                    range_to_use = rom_max - neutral

                # 限制在 ROM 范围内
                angle = max(rom_min, min(rom_max, angle))
                joint_angles[orca_joint] = float(angle)

                # 收集诊断信息(raw→normalized→angle), 供控制器周期性打印。
                # 始终记录(开销很小), 是否打印由 TeleoperationController 决定。
                diagnostics[orca_joint] = {
                    'raw': raw_glove_value,
                    'normalized': normalized,
                    'neutral': neutral,
                    'range_to_use': range_to_use,
                    'factor': factor,
                    'rom': (rom_min, rom_max),
                    'angle': angle,
                }

        # 手腕 - 默认在中性位置
        if 'wrist' in self.joint_roms:
            joint_angles['wrist'] = self.neutral_position.get('wrist', 0)

        self.last_diagnostics = diagnostics
        return joint_angles

    def _get_glove_value(self, glove_data: List[Vector3Float], glove_var: str, component: str) -> float:
        """
        从 glove_data 列表中获取指定变量的指定分量值

        Args:
            glove_data: 15 个 Vector3Float 的列表
            glove_var: 手套变量名 (如 "r0", "r1", etc.)
            component: 分量名 ("x", "y", "z")

        Returns:
            该变量的分量值
        """
        # 根据 _parse_finger_data 中的映射逻辑
        # 需要知道每个手套变量对应 glove_data 的哪个索引和哪个分量

        prefix = glove_var[0]  # 'l' 或 'r'
        num = int(glove_var[1:])  # 变量编号

        # 定义从变量编号到 glove_data 索引和分量的映射
        # 基于 _parse_finger_data 中的逻辑
        var_mapping = {
            # 拇指
            0: (2, 'x'),   # Thumb3.x = l0/r0
            1: (1, 'x'),   # Thumb2.x = l1/r1
            2: (0, 'x'),   # Thumb1.x = l2/r2
            3: (0, 'y'),   # Thumb1.y = l3/r3
            20: (0, 'z'),  # Thumb1.z = l20/r20
            # 食指
            4: (5, 'x'),   # Index3.x = l4/r4
            5: (4, 'x'),   # Index2.x = l5/r5
            6: (3, 'x'),   # Index1.x = l6/r6
            7: (3, 'y'),   # Index1.y = l7/r7
            21: (3, 'z'),  # Index1.z = l21/r21
            # 中指
            8: (8, 'x'),   # Middle3.x = l8/r8
            9: (7, 'x'),   # Middle2.x = l9/r9
            10: (6, 'x'),  # Middle1.x = l10/r10
            11: (6, 'y'),  # Middle1.y = l11/r11
            # 无名指
            12: (11, 'x'), # Ring3.x = l12/r12
            13: (10, 'x'), # Ring2.x = l13/r13
            14: (9, 'x'),  # Ring1.x = l14/r14
            15: (9, 'y'),  # Ring1.y = l15/r15
            # 小指
            16: (14, 'x'), # Pinky3.x = l16/r16
            17: (13, 'x'), # Pinky2.x = l17/r17
            18: (12, 'x'), # Pinky1.x = l18/r18
            19: (12, 'y'), # Pinky1.y = l19/r19
            22: (12, 'z'), # Pinky1.z = l22/r22
        }

        if num not in var_mapping:
            return 0.0

        idx, comp = var_mapping[num]
        if idx >= len(glove_data):
            return 0.0

        vec = glove_data[idx]
        if comp == 'x':
            return vec.x
        elif comp == 'y':
            return vec.y
        elif comp == 'z':
            return vec.z

        return 0.0


# ============================================================================
# 平滑控制器
# ============================================================================

class SmoothController:
    """指数平滑控制器"""

    def __init__(self, smoothing_factor: float = 0.3):
        """
        Args:
            smoothing_factor: 平滑系数 (0.0-1.0)
                             0.0 = 不平滑, 1.0 = 完全平滑
        """
        self.smoothing_factor = smoothing_factor
        self.last_angles: Dict[str, float] = {}

    def smooth(self, target_angles: Dict[str, float]) -> Dict[str, float]:
        """应用指数平滑"""
        smoothed = {}
        for joint, target in target_angles.items():
            last = self.last_angles.get(joint, target)
            smoothed[joint] = (self.smoothing_factor * target +
                             (1 - self.smoothing_factor) * last)
            self.last_angles[joint] = smoothed[joint]
        return smoothed

    def reset(self):
        """重置平滑状态"""
        self.last_angles.clear()


# ============================================================================
# 遥操作控制器
# ============================================================================

class TeleoperationController:
    """
    遥操作主控制器

    支持单手或双手遥操作模式
    """

    def __init__(
        self,
        left_hand_model_path: Optional[str] = None,
        right_hand_model_path: Optional[str] = None,
        glove_port: int = 8000,
        update_rate: int = 60
    ):
        """
        Args:
            left_hand_model_path: 左手模型路径
            right_hand_model_path: 右手模型路径
            glove_port: 手套 UDP 端口
            update_rate: 控制更新频率 (Hz)
        """
        # 初始化手套 SDK
        self.glove_sdk = UDEGloveSDK(port=glove_port)

        # 初始化灵巧手
        self.left_hand: Optional[OrcaHand] = None
        self.right_hand: Optional[OrcaHand] = None
        self.left_mapper: Optional[GloveToHandMapper] = None
        self.right_mapper: Optional[GloveToHandMapper] = None

        if left_hand_model_path:
            self.left_hand = OrcaHand(model_path=left_hand_model_path)
            self.left_mapper = GloveToHandMapper(self.left_hand, hand="left")
            print(f"[Teleop] 左手已加载: {left_hand_model_path}")

        if right_hand_model_path:
            self.right_hand = OrcaHand(model_path=right_hand_model_path)
            self.right_mapper = GloveToHandMapper(self.right_hand, hand="right")
            print(f"[Teleop] 右手已加载: {right_hand_model_path}")

        # 平滑控制器
        self.left_smoother = SmoothController(smoothing_factor=0.3)
        self.right_smoother = SmoothController(smoothing_factor=0.3)

        # 控制参数 (motion_scale 从 mapper 中获取)
        self.motion_scale = 0.8  # 默认值，会在 initialize 时从 mapper 更新
        self.update_rate = update_rate
        self.running = False

        # 统计信息
        self.update_count = 0
        self.last_update_time = 0

        # 运行时调参与诊断
        self.debug = False                  # 是否周期打印每个关节的 raw→normalized→angle
        self._cmd_thread: Optional[threading.Thread] = None
        self._probe_active = False          # 是否正在记录手套量程
        self._probe_data: Dict[str, List[float]] = {}  # {"hand:joint": [min, max]}

    def initialize(self) -> bool:
        """初始化所有组件"""
        print("[Teleop] 正在初始化遥操作系统...")

        # 初始化手套 SDK
        if not self.glove_sdk.initialize():
            print("[Teleop] 手套 SDK 初始化失败")
            return False

        # 连接左手
        if self.left_hand:
            success, msg = self.left_hand.connect()
            if not success:
                print(f"[Teleop] 左手连接失败: {msg}")
                return False
            print(f"[Teleop] 左手已连接")
            self.left_hand.enable_torque()
            self.left_hand.set_control_mode('current_based_position')

        # 连接右手
        if self.right_hand:
            success, msg = self.right_hand.connect()
            if not success:
                print(f"[Teleop] 右手连接失败: {msg}")
                return False
            print(f"[Teleop] 右手已连接")
            self.right_hand.enable_torque()
            self.right_hand.set_control_mode('current_based_position')

        # 开始监听手套数据
        self.glove_sdk.start_listening()

        # 从 mapper 中读取 motion_scale
        if self.right_mapper and hasattr(self.right_mapper, 'motion_scale'):
            self.motion_scale = self.right_mapper.motion_scale
        elif self.left_mapper and hasattr(self.left_mapper, 'motion_scale'):
            self.motion_scale = self.left_mapper.motion_scale

        # 等待第一包数据
        print("[Teleop] 等待手套数据...")
        for i in range(50):  # 等待 5 秒
            time.sleep(0.1)
            if self.glove_sdk.is_connected:
                print("[Teleop] 手套已连接!")
                break
        else:
            print("[Teleop] 警告: 未检测到手套数据，将使用默认值")

        print("[Teleop] 初始化完成")
        return True

    def start(self):
        """启动遥操作循环"""
        self.running = True
        self.last_update_time = time.time()

        # 启动运行时命令线程(不重启即可调参)
        self._start_command_loop()

        print("[Teleop] 遥操作已启动")
        print("[Teleop] 按 Ctrl+C 停止")
        print(f"[Teleop] 控制频率: {self.update_rate} Hz")
        print(f"[Teleop] 运动缩放: {self.motion_scale * 100:.0f}%")

        try:
            while self.running:
                self._update()

                # 量程探测: 每帧记录手套变量的 min/max(仅在探测开启时)
                if self._probe_active:
                    self._probe_update()

                self.update_count += 1

                # 控制频率
                elapsed = time.time() - self.last_update_time
                target_sleep = 1.0 / self.update_rate - elapsed
                if target_sleep > 0:
                    time.sleep(target_sleep)
                self.last_update_time = time.time()

                # 定期输出状态
                if self.update_count % 600 == 0:  # 每 10 秒
                    self._print_status()

                # 诊断自动打印(约每2秒一次); 也可随时输入 p 打印一次快照
                if self.debug and self.update_count % (self.update_rate * 2) == 0:
                    self._print_diagnostics()

        except KeyboardInterrupt:
            print("\n[Teleop] 收到停止信号")
        finally:
            self.stop()

    def stop(self):
        """停止遥操作并清理"""
        print("[Teleop] 正在停止...")

        self.running = False

        # 停止手套监听
        self.glove_sdk.end_listening()

        # 移动到中性位置
        if self.left_hand:
            try:
                self.left_hand.set_neutral_position(num_steps=50)
                self.left_hand.disable_torque()
                self.left_hand.disconnect()
                print("[Teleop] 左手已停止")
            except Exception as e:
                print(f"[Teleop] 左手停止错误: {e}")

        if self.right_hand:
            try:
                self.right_hand.set_neutral_position(num_steps=50)
                self.right_hand.disable_torque()
                self.right_hand.disconnect()
                print("[Teleop] 右手已停止")
            except Exception as e:
                print(f"[Teleop] 右手停止错误: {e}")

        print(f"[Teleop] 总更新次数: {self.update_count}")
        print("[Teleop] 已停止")

    def _filter_calibrated(self, hand, joint_angles):
        """剔除未标定关节（肌腱断 / --skip 过的），不驱动、也不刷警告。"""
        if not hand:
            return joint_angles
        if not hasattr(self, '_skip_logged'):
            self._skip_logged = set()
        out = {}
        for joint, angle in joint_angles.items():
            motor_id = hand.joint_to_motor_map.get(joint)
            limits = hand.motor_limits_dict.get(motor_id) if motor_id is not None else None
            if limits is None or any(l is None for l in limits):
                if joint not in self._skip_logged:
                    print(f"[Teleop] 跳过未标定关节 {joint}（motor {motor_id}）— 不驱动（肌腱断/已--skip）")
                    self._skip_logged.add(joint)
                continue
            out[joint] = angle
        return out

    def _update(self):
        """更新控制循环"""
        # 更新左手
        if self.left_hand and self.left_mapper:
            glove_data = self.glove_sdk.get_finger_data("left")
            target_angles = self.left_mapper.map_glove_to_hand(glove_data)
            smoothed_angles = self.left_smoother.smooth(target_angles)
            smoothed_angles = self._filter_calibrated(self.left_hand, smoothed_angles)
            self.left_hand.set_joint_pos(smoothed_angles, num_steps=1)

        # 更新右手
        if self.right_hand and self.right_mapper:
            glove_data = self.glove_sdk.get_finger_data("right")
            target_angles = self.right_mapper.map_glove_to_hand(glove_data)
            smoothed_angles = self.right_smoother.smooth(target_angles)
            smoothed_angles = self._filter_calibrated(self.right_hand, smoothed_angles)
            self.right_hand.set_joint_pos(smoothed_angles, num_steps=1)

    def _print_status(self):
        """打印状态信息"""
        connected = self.glove_sdk.is_connected
        packets = self.glove_sdk.packet_count

        status = f"[Teleop] 状态: "
        status += f"连接={'是' if connected else '否'}, "
        status += f"数据包={packets}, "
        status += f"更新={self.update_count}"

        # 打印右手拇指 MCP 角度作为参考
        if self.right_hand:
            try:
                current_pos = self.right_hand.get_joint_pos(as_list=False)
                thumb_mcp = current_pos.get('thumb_mcp', 0)
                status += f", 拇指MCP={thumb_mcp:.1f}°"
            except:
                pass

        print(status)

    def set_motion_scale(self, scale: float):
        """设置运动缩放因子"""
        scale = max(0.0, min(1.0, scale))
        # 同时更新 mapper 中的 motion_scale
        if self.left_mapper:
            self.left_mapper.motion_scale = scale
        if self.right_mapper:
            self.right_mapper.motion_scale = scale
        print(f"[Teleop] 运动缩放已设置为 {scale * 100:.0f}%")

    def set_smoothing(self, factor: float):
        """设置平滑系数

        注意: 这里 1.0 = 完全跟手(不平滑), 0.0 = 完全冻结。
        觉得"慢半拍/不跟手"就调大; 觉得"抖动"就调小。
        """
        factor = max(0.0, min(1.0, factor))
        self.left_smoother.smoothing_factor = factor
        self.right_smoother.smoothing_factor = factor
        print(f"[Teleop] 平滑系数已设置为 {factor}  (越大越跟手, 越小越稳)")

    def set_joint_factor(self, joint: str, factor: float):
        """运行时修改某个关节的 factor(立即生效, 不写回 yaml)。

        用于单独加强某根手指的弯曲幅度, 例如 set_joint_factor('index_mcp', 1.5)。
        """
        factor = max(0.0, min(3.0, factor))
        hit = False
        for mapper in (self.left_mapper, self.right_mapper):
            if mapper and joint in mapper.glove_mapping:
                mapper.glove_mapping[joint]['factor'] = factor
                hit = True
        if hit:
            print(f"[Teleop] {joint} factor = {factor}  (仅本次运行有效, 重启失效)")
        else:
            print(f"[Teleop] 未找到关节 {joint} (可选: thumb_mcp/thumb_pip/thumb_dip/"
                  f"index_abd/index_mcp/index_pip/middle_*/ring_*/pinky_*)")

    # ------------------------------------------------------------------
    # 诊断: 周期打印每个关节的 raw→normalized→angle, 帮助定位"不动/动太小"
    # ------------------------------------------------------------------
    @staticmethod
    def _bar(pct: float, width: int = 10) -> str:
        pct = max(0.0, min(100.0, pct))
        filled = int(round(pct / 100 * width))
        return "#" * filled + "-" * (width - filled)

    def _print_diagnostics(self):
        """打印一次诊断快照(带进度条)。供 p 命令或慢速自动打印调用。"""
        mapper = self.right_mapper or self.left_mapper
        if not mapper or not mapper.last_diagnostics:
            print("[Teleop] 暂无诊断数据(请稍等一帧)")
            return
        hand_label = "右" if self.right_mapper else "左"
        print("\n" + "=" * 60)
        print(f" 诊断快照({hand_label}手)  #=已弯曲  位置=占ROM(0=伸直,100=满弯)")
        print("-" * 60)
        print(f" {'关节':<13}{'raw':>8}{'norm':>7}  {'弯曲程度':<14}{'角度':>7}")
        print("-" * 60)
        for joint, d in mapper.last_diagnostics.items():
            rom_lo, rom_hi = d['rom']
            pos_pct = (d['angle'] - rom_lo) / (rom_hi - rom_lo) * 100 if rom_hi != rom_lo else 0
            bar = self._bar(pos_pct)
            print(f" {joint:<13}{d['raw']:>8.1f}{d['normalized']:>7.2f}  "
                  f"{bar} {pos_pct:>3.0f}%{d['angle']:>7.1f}")
        print("=" * 60)
        print(" 解读: raw≈0且不动 = 映射错/无手套数据 | norm小 = range_min太大(用c探测) | "
              "位置%低 = 用s/f放大")

    # ------------------------------------------------------------------
    # 量程探测: 戴着手套做伸直↔握紧, 自动给出每个关节建议的 range_min/range_max
    # ------------------------------------------------------------------
    def _probe_toggle(self):
        if not self._probe_active:
            self._probe_active = True
            self._probe_data = {}
            print("[Teleop] 量程探测 已开始。")
            print("  请戴好手套, 缓慢且充分地重复: 五指完全伸直 → 用力握紧, 做 2~3 次。")
            print("  完成后再次输入 c 结束, 将打印每个关节的建议量程。")
        else:
            self._probe_active = False
            self._probe_report()

    def _probe_update(self):
        """每帧调用, 记录每个关节手套变量的实测 min/max"""
        for mapper, hand in ((self.left_mapper, 'left'), (self.right_mapper, 'right')):
            if not mapper:
                continue
            glove_data = self.glove_sdk.get_finger_data(hand)
            for joint, cfg in mapper.glove_mapping.items():
                glove_var = cfg.get('glove_var', '')
                comp = cfg.get('component', 'x')
                val = mapper._get_glove_value(glove_data, glove_var, comp)
                key = f"{hand}:{joint}"
                if key not in self._probe_data:
                    self._probe_data[key] = [val, val]
                else:
                    lo, hi = self._probe_data[key]
                    self._probe_data[key] = [min(lo, val), max(hi, val)]

    def _probe_report(self):
        if not self._probe_data:
            print("[Teleop] 没有探测到数据(是否没收到手套包?)。")
            return
        print("\n" + "=" * 82)
        print(f" {'关节':<20}{'实测[min,max]':>18}{'当前range':>16}{'建议 range_min/max':>22}")
        print("-" * 82)
        for mapper, hand in ((self.left_mapper, 'left'), (self.right_mapper, 'right')):
            if not mapper:
                continue
            label = "左" if hand == 'left' else "右"
            for joint, cfg in mapper.glove_mapping.items():
                key = f"{hand}:{joint}"
                if key not in self._probe_data:
                    continue
                lo, hi = self._probe_data[key]
                cur_min = cfg.get('range_min', 0)
                cur_max = cfg.get('range_max', 0)
                direction = cfg.get('direction', 'inward')
                # 根据 direction 给出建议的归一化量程边界
                if direction == 'inward':       # 负值范围, 弯曲
                    sug_min, sug_max = lo, 0
                elif direction == 'outward':    # 正值范围
                    sug_min, sug_max = 0, hi
                else:                           # bidirectional
                    sug_min, sug_max = lo, hi
                measured = f"[{lo:.0f},{hi:.0f}]"
                current = f"[{cur_min:.0f},{cur_max:.0f}]"
                suggest = f"min={sug_min:.0f}, max={sug_max:.0f}"
                flag = ""
                if direction == 'inward' and lo >= 0:
                    flag = "  ⚠ 实测非负, 可能 direction/glove_var 配反"
                print(f" {label}:{joint:<18}{measured:>18}{current:>16}{suggest:>22}{flag}")
        print("=" * 82)
        print(" 把上面【建议】列的值填回 glove_mapping.yaml 对应关节的 range_min/range_max,")
        print(" 重启后弯曲幅度即可拉满。当前也可用 s1.0 临时放大整体幅度验证。")

    # ------------------------------------------------------------------
    # 运行时命令线程: 不重启即可调参
    # ------------------------------------------------------------------
    def _start_command_loop(self):
        self._cmd_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._cmd_thread.start()

    def _command_loop(self):
        print("[Teleop] 运行时命令(输入后回车):")
        print("  s<数>      运动缩放, 如 s1.0   (0~1, 越大幅度越大)")
        print("  m<数>      平滑系数, 如 m0.6   (越大越跟手, 越小越稳)")
        print("  f<关节> <数>  单关节系数, 如 f index_mcp 1.5")
        print("  p          打印一次诊断快照(按需, 最直观)")
        print("  d          开/关 自动诊断打印(每2秒)")
        print("  c          开始/停止 量程探测")
        print("  n          回到中性位")
        print("  q          退出")
        while self.running:
            try:
                cmd = input().strip().lower()
            except EOFError:
                break
            if not cmd:
                continue
            try:
                self._handle_command(cmd)
            except Exception as e:
                print(f"[Teleop] 命令错误: {e}")

    def _handle_command(self, cmd: str):
        if cmd in ('q', 'quit', 'exit'):
            self.running = False
            print("[Teleop] 收到退出命令")
            return
        if cmd in ('h', 'help'):
            print("  s<数> / m<数> / f<关节> <数> / d=诊断 / c=量程 / n=归中 / q=退出")
            return
        if cmd == 'p':
            self._print_diagnostics()
            return
        if cmd == 'd':
            self.debug = not self.debug
            print(f"[Teleop] 自动诊断: {'开(每2秒)' if self.debug else '关'}  (随时可按 p 看一次快照)")
            return
        if cmd == 'n':
            print("[Teleop] 回中性位...")
            if self.right_hand:
                self.right_hand.set_neutral_position(num_steps=30)
            if self.left_hand:
                self.left_hand.set_neutral_position(num_steps=30)
            return
        if cmd == 'c':
            self._probe_toggle()
            return

        parts = cmd.replace(',', ' ').split()
        head = parts[0]
        if head in ('s', 'scale') and len(parts) >= 2:
            self.set_motion_scale(float(parts[1]))
        elif head in ('m', 'sm', 'smooth') and len(parts) >= 2:
            self.set_smoothing(float(parts[1]))
        elif head in ('f', 'factor') and len(parts) >= 3:
            self.set_joint_factor(parts[1], float(parts[2]))
        else:
            print(f"[Teleop] 未知命令: {cmd}  (输入 h 看帮助)")


# ============================================================================
# 辅助函数
# ============================================================================

def discover_models() -> List[Dict]:
    """发现可用的手部模型"""
    models_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "orca_core", "models"
    )

    available_models = []

    if not os.path.exists(models_dir):
        return available_models

    for model_name in os.listdir(models_dir):
        model_path = os.path.join(models_dir, model_name)
        config_path = os.path.join(model_path, "config.yaml")

        if os.path.isdir(model_path) and os.path.exists(config_path):
            # 读取配置
            try:
                import yaml
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)
                    hand_type = config.get('type', 'unknown')
            except:
                hand_type = 'unknown'

            available_models.append({
                'name': model_name,
                'path': model_path,
                'type': hand_type
            })

    return available_models


def print_banner():
    """打印横幅"""
    print("=" * 60)
    print(" OrcaHand 遥操作系统")
    print(" Teleoperation with Data Gloves")
    print("=" * 60)


# ============================================================================
# 主函数
# ============================================================================

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='OrcaHand 遥操作系统')
    parser.add_argument(
        '--left-hand', '-l',
        type=str,
        default=None,
        help='左手模型路径'
    )
    parser.add_argument(
        '--right-hand', '-r',
        type=str,
        default=None,
        help='右手模型路径 (默认: orca_core/models/orcahand_v1_right)'
    )
    parser.add_argument(
        '--glove-port', '-p',
        type=int,
        default=8000,
        help='手套 UDP 端口 (默认: 8000)'
    )
    parser.add_argument(
        '--rate',
        type=int,
        default=60,
        help='控制更新频率 Hz (默认: 60)'
    )
    parser.add_argument(
        '--scale', '-s',
        type=float,
        default=0.8,
        help='运动缩放因子 (默认: 0.8)'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='启动时即开启诊断打印(每秒打印各关节 raw→norm→angle)'
    )

    args = parser.parse_args()

    print_banner()

    # 确定要加载的手：指定了 --left-hand 就只跑左手；否则默认右手
    right_hand_path = args.right_hand
    if right_hand_path is None and args.left_hand is None:
        default_path = "orca_core/models/orcahand_v1_right"
        if os.path.exists(default_path):
            right_hand_path = default_path
            print(f"[信息] 使用默认右手模型: {default_path}")
        else:
            print(f"[错误] 默认右手模型不存在: {default_path}")
            print(f"[信息] 请使用 --right-hand 参数指定模型路径")
            return
    if args.left_hand:
        print(f"[信息] 仅加载左手模型: {args.left_hand}")

    # 创建控制器
    controller = TeleoperationController(
        left_hand_model_path=args.left_hand,
        right_hand_model_path=right_hand_path,
        glove_port=args.glove_port,
        update_rate=args.rate
    )

    # 设置运动缩放
    controller.set_motion_scale(args.scale)

    # 诊断打印(也可运行时用 d 命令切换)
    if args.debug:
        controller.debug = True
        print("[信息] 诊断打印已开启")

    # 初始化
    if not controller.initialize():
        print("[错误] 初始化失败")
        return

    # 启动遥操作
    controller.start()


if __name__ == "__main__":
    main()
