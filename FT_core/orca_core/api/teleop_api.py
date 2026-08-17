# ==============================================================================
# Copyright (c) 2025 ORCA
#
# OrcaHand 遥操作 API
# 通过 UDP 端口 8000 接收手套数据并实时控制 ORCA 灵巧手
# 使用实际关节角度（度）进行控制
# ==============================================================================

import os
import sys
import time
import socket
import json
from datetime import datetime
from fastapi import FastAPI, HTTPException, Body
from pydantic import BaseModel, Field
from typing import List, Dict, Optional, Union, Tuple
import numpy as np
import uvicorn
from threading import Lock, Thread
import yaml

# 添加父目录到导入路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand

app = FastAPI(title="OrcaHand 遥操作 API", version="2.0.0")

# --- 全局变量 ---
hand = None
command_lock = Lock()
glove_receiver = None
teleop_state = None
udp_control_thread = None


# =============================================================================
# 配置管理
# ==============================================================================

class HandConfig:
    """手部配置管理"""

    # ORCA Hand 17个关节的 ROM (度)
    JOINT_ROMS = {
        'thumb_mcp': [-50, 50],
        'thumb_abd': [-20, 42],
        'thumb_pip': [-12, 108],
        'thumb_dip': [-20, 112],
        'index_abd': [-37, 37],
        'index_mcp': [-20, 95],
        'index_pip': [-20, 108],
        'middle_abd': [-37, 37],
        'middle_mcp': [-20, 91],
        'middle_pip': [-20, 107],
        'ring_abd': [-37, 37],
        'ring_mcp': [-20, 91],
        'ring_pip': [-20, 107],
        'pinky_abd': [-37, 37],
        'pinky_mcp': [-20, 98],
        'pinky_pip': [-20, 108],
        'wrist': [-50, 30],
    }

    # 中性位置（默认姿态）
    NEUTRAL_POSITION = {
        'thumb_mcp': -13, 'thumb_abd': 43, 'thumb_pip': 33, 'thumb_dip': 19,
        'index_abd': 25, 'index_mcp': 0, 'index_pip': 0,
        'middle_abd': -2, 'middle_mcp': 0, 'middle_pip': 0,
        'ring_abd': -20, 'ring_mcp': -1, 'ring_pip': 0,
        'pinky_abd': -55, 'pinky_mcp': 1, 'pinky_pip': 0,
        'wrist': 0,
    }

    @staticmethod
    def get_model_path(hand_type: str) -> str:
        """获取模型路径"""
        # 目前只有右手模型，左手使用相同配置但镜像处理
        return "orca_core/models/orcahand_v1_right"

    @staticmethod
    def load_config_from_file(model_path: str) -> dict:
        """从配置文件加载"""
        config_path = os.path.join(model_path, "config.yaml")
        if os.path.exists(config_path):
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        return {}

    @staticmethod
    def load_roms_from_file(model_path: str) -> dict:
        """从配置文件加载 ROM"""
        config = HandConfig.load_config_from_file(model_path)
        return config.get('joint_roms', HandConfig.JOINT_ROMS)

    @staticmethod
    def load_neutral_from_file(model_path: str) -> dict:
        """从配置文件加载中性位置"""
        config = HandConfig.load_config_from_file(model_path)
        return config.get('neutral_position', HandConfig.NEUTRAL_POSITION)


# =============================================================================
# UDP 手套数据接收器
# ==============================================================================

class UDPGloveReceiver:
    """
    从 UDP 端口 8000 接收手套数据
    手套有 6 个传感器 (f1-f6)，需要映射到 ORCA Hand 的 17 个关节
    """

    # 手套传感器索引（与 glove_data_recorder.py 兼容）
    LEFT_FINGER_INDICES = {'f1': 18, 'f2': 14, 'f3': 10, 'f4': 6, 'f5': 1, 'f6': 20}
    RIGHT_FINGER_INDICES = {'f1': 18, 'f2': 14, 'f3': 10, 'f4': 6, 'f5': 1, 'f6': 20}

    # 手套原始值范围
    MIN_GLOVE_VALUES = [20.0, 20.0, 20.0, 20.0, 25.0, 0.0]
    MAX_GLOVE_VALUES = [60.0, 80.0, 80.0, 80.0, 35.0, 20.0]

    # 手套 6 传感器到 ORCA 17 关节的映射
    # f1=拇指, f2=食指, f3=中指, f4=无名指, f5=小指, f6=手腕
    GLOVE_TO_ORCA_MAP = {
        'f1': ['thumb_mcp', 'thumb_abd', 'thumb_pip', 'thumb_dip'],
        'f2': ['index_abd', 'index_mcp', 'index_pip'],
        'f3': ['middle_abd', 'middle_mcp', 'middle_pip'],
        'f4': ['ring_abd', 'ring_mcp', 'ring_pip'],
        'f5': ['pinky_abd', 'pinky_mcp', 'pinky_pip'],
        'f6': ['wrist']
    }

    def __init__(self, port: int = 8000, hand_type: str = "right", debug: bool = True):
        self.port = port
        self.hand_type = hand_type  # "left" 或 "right"
        self.debug = debug
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.1)

        self.current_data = {}
        self.data_lock = Lock()

        self.running = True
        self.last_packet_time = 0.0
        self.packet_count = 0
        self.first_packet_received = False

        # 调试数据
        self.last_raw_json = ""
        self.last_params = {}
        self.last_sensor_values = []  # 6 个传感器的原始值
        self.last_joint_angles = {}  # 17 个关节的角度值

        # 加载配置
        model_path = HandConfig.get_model_path(hand_type)
        self.joint_roms = HandConfig.load_roms_from_file(model_path)
        self.neutral_position = HandConfig.load_neutral_from_file(model_path)

        # 启动接收线程
        self.recv_thread = Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        print(f"[UDP 接收器] 已启动，监听端口 {self.port}，手部类型: {hand_type}")

    def _recv_loop(self):
        """UDP 接收线程"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                if not data:
                    continue

                self.packet_count += 1
                self.last_packet_time = time.time()

                try:
                    parsed = json.loads(data.decode("utf-8"))
                except json.JSONDecodeError as e:
                    if self.debug and self.packet_count % 100 == 0:
                        print(f"[UDP 接收器] JSON 解析失败: {e}")
                    continue

                with self.data_lock:
                    self.current_data = parsed
                    if self.debug:
                        self.last_raw_json = json.dumps(parsed, ensure_ascii=False, indent=2)

                if not self.first_packet_received:
                    self.first_packet_received = True
                    print(f"[UDP 接收器] 已收到第一包数据，来自: {addr[0]}:{addr[1]}")

                # 调试输出（每 100 包）
                if self.debug and self.packet_count % 100 == 0:
                    params = self._build_param_map(parsed)
                    prefix = 'l' if self.hand_type == "left" else 'r'
                    print(f"[UDP 接收器] 第 {self.packet_count} 包")
                    for fid, idx in self.LEFT_FINGER_INDICES.items():
                        key = f"{prefix}{idx}"
                        if key in params:
                            print(f"  {key} ({fid}): {params[key]:.2f}")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running and self.debug:
                    print(f"[UDP 接收器] 接收异常: {e}")
                time.sleep(0.05)

    def _build_param_map(self, data) -> dict:
        """从 JSON 数据构建参数映射"""
        params = {}
        for dev in data.values():
            for p in dev.get("Parameter", []):
                params[p.get("Name")] = p.get("Value")
        return params

    def _extract_sensor_values(self) -> Optional[List[float]]:
        """
        提取并转换手套传感器值到 0-1 范围
        返回: [f1, f2, f3, f4, f5, f6] 0-1 范围的值
        """
        with self.data_lock:
            snapshot = self.current_data.copy() if self.current_data else None

        if snapshot is None:
            return None

        params = self._build_param_map(snapshot)
        indices = self.LEFT_FINGER_INDICES  # 左右手的索引相同
        prefix = 'l' if self.hand_type == "left" else 'r'

        sensor_values = []
        valid_count = 0

        for i in range(1, 7):
            fid = f"f{i}"
            idx = indices[fid]
            key = f"{prefix}{idx}"
            if key in params:
                valid_count += 1
                raw_val = params[key]
                # 归一化到 0-1 (0=张开, 1=闭合)
                min_v = self.MIN_GLOVE_VALUES[i - 1]
                max_v = self.MAX_GLOVE_VALUES[i - 1]
                norm = max(0.0, min(1.0, (raw_val - min_v) / (max_v - min_v + 1e-6)))
                sensor_values.append(norm)
            else:
                sensor_values.append(0.5)  # 默认中间位置

        if valid_count < 4:
            return None

        # 保存用于调试
        if self.debug:
            self.last_params = params.copy()
            self.last_sensor_values = sensor_values.copy()

        return sensor_values

    def sensor_values_to_joint_angles(self, sensor_values: List[float]) -> Dict[str, float]:
        """
        将手套传感器值 (0-1) 转换为 ORCA Hand 关节角度 (度)

        Args:
            sensor_values: [f1, f2, f3, f4, f5, f6] 0-1 范围的值

        Returns:
            {joint_name: angle_in_degrees} 17 个关节的角度
        """
        joint_angles = {}

        for i, finger in enumerate(['f1', 'f2', 'f3', 'f4', 'f5', 'f6']):
            sensor_val = sensor_values[i]
            joints = self.GLOVE_TO_ORCA_MAP.get(finger, [])

            for joint in joints:
                rom = self.joint_roms.get(joint, [0, 90])
                min_angle, max_angle = rom

                # 根据关节类型进行映射
                if finger == 'f1':  # 拇指特殊处理
                    if joint == 'thumb_mcp':
                        # MCP: 从中性位置到弯曲
                        neutral = self.neutral_position.get('thumb_mcp', -13)
                        angle = neutral + sensor_val * (max_angle - neutral)
                    elif joint == 'thumb_abd':
                        # ABD: 保持相对稳定
                        angle = self.neutral_position.get('thumb_abd', 43)
                    elif joint == 'thumb_pip':
                        # PIP: 跟随 MCP
                        angle = self.neutral_position.get('thumb_pip', 33) + sensor_val * (max_angle - self.neutral_position.get('thumb_pip', 33)) * 0.8
                    elif joint == 'thumb_dip':
                        # DIP: 跟随 PIP
                        angle = self.neutral_position.get('thumb_dip', 19) + sensor_val * (max_angle - self.neutral_position.get('thumb_dip', 19)) * 0.6

                elif finger == 'f6':  # 手腕
                    # 手腕: 以中性位置为中心
                    neutral = self.neutral_position.get('wrist', 0)
                    angle = neutral + (sensor_val - 0.5) * (max_angle - min_angle)

                else:  # 其他四指
                    if joint.endswith('_abd'):
                        # 外展关节: 保持稳定
                        neutral_key = joint
                        angle = self.neutral_position.get(neutral_key, 0)
                    elif joint.endswith('_mcp'):
                        # MCP: 从中性位置到弯曲
                        neutral = self.neutral_position.get(joint, 0)
                        angle = neutral + sensor_val * (max_angle - neutral)
                    elif joint.endswith('_pip'):
                        # PIP: 跟随 MCP 但比例较小
                        neutral = self.neutral_position.get(joint, 0)
                        angle = neutral + sensor_val * (max_angle - neutral) * 0.7

                # 限制在 ROM 范围内
                angle = max(min_angle, min(max_angle, angle))
                joint_angles[joint] = float(angle)

        return joint_angles

    def get_joint_angles(self) -> Optional[Dict[str, float]]:
        """获取当前关节角度（度）"""
        sensor_values = self._extract_sensor_values()
        if sensor_values is None:
            return None

        joint_angles = self.sensor_values_to_joint_angles(sensor_values)

        # 保存用于调试
        if self.debug:
            self.last_joint_angles = joint_angles.copy()

        return joint_angles

    def get_sensor_values(self) -> Optional[List[float]]:
        """获取当前传感器值 (0-1)"""
        return self._extract_sensor_values()

    def is_data_available(self) -> bool:
        """检查是否有有效数据"""
        return self.first_packet_received and (time.time() - self.last_packet_time) < 1.0

    def get_debug_data(self) -> dict:
        """获取调试数据"""
        # 如果遥操作未运行，手动解析
        if not self.last_joint_angles and self.current_data:
            self.get_joint_angles()

        return {
            "raw_json": self.last_raw_json,
            "params": self.last_params,
            "sensor_values": self.last_sensor_values,  # 0-1 范围
            "joint_angles": self.last_joint_angles,  # 度数
            "packet_count": self.packet_count,
            "last_packet_time": self.last_packet_time,
            "data_available": self.is_data_available()
        }

    def shutdown(self):
        """关闭接收器"""
        self.running = False
        try:
            self.sock.close()
        except:
            pass


# =============================================================================
# 遥操作状态管理
# =============================================================================

class TeleopState:
    """遥操作状态管理"""
    def __init__(self):
        self.enabled = False
        self.smoothing_factor = 0.3
        self.last_joint_angles: Dict[str, float] = {}
        self.hand_type = "right"  # "left" 或 "right"
        self.model_path = None
        self.udp_mode = False

    def process_joint_angles(self, joint_angles: Dict[str, float]) -> Dict[str, float]:
        """处理关节角度并应用平滑"""
        smoothed_angles = {}

        for joint, angle in joint_angles.items():
            # 应用指数平滑
            last_angle = self.last_joint_angles.get(joint, angle)
            smoothed = self.smoothing_factor * angle + (1 - self.smoothing_factor) * last_angle
            smoothed_angles[joint] = smoothed
            self.last_joint_angles[joint] = smoothed

        return smoothed_angles


# =============================================================================
# UDP 自动控制线程
# =============================================================================

class UDPControlThread:
    """UDP 数据自动控制线程"""

    def __init__(self):
        self.running = False
        self.thread = None

    def _control_loop(self):
        """控制循环"""
        while self.running:
            if not teleop_state.enabled:
                time.sleep(0.05)
                continue

            if glove_receiver is None:
                time.sleep(0.05)
                continue

            # 获取关节角度
            joint_angles = glove_receiver.get_joint_angles()
            if joint_angles is None:
                time.sleep(0.02)
                continue

            # 应用平滑
            smoothed_angles = teleop_state.process_joint_angles(joint_angles)

            # 发送指令
            try:
                with command_lock:
                    hand.set_joint_pos(smoothed_angles)
            except Exception as e:
                if teleop_state.enabled and glove_receiver.debug:
                    print(f"[UDP 控制] 控制异常: {e}")

            time.sleep(0.02)  # 50Hz

    def start(self):
        """启动控制线程"""
        if not self.running:
            self.running = True
            self.thread = Thread(target=self._control_loop, daemon=True)
            self.thread.start()
            print("[UDP 控制] 控制线程已启动")

    def stop(self):
        """停止控制线程"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None
            print("[UDP 控制] 控制线程已停止")


# =============================================================================
# 错误处理
# =============================================================================

def handle_hand_exception(e: Exception):
    """将 OrcaHand 运行时错误转换为 HTTP 异常"""
    if isinstance(e, RuntimeError):
        if "not connected" in str(e).lower():
            raise HTTPException(status_code=409, detail=f"手部操作失败: {e}")
        elif "not calibrated" in str(e).lower():
            raise HTTPException(status_code=409, detail=f"手部操作失败: {e}")
        else:
            raise HTTPException(status_code=400, detail=f"手部操作失败: {e}")
    elif isinstance(e, ValueError):
        raise HTTPException(status_code=422, detail=f"无效输入: {e}")
    else:
        raise HTTPException(status_code=500, detail=f"服务器内部错误: {e}")


# =============================================================================
# 数据模型
# =============================================================================

class MotorList(BaseModel):
    motor_ids: Optional[List[int]] = None

class MaxCurrent(BaseModel):
    current: Union[float, List[float]]

class JointPositions(BaseModel):
    positions: Dict[str, float] = Field(..., example={"thumb_mcp": 0.0, "index_mcp": 45.0})

class GloveData(BaseModel):
    """手动控制的手套数据格式"""
    positions: Optional[List[int]] = Field(None, description="6个位置的数组 [f1-f6] (0-1000)")


# =============================================================================
# 基础 API 端点
# =============================================================================

def get_hand():
    """获取或创建手部实例"""
    global hand
    if hand is None:
        model_path = HandConfig.get_model_path("right")
        hand = OrcaHand(model_path=model_path)
    return hand


@app.post("/config", summary="设置手部配置", tags=["配置"])
def set_hand_config(
    hand_type: str = Body("right", description="left 或 right"),
    model_path: str = Body(None, description="自定义模型路径")
):
    """设置或更新手部配置"""
    global hand, teleop_state

    if teleop_state and teleop_state.enabled:
        raise HTTPException(status_code=409, detail="遥操作运行中，无法修改配置")

    try:
        if hand_type not in ["left", "right"]:
            raise HTTPException(status_code=400, detail="hand_type 必须是 'left' 或 'right'")

        path = model_path if model_path else HandConfig.get_model_path(hand_type)

        if hand and hand.is_connected():
            hand.disconnect()

        hand = OrcaHand(model_path=path)

        return {
            "message": f"手部配置已更新: {hand_type}",
            "hand_type": hand_type,
            "model_path": path
        }
    except Exception as e:
        handle_hand_exception(e)


@app.post("/connect", summary="连接到 OrcaHand", tags=["连接"])
def connect_hand():
    """建立与 OrcaHand 硬件的连接"""
    h = get_hand()
    if h.is_connected():
        return {"message": "手部已连接"}
    try:
        success, msg = h.connect()
        if success:
            return {"message": msg}
        else:
            raise HTTPException(status_code=500, detail=f"连接失败: {msg}")
    except Exception as e:
        handle_hand_exception(e)


@app.post("/disconnect", summary="断开 OrcaHand 连接", tags=["连接"])
def disconnect_hand():
    """断开与 OrcaHand 硬件的连接"""
    global teleop_state, udp_control_thread

    if teleop_state and teleop_state.enabled:
        stop_teleop()

    h = get_hand()
    if not h.is_connected():
        return {"message": "手部已断开连接"}
    try:
        try:
            h.disable_torque()
            time.sleep(0.1)
        except Exception:
            pass

        success, msg = h.disconnect()
        if success:
            return {"message": msg}
        else:
            raise HTTPException(status_code=500, detail=f"断开连接失败: {msg}")
    except Exception as e:
        handle_hand_exception(e)


@app.get("/status", summary="获取手部状态", tags=["状态"])
def get_status():
    """获取当前连接和校准状态"""
    h = get_hand()
    udp_status = None
    if glove_receiver:
        udp_status = {
            "enabled": teleop_state.udp_mode if teleop_state else False,
            "hand_type": glove_receiver.hand_type,
            "data_available": glove_receiver.is_data_available(),
            "packet_count": glove_receiver.packet_count,
            "last_packet_time": glove_receiver.last_packet_time
        }

    try:
        return {
            "connected": h.is_connected(),
            "calibrated": h.is_calibrated() if h.is_connected() else False,
            "teleop_enabled": teleop_state.enabled if teleop_state else False,
            "hand_type": teleop_state.hand_type if teleop_state else "not configured",
            "udp_status": udp_status
        }
    except Exception as e:
        handle_hand_exception(e)


@app.post("/torque/enable", summary="使能电机扭矩", tags=["控制"])
def enable_torque(motor_list: MotorList = Body(None)):
    """使能指定电机或所有电机的扭矩"""
    h = get_hand()
    try:
        ids = motor_list.motor_ids if motor_list else None
        h.enable_torque(motor_ids=ids)
        return {"message": f"已为电机使能扭矩: {ids if ids else '全部'}"}
    except Exception as e:
        handle_hand_exception(e)


@app.post("/torque/disable", summary="失能电机扭矩", tags=["控制"])
def disable_torque(motor_list: MotorList = Body(None)):
    """失能指定电机或所有电机的扭矩"""
    h = get_hand()
    try:
        ids = motor_list.motor_ids if motor_list else None
        h.disable_torque(motor_ids=ids)
        return {"message": f"已为电机失能扭矩: {ids if ids else '全部'}"}
    except Exception as e:
        handle_hand_exception(e)


@app.get("/joints/position", summary="获取关节位置", tags=["状态"])
def get_joint_position():
    """获取所有已校准关节的当前位置（度）"""
    h = get_hand()
    try:
        j_pos = h.get_joint_pos(as_list=False)
        return {"positions": j_pos}
    except Exception as e:
        handle_hand_exception(e)


@app.post("/joints/position", summary="设置关节位置", tags=["控制"])
def set_joint_position(joint_positions: JointPositions):
    """
    设置指定关节的期望位置（度）。

    例如: {"thumb_mcp": 10.0, "index_mcp": 45.0, ...}
    """
    h = get_hand()
    try:
        h.set_joint_pos(joint_pos=joint_positions.positions)
        return {"message": "关节位置指令发送成功"}
    except Exception as e:
        handle_hand_exception(e)


@app.get("/calibrate/status", summary="获取校准状态", tags=["校准"])
def get_calibration_status():
    """检查手部是否已完全校准"""
    h = get_hand()
    try:
        return {"calibrated": h.is_calibrated()}
    except Exception as e:
        handle_hand_exception(e)


@app.post("/calibrate", summary="自动校准", tags=["校准"])
def calibrate_auto():
    """启动配置中定义的自动校准程序"""
    h = get_hand()
    if not h.is_connected():
        raise HTTPException(status_code=409, detail="必须先连接手部才能校准")
    try:
        h.calibrate()
        calib_status = h.is_calibrated()
        msg = "自动校准完成" + ("成功" if calib_status else "失败或未完成")
        return {"message": msg, "calibrated": calib_status}
    except Exception as e:
        handle_hand_exception(e)


@app.get("/config/roms", summary="获取关节活动范围", tags=["配置"])
def get_joint_roms():
    """获取所有关节的活动范围（度）"""
    model_path = HandConfig.get_model_path("right")
    roms = HandConfig.load_roms_from_file(model_path)
    return {"joint_roms": roms}


@app.get("/config/neutral", summary="获取中性位置", tags=["配置"])
def get_neutral_position():
    """获取中性位置（度）"""
    model_path = HandConfig.get_model_path("right")
    neutral = HandConfig.load_neutral_from_file(model_path)
    return {"neutral_position": neutral}


# =============================================================================
# 遥操作端点
# =============================================================================

@app.post("/teleop/udp/init", summary="初始化 UDP 接收器", tags=["遥操作"])
def init_udp_receiver(hand_type: str = Body("right", embed=False)):
    """初始化 UDP 接收器用于调试"""
    global glove_receiver, teleop_state

    if glove_receiver is not None:
        return {
            "message": "UDP 接收器已存在",
            "hand_type": glove_receiver.hand_type,
            "port": glove_receiver.port,
            "packet_count": glove_receiver.packet_count
        }

    teleop_state = teleop_state or TeleopState()
    teleop_state.hand_type = hand_type

    glove_receiver = UDPGloveReceiver(port=8000, hand_type=hand_type, debug=True)
    time.sleep(0.5)

    return {
        "message": "UDP 接收器已初始化",
        "hand_type": hand_type,
        "port": 8000
    }


@app.post("/teleop/start", summary="启动遥操作模式", tags=["遥操作"])
def start_teleop():
    """启动遥操作模式，使能扭矩并从 UDP 接收手套数据"""
    global hand, teleop_state, glove_receiver, udp_control_thread

    h = get_hand()

    try:
        if not h.is_connected():
            raise HTTPException(status_code=409, detail="必须先连接手部")

        if not h.is_calibrated():
            raise HTTPException(status_code=409, detail="必须先校准手部")

        # 初始化状态
        if teleop_state is None:
            teleop_state = TeleopState()

        # 初始化 UDP 接收器
        if glove_receiver is None:
            glove_receiver = UDPGloveReceiver(port=8000, hand_type=teleop_state.hand_type, debug=True)
            time.sleep(0.5)

        h.enable_torque()
        teleop_state.enabled = True
        teleop_state.udp_mode = True

        # 用当前手部位置初始化
        current_pos = h.get_joint_pos(as_list=False)
        if current_pos:
            teleop_state.last_joint_angles = current_pos.copy()
        else:
            teleop_state.last_joint_angles = HandConfig.NEUTRAL_POSITION.copy()

        # 启动 UDP 控制线程
        if udp_control_thread is None:
            udp_control_thread = UDPControlThread()
        udp_control_thread.start()

        return {
            "message": f"遥操作模式已启动，监听 UDP 端口 8000",
            "enabled": True,
            "hand_type": teleop_state.hand_type,
            "data_available": glove_receiver.is_data_available()
        }
    except Exception as e:
        handle_hand_exception(e)


@app.post("/teleop/stop", summary="停止遥操作模式", tags=["遥操作"])
def stop_teleop():
    """停止遥操作模式，失能扭矩并停止 UDP 控制"""
    global teleop_state, udp_control_thread

    if teleop_state is None:
        return {"message": "遥操作未启动", "enabled": False}

    try:
        teleop_state.enabled = False
        teleop_state.udp_mode = False

        if udp_control_thread:
            udp_control_thread.stop()

        h = get_hand()
        h.disable_torque()
        return {"message": "遥操作模式已停止", "enabled": False}
    except Exception as e:
        handle_hand_exception(e)


@app.get("/teleop/status", summary="获取遥操作状态", tags=["遥操作"])
def get_teleop_status():
    """获取当前遥操作状态"""
    h = get_hand()
    status = {
        "enabled": teleop_state.enabled if teleop_state else False,
        "hand_type": teleop_state.hand_type if teleop_state else "not configured",
        "connected": h.is_connected(),
        "calibrated": h.is_calibrated(),
        "smoothing_factor": teleop_state.smoothing_factor if teleop_state else 0.3
    }

    if glove_receiver:
        status["udp_receiver"] = {
            "port": glove_receiver.port,
            "data_available": glove_receiver.is_data_available(),
            "packet_count": glove_receiver.packet_count
        }

    return status


@app.get("/teleop/glove/current", summary="获取当前手套数据", tags=["遥操作"])
def get_current_glove_data():
    """获取当前手套数据（传感器值和关节角度）"""
    if glove_receiver is None:
        raise HTTPException(status_code=409, detail="UDP 接收器未初始化")

    debug_data = glove_receiver.get_debug_data()

    return {
        "data_available": debug_data["data_available"],
        "packet_count": debug_data["packet_count"],
        "sensor_values": debug_data["sensor_values"],  # 0-1 范围
        "joint_angles": debug_data["joint_angles"]  # 度数
    }


@app.get("/teleop/glove/debug", summary="获取详细调试数据", tags=["遥操作"])
def get_glove_debug_data():
    """获取详细的手套数据调试信息"""
    if glove_receiver is None:
        raise HTTPException(status_code=409, detail="UDP 接收器未初始化")

    debug_data = glove_receiver.get_debug_data()

    return {
        "hand_type": glove_receiver.hand_type,
        "packet_count": debug_data["packet_count"],
        "data_available": debug_data["data_available"],
        "sensor_values": debug_data["sensor_values"],
        "joint_angles": debug_data["joint_angles"],
        "parsed_params": debug_data["params"],
        "raw_json": debug_data["raw_json"]
    }


@app.post("/teleop/smoothing", summary="设置平滑系数", tags=["遥操作"])
def set_smoothing(factor: float = Body(..., ge=0.0, le=1.0)):
    """设置平滑系数 (0.0-1.0)"""
    if teleop_state is None:
        raise HTTPException(status_code=409, detail="遥操作未初始化")

    teleop_state.smoothing_factor = factor
    return {
        "message": f"平滑系数已设置为 {factor}",
        "smoothing_factor": factor
    }


@app.post("/teleop/glove", summary="手动发送手套数据（测试用）", tags=["遥操作"])
def receive_glove_data(glove_data: GloveData):
    """
    手动发送手套数据用于测试。

    positions: [f1, f2, f3, f4, f5, f6] 0-1000 范围
    """
    global glove_receiver, teleop_state

    if glove_receiver is None:
        raise HTTPException(status_code=409, detail="UDP 接收器未初始化")

    if glove_data.positions is None:
        raise HTTPException(status_code=400, detail="缺少 positions 参数")

    # 转换 0-1000 到 0-1
    sensor_values = [p / 1000.0 for p in glove_data.positions]

    # 转换为关节角度
    joint_angles = glove_receiver.sensor_values_to_joint_angles(sensor_values)

    # 发送到手部
    h = get_hand()
    try:
        h.set_joint_pos(joint_angles)
        return {
            "status": "ok",
            "sensor_values": sensor_values,
            "joint_angles": joint_angles
        }
    except Exception as e:
        handle_hand_exception(e)


# =============================================================================
# 主程序
# =============================================================================

if __name__ == "__main__":
    print("=" * 70)
    print("OrcaHand 遥操作 API 服务器 v2.0")
    print("=" * 70)
    print("服务器运行地址: http://0.0.0.0:8000")
    print("API 文档地址: http://localhost:8000/docs")
    print()
    print("数据源: UDP 端口 8000")
    print("控制方式: 实际关节角度（度）")
    print()
    print("快速开始:")
    print("  1. POST /config            - 配置手部类型 (left/right)")
    print("  2. POST /connect           - 连接手部")
    print("  3. POST /calibrate         - 校准手部")
    print("  4. POST /teleop/start      - 启动遥操作")
    print("  5. POST /teleop/stop       - 停止遥操作")
    print()
    print("调试:")
    print("  POST /teleop/udp/init     - 初始化 UDP 接收器")
    print("  GET  /teleop/glove/debug   - 查看手套数据")
    print("  GET  /config/roms          - 查看关节活动范围")
    print("=" * 70)

    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    finally:
        if glove_receiver:
            glove_receiver.shutdown()
        if udp_control_thread:
            udp_control_thread.stop()
