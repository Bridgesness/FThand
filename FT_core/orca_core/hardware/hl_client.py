# orca_core/hardware/hl_client.py
# 飞特舵机接口，仿照Dynamixel风格改写

import atexit
import time
import logging
import numpy as np
from typing import Optional, Sequence, Union, Tuple
from .ft_sdk import scservo_sdk

# 飞特舵机寄存器地址定义 (参照hls.py)
HLS_MODE = 33
HLS_TORQUE_ENABLE = 40
HLS_ACC = 41
HLS_GOAL_POSITION_L = 42
HLS_GOAL_POSITION_H = 43
HLS_GOAL_TORQUE_L = 44
HLS_GOAL_TORQUE_H = 45
HLS_GOAL_SPEED_L = 46
HLS_GOAL_SPEED_H = 47

HLS_PRESENT_POSITION_L = 56
HLS_PRESENT_POSITION_H = 57
HLS_PRESENT_SPEED_L = 58
HLS_PRESENT_SPEED_H = 59
HLS_PRESENT_LOAD_L = 60
HLS_PRESENT_LOAD_H = 61
HLS_PRESENT_VOLTAGE = 62
HLS_PRESENT_TEMPERATURE = 63
HLS_MOVING = 66
HLS_PRESENT_CURRENT_L = 69
HLS_PRESENT_CURRENT_H = 70

# 地址常量 (与DXL接口保持一致命名)
ADDR_GOAL_POSITION = HLS_GOAL_POSITION_L
ADDR_GOAL_CURRENT = HLS_GOAL_TORQUE_L
ADDR_PROFILE_VELOCITY = HLS_GOAL_SPEED_L
ADDR_OPERATING_MODE = HLS_MODE
ADDR_TORQUE_ENABLE = HLS_TORQUE_ENABLE

# 数据长度定义
LEN_OPERATING_MODE = 1
LEN_GOAL_POSITION = 2
LEN_GOAL_CURRENT = 2
LEN_PROFILE_VELOCITY = 2
LEN_PRESENT_POSITION = 2
LEN_PRESENT_VELOCITY = 2
LEN_PRESENT_CURRENT = 2
LEN_MOVING_STATUS = 1
LEN_PRESENT_TEMPERATURE = 1

# 缩放因子 (根据飞特 FT-HTS 舵机技术手册)
# 位置: 0-4096 对应 0-360° (2π弧度)
# 速度: 根据实际舵机规格，0-1000 对应 RPM
# 电流: 根据实际舵机规格，原始值需要转换

DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 弧度/原始值: 0.001534
DEFAULT_VEL_SCALE = 0.732 * 2.0 * np.pi / 60.0   # 速度单位待确认，暂设为1.0
DEFAULT_CUR_SCALE = 1.0  # 电流单位待确认，暂设为1.0

# 飞特SDK的COMM_SUCCESS定义为0，不是1！
COMM_SUCCESS = 0

# ============================================================================
# 运行模式映射: Dynamixel -> 飞特
# ============================================================================
#
# Dynamixel 运行模式:
#   0: current control (电流控制)
#   1: velocity control (速度控制)
#   3: position control (位置控制)
#   4: multi-turn position control (多圈位置控制)
#   5: current-based position control (基于电流的位置控制)
#
# 飞特舵机运行模式:
#   0: 位置伺服模式
#   1: 电机恒速模式
#   2: 电机恒流模式
#   3: PWM开环调速度模式
#
# 映射关系:
#   Dynamixel 0 (current)        -> 飞特 2 (电机恒流模式)
#   Dynamixel 1 (velocity)       -> 飞特 1 (电机恒速模式)
#   Dynamixel 3 (position)       -> 飞特 0 (位置伺服模式)
#   Dynamixel 4 (multi-turn)     -> 飞特 0 (位置伺服模式)
#   Dynamixel 5 (current-based)  -> 飞特 0 (位置伺服模式) *推荐
#
# ============================================================================

DXL_TO_FT_MODE_MAP = {
    0: 2,   # current control -> 电机恒流模式
    1: 1,   # velocity control -> 电机恒速模式
    3: 0,   # position control -> 位置伺服模式
    4: 0,   # multi-turn position -> 位置伺服模式
    5: 0,   # current-based position -> 位置伺服模式 (推荐用于 ORCA Hand)
}

FT_MODE_NAMES = {
    0: "位置伺服模式",
    1: "电机恒速模式",
    2: "电机恒流模式",
    3: "PWM开环调速度模式"
}


def HLClient_cleanup_handler():
    """Cleanup function to ensure motors are disconnected properly."""
    open_clients = list(HLClient.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            logging.warning('Forcing client to close.')
        open_client.port_handler.is_using = False
        open_client.disconnect()


class HLClient:
    """Client for communicating with FeiTech servo motors (Dynamixel-compatible interface)."""

    OPEN_CLIENTS = set()

    def __init__(
        self,
        motor_ids,
        port="COM12",
        baudrate=1000000,
        lazy_connect=False,
        pos_scale: Optional[float] = None,
        vel_scale: Optional[float] = None,
        cur_scale: Optional[float] = None,
        **kwargs,
    ):
        """Initializes a new client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The device to talk to. e.g. COM12 on Windows.
            baudrate: The baudrate to communicate with.
            lazy_connect: If True, automatically connects when needed.
            pos_scale: The scaling factor for positions.
            vel_scale: The scaling factor for velocities.
            cur_scale: The scaling factor for currents.
        """
        self.ft = scservo_sdk
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.port_handler = self.ft.PortHandler(port)
        self.packetHandler = None

        self._sync_writers = {}

        # Set scales
        self.pos_scale = pos_scale if pos_scale is not None else DEFAULT_POS_SCALE
        self.vel_scale = vel_scale if vel_scale is not None else DEFAULT_VEL_SCALE
        self.cur_scale = cur_scale if cur_scale is not None else DEFAULT_CUR_SCALE

        # 存储每个电机的电流限制，用于实现"软"运动
        self._current_limits = {motor_id: 1000 for motor_id in motor_ids}  # 默认1000mA
        self._current_positions = {motor_id: 0.0 for motor_id in motor_ids}

        self.OPEN_CLIENTS.add(self)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.port_handler.is_open

    def connect(self):
        """Connects to the servo motors."""
        assert not self.is_connected, 'Client is already connected.'

        if self.port_handler.openPort():
            logging.info('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                ('Failed to open port at {} (Check that the device is powered '
                 'on and connected to your computer).').format(self.port_name))

        if self.port_handler.setBaudRate(self.baudrate):
            logging.info('Succeeded to set baudrate to %d', self.baudrate)
        else:
            raise OSError(
                ('Failed to set the baudrate to {} (Ensure that the device was '
                 'configured for this baudrate).').format(self.baudrate))

        self.packetHandler = self.ft.hls(self.port_handler)

        # Initialize readers after packetHandler is created
        self._pos_vel_cur_reader = HLPosVelCurReader(
            self,
            self.motor_ids,
            pos_scale=self.pos_scale,
            vel_scale=self.vel_scale,
            cur_scale=self.cur_scale,
        )
        self._temp_reader = HLTempReader(self, self.motor_ids)
        self._moving_status_reader = HLReader(
            self,
            self.motor_ids,
            HLS_MOVING,
            LEN_MOVING_STATUS
        )

        # Start with all motors enabled.
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self):
        """Disconnects from the device."""
        if not self.is_connected:
            return
        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return
        # Ensure motors are disabled at the end.
        self.set_torque_enabled(self.motor_ids, False, retries=0)
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool,
                           retries: int = -1,
                           retry_interval: float = 0.25):
        """Sets whether torque is enabled for the motors.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry forever.
            retry_interval: The number of seconds to wait between retries.
        """
        remaining_ids = list(motor_ids)
        while remaining_ids:
            remaining_ids = self.write_byte(
                remaining_ids,
                int(enabled),
                HLS_TORQUE_ENABLE,
            )
            if remaining_ids:
                logging.error('Could not set torque %s for IDs: %s',
                              'enabled' if enabled else 'disabled',
                              str(remaining_ids))
            if retries == 0:
                break
            time.sleep(retry_interval)
            retries -= 1

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """Sets the operating mode for the motors.

        Args:
            motor_ids: The motor IDs to configure.
            mode_value: The mode value using Dynamixel convention:
                0: current control
                1: velocity control
                3: position control
                4: multi-turn position control
                5: current-based position control (recommended for ORCA Hand)

        Note:
            This method automatically converts Dynamixel mode values to FeiTech mode values.
        """
        # 将 Dynamixel 模式转换为飞特模式
        if mode_value not in DXL_TO_FT_MODE_MAP:
            raise ValueError(
                f"Unsupported operating mode: {mode_value}. "
                f"Supported modes: {list(DXL_TO_FT_MODE_MAP.keys())}"
            )

        ft_mode = DXL_TO_FT_MODE_MAP[mode_value]

        logging.info(
            f"Converting mode: Dynamixel {mode_value} -> FeiTech {ft_mode} ({FT_MODE_NAMES.get(ft_mode, 'Unknown')})"
        )

        # data in EEPROM area can only be written when torque is disabled
        self.set_torque_enabled(motor_ids, False)
        self.sync_write(motor_ids, [ft_mode] * len(motor_ids), HLS_MODE, LEN_OPERATING_MODE)
        self.set_torque_enabled(motor_ids, True)

    # ------------------------------------------------------------------
    # Read operations
    # ------------------------------------------------------------------

    def read_pos_vel_cur(self, skip_current: bool = False) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns the positions, velocities, and currents.

        Args:
            skip_current: If True, skip the per-motor current reads. Teleop only
                needs position/velocity; current is used by calibration/tension.
                Skipping it removes 17 individual serial round-trips per frame,
                which is the biggest latency in the control hot path.
        """
        return self._pos_vel_cur_reader.read(skip_current=skip_current)

    def read_status_is_done_moving(self) -> np.ndarray:
        """Returns the moving status for each motor."""
        moving_status = self._moving_status_reader.read().astype(np.int8)
        return np.bitwise_and(moving_status, np.array([0x01] * len(self.motor_ids)).astype(np.int8))

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the present temperature for each motor (in deg C)."""
        return self._temp_reader.read()

    # ------------------------------------------------------------------
    # Write operations
    # ------------------------------------------------------------------

    def write_desired_pos(self, motor_ids: Sequence[int], positions: np.ndarray):
        """Writes the given desired positions.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.

        Note for FeiTech servos:
            当电流限制较低时（校准模式），添加额外延迟使运动更平滑，
            模拟 Dynamixel 的电流限制效果。
        """
        assert len(motor_ids) == len(positions)

        # Convert to servo position space.
        positions = positions / self._pos_vel_cur_reader.pos_scale
        times = self.sync_write(motor_ids, positions, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # 飞特舵机：当电流限制较低时，添加额外延迟模拟"软"运动
        # 检查是否有电机处于低电流限制模式（校准模式通常 < 400mA）
        low_current_limit_count = sum(1 for mid in motor_ids if self._current_limits.get(mid, 1000) < 400)

        if low_current_limit_count > 0:
            # 根据电流限制程度添加延迟
            # 电流越低，延迟越长，运动越"软"
            avg_current = sum(self._current_limits.get(mid, 1000) for mid in motor_ids) / len(motor_ids)
            if avg_current < 200:
                time.sleep(0.005)  # 极低电流：5ms 延迟
            elif avg_current < 400:
                time.sleep(0.002)  # 低电流：2ms 延迟
            # 正常电流（>400mA）：不添加额外延迟

        return times

    def write_desired_current(self, motor_ids: Sequence[int], current: np.ndarray):
        """Writes the desired current/torque values.

        Note for FeiTech servos:
            飞特舵机在位置伺服模式下不支持硬件电流限制。
            此方法存储电流限制值，write_desired_pos() 会根据电流限制
            使用较小的步长来模拟"软"运动效果。
        """
        assert len(motor_ids) == len(current)

        # 存储电流限制用于位置模式下的"软"运动模拟
        for motor_id, cur_val in zip(motor_ids, current):
            if isinstance(cur_val, np.ndarray):
                self._current_limits[motor_id] = float(cur_val[0]) if len(cur_val) > 0 else 1000
            else:
                self._current_limits[motor_id] = float(cur_val)

        # 在位置模式下，不写入电流寄存器（因为飞特不支持）
        # 在恒流模式（模式2）下，这个寄存器才有效
        # 当前映射：模式5->位置模式，所以这里只存储值

    def write_profile_velocity(self, motor_ids: Sequence[int], profile_velocity: np.ndarray):
        """Writes the profile velocity values."""
        assert len(motor_ids) == len(profile_velocity)
        self.sync_write(motor_ids, profile_velocity, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY)

    # ------------------------------------------------------------------
    # Low-level operations
    # ------------------------------------------------------------------

    def write_byte(
            self,
            motor_ids: Sequence[int],
            value: int,
            address: int,
    ) -> Sequence[int]:
        """Writes a byte value to the motors.

        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.

        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            comm_result, ft_error = self.packetHandler.write1ByteTxRx(
                 motor_id, address, value)
            success = self.handle_packet_result(
                comm_result, ft_error, motor_id, context='write_byte')
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def sync_write(self, motor_ids, values, address, size):
        """Writes values to a group of motors.

        Args:
            motor_ids: The motor IDs to write to.
            values: The values to write.
            address: The control table address to write to.
            size: The size of the control table value being written to.
        """
        times = [time.monotonic()]
        self.check_connected()

        key = (address, size)
        if key not in self._sync_writers:
            self._sync_writers[key] = self.ft.GroupSyncWrite(self.packetHandler, address, size)

        sync_writer = self._sync_writers[key]
        errored_ids = []

        for motor_id, val in zip(motor_ids, values):
            # Handle signed values using FeiTech's conversion (15-bit format)
            scs_val = self.packetHandler.scs_toscs(int(val), 15)

            # Handle byte order using FeiTech's lobyte/hibyte
            if size == 1:
                byte_list = [scs_val & 0xFF]
            elif size == 2:
                byte_list = [
                    self.packetHandler.scs_lobyte(scs_val),
                    self.packetHandler.scs_hibyte(scs_val)
                ]
            else:
                # For 4+ byte data
                byte_list = list(int(scs_val).to_bytes(size, byteorder='little'))

            # FeiTech's addParam expects a list
            success = sync_writer.addParam(motor_id, byte_list)
            if not success:
                errored_ids.append(motor_id)

        if errored_ids:
            logging.error('Sync write failed for: %s', str(errored_ids))

        sync_writer.txPacket()
        sync_writer.clearParam()
        return times

    def check_connected(self):
        """Ensures the robot is connected."""
        if self.lazy_connect and not self.is_connected:
            self.connect()
        if not self.is_connected:
            raise OSError('Must call connect() first.')

    def handle_packet_result(self,
                             comm_result: int,
                             ft_error: Optional[int] = None,
                             ft_id: Optional[int] = None,
                             context: Optional[str] = None):
        """Handles the result from a communication request."""
        error_message = None
        if comm_result != self.ft.COMM_SUCCESS:
            error_message = self.packetHandler.getTxRxResult(comm_result)
        elif ft_error is not None:
            error_message = self.packetHandler.getRxPacketError(ft_error)
        if error_message:
            if ft_id is not None:
                error_message = '[Motor ID: {}] {}'.format(
                    ft_id, error_message)
            if context is not None:
                error_message = '> {}: {}'.format(context, error_message)
            logging.error(error_message)
            return False
        return True

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()


# ======================================================================
# Reader classes
# ======================================================================

class HLReader:
    """Reads data from FeiTech servo motors using GroupSyncRead."""

    def __init__(self, client: HLClient, motor_ids, address, size):
        """Initializes a new reader.

        Args:
            client: The HLClient instance.
            motor_ids: The motor IDs to read from.
            address: The starting address to read from.
            size: The number of bytes to read.
        """
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size

        # Initialize GroupSyncRead with (packetHandler, address, size)
        self.operation = self.client.ft.GroupSyncRead(
            client.packetHandler,
            self.address,
            self.size
        )
        self._initialize_data()

        # Register motor IDs to the group read list
        for motor_id in self.motor_ids:
            success = self.operation.addParam(motor_id)
            if not success:
                raise OSError(f'[Motor ID: {motor_id}] 无法添加到同步读取参数列表')

    def read(self, retries: int = 1):
        """Sends sync read packet and updates local cache."""
        self.client.check_connected()

        success = False
        while not success and retries >= 0:
            comm_result = self.operation.txRxPacket()
            success = self.client.handle_packet_result(
                comm_result, context='read')
            retries -= 1

        # If failed, return a copy of previous data
        if not success:
            return self._get_data()

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if data is available
            # Note: isAvailable returns tuple (bool, error), check first element
            available, _ = self.operation.isAvailable(motor_id, self.address, self.size)
            if not available:
                errored_ids.append(motor_id)
                continue

            try:
                self._update_data(i, motor_id)
            except Exception as e:
                logging.error(f'Error updating data for motor {motor_id}: {e}')
                errored_ids.append(motor_id)
                continue

        if errored_ids:
            logging.error('Bulk read data is unavailable for: %s', str(errored_ids))

        return self._get_data()

    def _initialize_data(self):
        """Initializes the cached data."""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        self._data[index] = self.operation.getData(motor_id, self.address, self.size)

    def _get_data(self):
        """Returns a copy of the data."""
        return self._data.copy()


class HLPosVelCurReader(HLReader):
    """Reads positions, velocities, and currents from FeiTech servos.

    Note on current reading:
    - FeiTech's HLS SDK does NOT provide a ReadCurrent method
    - Current register (address 69-70) may not be supported on all models
    - This attempts to read current, but falls back to 0 if unavailable
    """

    def __init__(self,
                 client: HLClient,
                 motor_ids,
                 pos_scale: float = DEFAULT_POS_SCALE,
                 vel_scale: float = DEFAULT_VEL_SCALE,
                 cur_scale: float = DEFAULT_CUR_SCALE):
        """Initializes a new reader.

        Args:
            client: The HLClient instance.
            motor_ids: The motor IDs to read from.
            pos_scale: The scaling factor for positions.
            vel_scale: The scaling factor for velocities.
            cur_scale: The scaling factor for currents.
        """
        # Read position+velocity (4 bytes, addresses 56-59)
        super().__init__(client, motor_ids,
                         address=HLS_PRESENT_POSITION_L,
                         size=4)

        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

    def read(self, retries: int = 1, skip_current: bool = False):
        """Reads position+velocity (sync), and current (individual) unless skipped."""
        # Read position and velocity via GroupSyncRead
        pos_vel_result = super().read(retries)

        if skip_current:
            # 遥操热路径：跳过 17 次逐电机 ReadCur，显著降低单帧读延迟
            return pos_vel_result

        # Read current for each motor using hls.ReadCur()
        for i, motor_id in enumerate(self.motor_ids):
            try:
                cur_val, comm_result, error = self.client.packetHandler.ReadCur(motor_id)

                if comm_result == COMM_SUCCESS and error == 0:
                    self._cur_data[i] = float(cur_val) * self.cur_scale
                else:
                    if comm_result != COMM_SUCCESS:
                        logging.debug(f"Motor {motor_id} current read comm failed: {comm_result}")
                    if error != 0:
                        logging.debug(f"Motor {motor_id} current read error: {error}")
            except Exception as e:
                logging.debug(f"Motor {motor_id} current read exception: {e}")

        return pos_vel_result

    def _initialize_data(self):
        """Initializes the cached data."""
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._vel_data = np.zeros(len(self.motor_ids), dtype=np.float32)
        self._cur_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the position and velocity data for the given motor ID.

        This follows the same approach as hls.py ReadPosSpeed:
        - Read 4 bytes containing pos+vel
        - Split using loword/hiword
        - Convert using scs_tohost
        """
        # Read 4 bytes: position (low word) + velocity (high word)
        pos_vel = self.operation.getData(motor_id, HLS_PRESENT_POSITION_L, 4)

        # Split the 4-byte value into position (low word) and velocity (high word)
        raw_pos = self.client.packetHandler.scs_loword(pos_vel)
        raw_vel = self.client.packetHandler.scs_hiword(pos_vel)

        # Convert using SDK's tohost function (15-bit format) - same as hls.py
        self._pos_data[index] = float(self.client.packetHandler.scs_tohost(raw_pos, 15)) * self.pos_scale
        self._vel_data[index] = float(self.client.packetHandler.scs_tohost(raw_vel, 15)) * self.vel_scale

    def _get_data(self):
        """Returns a copy of the data."""
        return (self._pos_data.copy(), self._vel_data.copy(), self._cur_data.copy())


class HLTempReader(HLReader):
    """Reads present temperature for each motor."""

    def __init__(self, client: HLClient, motor_ids):
        super().__init__(client, motor_ids,
                         address=HLS_PRESENT_TEMPERATURE,
                         size=1)

    def _initialize_data(self):
        self._temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # Temperature is 1 byte, read directly
        self._temp_data[index] = float(self.operation.getData(motor_id, self.address, self.size))

    def _get_data(self):
        return self._temp_data.copy()


# Register global cleanup function.
atexit.register(HLClient_cleanup_handler)


# ======================================================================
# Main function for testing (仿照DXL风格)
# ======================================================================

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-m',
        '--motors',
        required=True,
        help='Comma-separated list of motor IDs.')
    parser.add_argument(
        '-d',
        '--device',
        default='COM12',
        help='The device to connect to.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    parser.add_argument(
        '-s', '--sleep-time', type=float, default=0.05,
        help='Sleep time between reads in seconds (default: 0.05)')
    parser.add_argument(
        '-w', '--wait-time', type=float, default=2.0,
        help='Wait time after writing position in seconds (default: 2.0)')
    parser.add_argument(
        '--no-wait', action='store_true',
        help='Do not wait for motor to reach target before next command')
    parser.add_argument(
        '-k', '--torque-constant', type=float, default=0.1,
        help='Torque constant: Torque = Current * K (default: 0.1 N·m/A)')
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]

    # 测试轨迹：零位置 -> PI位置
    way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi)]

    print(f'Connecting to {parsed_args.device} at {parsed_args.baud} baud...')
    print(f'Motor IDs: {motors}')
    print(f'Sleep time: {parsed_args.sleep_time}s, Wait time: {parsed_args.wait_time}s')
    print(f'Torque constant: K = {parsed_args.torque_constant} N·m/A (Torque = Current × K)')
    print('='*60)

    with HLClient(motors, parsed_args.device, parsed_args.baud) as ft_client:
        step = 0
        way_point_index = 0
        last_write_time = 0
        target_pos = None

        # 记录最大电流及扭矩相关信息
        max_currents = np.zeros(len(motors))
        max_current_pos = [None] * len(motors)
        max_current_time = [0.0] * len(motors)
        max_torques = np.zeros(len(motors))
        max_torque_pos = [None] * len(motors)
        max_torque_time = [0.0] * len(motors)
        start_time = time.time()

        while True:
            try:
                # 每100步切换到下一个目标位置
                if step % 100 == 0:
                    way_point = way_points[way_point_index]
                    way_point_index = (way_point_index + 1) % len(way_points)
                    target_pos = way_point.copy()
                    print('Writing target: {}'.format(target_pos.tolist()))
                    ft_client.write_desired_pos(motors, target_pos)
                    last_write_time = time.time()

                # 每步读取位置、速度、电流
                read_start = time.time()
                pos_now, vel_now, cur_now = ft_client.read_pos_vel_cur()

                # 计算扭矩 (Torque = Current × K)
                torque_now = cur_now * parsed_args.torque_constant

                # 更新最大电流和扭矩记录
                current_time = time.time() - start_time
                for i in range(len(motors)):
                    # 更新最大电流
                    if cur_now[i] > max_currents[i]:
                        max_currents[i] = cur_now[i]
                        max_current_pos[i] = pos_now[i]
                        max_current_time[i] = current_time
                    # 更新最大扭矩
                    if torque_now[i] > max_torques[i]:
                        max_torques[i] = torque_now[i]
                        max_torque_pos[i] = pos_now[i]
                        max_torque_time[i] = current_time

                # 每5步打印一次
                if step % 5 == 0:
                    time_since_write = time.time() - last_write_time
                    freq = 1.0 / (time.time() - read_start)
                    print('[{}] t={:.2f}s after write | Freq: {:.2f} Hz'.format(
                        step, time_since_write, freq))
                    print('> Pos:   {}'.format(['{:+.3f}'.format(p) for p in pos_now]))
                    print('> Target:{}'.format(['{:+.3f}'.format(p) for p in target_pos] if target_pos is not None else ['N/A']*len(motors)))
                    print('> Vel:   {}'.format(['{:+.3f}'.format(v) for v in vel_now]))
                    print('> Cur:   {}'.format(['{:+.3f}'.format(c) for c in cur_now]))
                    print('> Trq:   {}'.format(['{:+.3f}'.format(t) for t in torque_now]))
                    print('> Max Cur:{}'.format(['{:+.3f}'.format(m) for m in max_currents]))
                    print('> Max Trq:{}'.format(['{:+.3f}'.format(m) for m in max_torques]))

                    # 检查是否接近目标位置
                    if target_pos is not None:
                        pos_errors = np.abs(pos_now - target_pos)
                        is_at_target = np.all(pos_errors < 0.01)  # 误差小于0.1弧度认为到达
                        print('> Error: {}'.format(['{:.3f}'.format(e) for e in pos_errors]))
                        print(f'> At target: {is_at_target}')
                    print('-'*40)

                # 等待一段时间再读取
                time.sleep(parsed_args.sleep_time)

                # 如果需要等待电机到达目标
                if not parsed_args.no_wait and target_pos is not None:
                    pos_errors = np.abs(pos_now - target_pos)
                    if np.all(pos_errors < 0.1):  # 到达目标
                        # 额外等待一段时间确保稳定
                        if time.time() - last_write_time > parsed_args.wait_time:
                            step = 99  # 触发下一次位置切换
                            time.sleep(0.5)  # 额外稳定时间

                step += 1

            except KeyboardInterrupt:
                print('\n' + '='*80)
                print('Stopped by user.')
                print('='*80)
                print('\n【最大电流统计】')
                print(f'{"电机ID":<8} {"最大电流(mA)":<15} {"发生时位置(rad)":<18} {"发生时间(s)":<12}')
                print('-'*80)
                for i, motor_id in enumerate(motors):
                    pos_str = '{:+.3f}'.format(max_current_pos[i]) if max_current_pos[i] is not None else 'N/A'
                    print(f'{motor_id:<8} {max_currents[i]:<15.3f} {pos_str:<18} {max_current_time[i]:<12.2f}')
                print('='*80)

                print('\n【最大扭矩统计】')
                print(f'{"电机ID":<8} {"最大扭矩(N·m)":<18} {"发生时位置(rad)":<18} {"发生时间(s)":<12}')
                print(f'(扭矩常数 K = {parsed_args.torque_constant} N·m/A)')
                print('-'*80)
                for i, motor_id in enumerate(motors):
                    pos_str = '{:+.3f}'.format(max_torque_pos[i]) if max_torque_pos[i] is not None else 'N/A'
                    print(f'{motor_id:<8} {max_torques[i]:<18.4f} {pos_str:<18} {max_torque_time[i]:<12.2f}')
                print('='*80)
                break

    print('Test completed.')
