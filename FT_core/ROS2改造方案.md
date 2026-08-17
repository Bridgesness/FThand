# OrcaHand 遥操作 ROS2 改造方案

> 把 `teleoperation.py`（单进程大程序）改造成 ROS2 多节点架构。
> 路线：**Phase 1 先在单台 PC 上跑通 ROS2 版**（3 节点同机）→ **Phase 2 整机迁移到 RK3588**。
> 改造原则：**核心算法（映射/平滑/orca_core）几乎不改，只把"主循环/存变量"换成"publish/callback"**。

---

## 0. 一句话总览

```
现在:  teleoperation.py 一个进程干完 [收UDP → 映射 → 平滑 → 控手]
改后:  glove_driver_node ──/glove/data──> retargeter_node ──/hand/joint_targets──> hand_controller_node
       (3个节点, 用一条 ros2 launch 一键拉起, 用的时候还是单终端单命令)
```

**为什么拆 3 个节点（而不是像雷达驱动那样 1 个）**：中间的 `retargeter_node` 是**可插拔**的——将来想用 RL 策略/别的映射算法替换规则映射，只换中间节点，数据源（glove_driver）和硬件（hand_controller）不动。这就是模块化的价值。

> ⚠ 澄清"用时开几个终端"：不管几个节点，**一条 `ros2 launch` 命令在单终端里把 3 个节点全拉起来**。节点数对你"用"没影响。调试时另开终端跑 `ros2 topic echo` 是可选。

---

## 1. 现状架构分析

### 1.1 teleoperation.py 的 4 阶段（全在一个进程）

```
手套 ─UDP8000→ ①UDEGloveSDK(收+解析) → ②GloveToHandMapper(映射) → ③SmoothController(平滑) → ④OrcaHand.set_joint_pos(发舵机)
```

对应代码（行号）：
| 阶段 | 类/函数 | 位置 |
|---|---|---|
| ① 收UDP+解析 | `UDEGloveSDK._recv_func/_process_data/_parse_finger_data` | 行60-296 |
| ② 映射 | `GloveToHandMapper.map_glove_to_hand` | 行391-696 |
| ③ 平滑 | `SmoothController.smooth` | 行703-727 |
| ④ 控手 | `TeleoperationController._update` 里 `set_joint_pos` | 行918-932 |
| 主循环 | `TeleoperationController.start` 的 60Hz while | 行844-885 |

### 1.2 注意：项目里还有一套平行的 HTTP 实现

`orca_core/api/teleop_api.py` 是 **FastAPI HTTP 版**遥操作（`/teleop/start` 等端点），用的是**更简单的 6 传感器映射**（`UDPGloveReceiver`，硬编码 `LEFT_FINGER_INDICES`），50Hz。

- 本方案**只改 `teleoperation.py`**（它的 yaml 映射更精细、可调）。
- 改造时**别和 teleop_api 重复封装**——ROS2 节点直接复用 `GloveToHandMapper`，不要另写一套映射。
- （将来若想统一，ROS2 的 hand_controller_node 可以替代 teleop_api 的控手部分。）

---

## 2. 目标 ROS2 架构（3 节点 + 2 topic）

```
┌─────────────────────┐   /glove/data    ┌──────────────────┐  /hand/joint_targets  ┌───────────────────────┐
│ glove_driver_node   │ ───────────────→ │ retargeter_node  │ ────────────────────→ │ hand_controller_node  │
│ - UDP收8000         │  Float64MultiArr │ - GloveToHandMapper│  sensor_msgs/JointState│ - OrcaHand.set_joint_pos│
│ - 解析JSON          │                  │ - SmoothController│                       │ - num_steps=1         │
│ - publish 手套数据   │                  │ - publish 关节目标│                       │ - 接真硬件             │
└─────────────────────┘                  └──────────────────┘                       └───────────────────────┘
   纯发布者                                  订阅+发布(可插拔)                            纯订阅者
```

| 节点                     | 订阅                    | 发布                    | 复用现有代码                                                    | 必须跑在哪        |
| ---------------------- | --------------------- | --------------------- | --------------------------------------------------------- | ------------ |
| `glove_driver_node`    | —                     | `/glove/data`         | `UDEGloveSDK`（行60-296）+ `scripts/monitor_glove_udp.py` 模板 | 手套 UDP 发到的机器 |
| `retargeter_node`      | `/glove/data`         | `/hand/joint_targets` | `GloveToHandMapper` + `SmoothController`（原封复用）            | 任意（纯计算）      |
| `hand_controller_node` | `/hand/joint_targets` | —                     | `OrcaHand.set_joint_pos(num_steps=1)`                     | **物理连着手的机器** |

**代码复用原则**：mapper/smoother/UDEGloveSDK **逻辑不改**，只把"解析完存 `self.xxx`"换成"`self.pub.publish(msg)`"，把"60Hz while 循环"删掉换成"callback 触发"。

---

## 3. 消息设计（用标准消息，零额外配置）

### 3.1 `/glove/data` → `std_msgs/Float64MultiArray`

布局（共 90 个 float，左右手各 45）：

```
data[0..44]   = 左手 15 个 Vector3Float，每个3分量(x,y,z) 摊平: [x0,y0,z0, x1,y1,z1, ..., x14,y14,z14]
data[45..89]  = 右手 15 个 Vector3Float 同上
```

> 即 `UDEGloveSDK.left_finger_data`/`right_finger_data`（各 15 个 Vector3Float）直接摊平。retargeter 端 reshape 回 `List[Vector3Float]` 喂给 mapper。
> 若要用控制器（摇杆/按键），再追加 12 个 float（l_ctrl6 + r_ctrl6），布局写进注释即可。

### 3.2 `/hand/joint_targets` → `sensor_msgs/JointState`

```python
JointState(
  name     = ['thumb_mcp','thumb_abd',...,'wrist'],   # 17个，对齐 OrcaHand.joint_ids
  position = [角度度数, ...],                          # 单位：度（与 config.yaml joint_roms 一致）
  header.stamp = now()
)
```

> 用 JointState 的好处：带关节名，hand_controller 端 `dict(zip(name,position))` 直接还原成 mapper 输出的 dict。

---

## 4. 代码映射（现有类 → 新节点）

| 现有                                               | 去向                              | 改动                                             |
| ------------------------------------------------ | ------------------------------- | ---------------------------------------------- |
| `UDEGloveSDK`                                    | glove_driver_node 内部            | 解析结果从存 `self.right_finger_data` 改为 `publish`   |
| `GloveToHandMapper`                              | retargeter_node 内部              | **不改**，照常 `map_glove_to_hand(glove_data)`      |
| `SmoothController`                               | retargeter_node 内部              | **不改**，照常 `.smooth(target)`                    |
| `OrcaHand.set_joint_pos`                         | hand_controller_node 的 callback | 从 60Hz 循环里抽出来，`num_steps=1`                    |
| `TeleoperationController`（60Hz while + stdin 命令） | **删除**                          | 数据驱动（callback 链）替代主循环；stdin 调参改成 ROS2 参数（见 §7） |

**先做一次小重构（强烈建议）**：把 `teleoperation.py` 里的 `UDEGloveSDK`、`GloveToHandMapper`、`SmoothController`、`load_full_glove_config`、`Vector3Float` 抽到一个共享模块（如 `orca_core/teleop_lib.py`），让 `teleoperation.py` 和 ROS2 节点都能 import。这样节点代码里 `from orca_core.teleop_lib import GloveToHandMapper, SmoothController, UDEGloveSDK, Vector3Float` 即可，不复制粘贴代码。

---

## 5. Phase 1：单台 PC 改造（详细步骤）

> 目标：3 个节点同机跑，功能等价于现在 `python teleoperation.py`。先用 MockOrcaHand 联调整条链，再接真手。

### 5.0 ⭐ WSL2 环境准备（你的 PC 是 Windows，必读）

> 你 PC 是 Windows，ROS2 跑在 WSL2（跑 lidar 那套）。FThand 改造**复用同一套 WSL2 环境**，但比 lidar 多两个硬件穿透：**手的 USB-串口** + **手套的 UDP**。建议**先用 MockOrcaHand + 假数据把整条链跑通，再分头接硬件**，别一上来两个硬件一起调。

**① 代码放 WSL 的 `~`，别在 `/mnt/c`**

```bash
cp -r /mnt/c/Users/haoyuanwu/Desktop/wanren/FThand ~/FThand   # 复制进 Linux 文件系统
cd ~/FThand/FThand/FT_core
```

> 原因（lidar 踩过）：`/mnt/c` 上 `colcon build` 慢、SQLite/串口有坑。所有 ROS2 活儿在 `~` 做。

**② ROS2 Humble 已就绪（lidar 装过，不用重装）**

- 每开新终端确保 `source /opt/ros/humble/setup.bash`（你 `.bashrc` 已加，自动）。

**③ 依赖补齐（WSL 里 pip 装）**

```bash
pip install pyserial pyyaml numpy        # ★pyserial 必装(飞特HL client 依赖，原 pyproject 漏了)
pip install dynamixel-sdk                # 仅当用 dynamixel 舵机；当前飞特 HL 可跳过
```

**④ 手的 USB 穿透（usbipd，和雷达同套工具，换设备 BUSID）**

- 灵巧手是 USB-串口设备（CH343 芯片，Windows 下即 COM16）。WSL 要用必须 usbipd 穿透：
  
  ```powershell
  usbipd list                                    # 找 USB-Serial(CH343) 的 BUSID（不是雷达那个 4-1！）
  usbipd bind --force --busid <手串口的BUSID>
  usbipd attach --wsl --busid <手串口的BUSID>     # WSL 窗口要先开着
  ```
- WSL 里 `ls /dev/ttyUSB*` 看到号，改 config.yaml 的 `port`。
- **同雷达：每次拔插/重启要重 attach**；嫌烦用 `--auto-attach`。
- ⭐ **初期开发先别接真手**——hand_controller 设 `mock:True` 用 MockOrcaHand，绕开这步。

**⑤ 手套 UDP 的 WSL2 NAT 坑（关键，lidar 没遇到这个）**

- 手套服务把 UDP 发到 8000 端口。WSL2 默认 NAT → **发到 Windows 主机的包进不了 WSL**。
- 解法：开 **WSL2 镜像网络模式**。`C:\Users\haoyuanwu\.wslconfig`：
  
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  
  然后 PowerShell `wsl --shutdown` 重进。WSL 共用宿主机 IP，手套发到 `<主机IP>:8000` 就能被 WSL 的 glove_driver 收到。
- 验证：WSL 里 `nc -lu 0.0.0.0 8000` 监听，手套发包，看 WSL 收不收到。

**⑥ config.yaml 端口切换**

- WSL 里改 `port: /dev/ttyUSB0`（原 Windows `COM16`）；`baudrate`/`client_type: hl` 不变。拔插后 ttyUSB 号可能变，`ls /dev/ttyUSB*` 确认。

**⑦ 推荐开发顺序（把三件事分开 debug，别一次全接）**

1. **纯软件联调**（不接任何硬件）：hand_controller 设 `mock:True`；再写个"假 glove publisher"小节点定时发随机 `/glove/data` → 跑通 3 节点 publish/callback 链（验证架构）。
2. **接手套**：开镜像网络，glove_driver 收真手套 UDP，看 `/glove/data` 有数据、`/hand/joint_targets` 随手势变（mock 手打印目标角）。
3. **接真手**：usbipd 穿透手的 USB，hand_controller 设 `mock:False`，手指随手套弯。

> 这样把「WSL 环境 / ROS2 架构 / 两个硬件穿透」分开验证，不会卡死。lidar 那套经验（usbipd 用法、~ 下干活、WSLg）这里直接复用。

### 5.1 建包

```bash
cd C:\Users\haoyuanwu\Desktop\wanren\FThand\FThand\FT_core
ros2 pkg create --build-type ament_python orcahand_teleop_ros2 \
  --dependencies rclpy std_msgs sensor_msgs
```

> 包放 `FT_core/orcahand_teleop_ros2/`，这样能 `from orca_core.core import OrcaHand`（FT_core 在 sys.path）。
> 把上面 §4 说的 `teleop_lib.py` 共享模块先建好（或临时从 teleoperation.py import）。

### 5.2 写 3 个节点（代码骨架）

#### ① `glove_driver_node.py`（发布者）

```python
import rclpy, socket, threading
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from orca_core.teleop_lib import UDEGloveSDK   # 复用，几乎不改

class GloveDriverNode(Node):
    def __init__(self):
        super().__init__('glove_driver')
        self.pub = self.create_publisher(Float64MultiArray, '/glove/data', 10)
        self.glove = UDEGloveSDK(port=self.declare_parameter('glove_port', 8000).value)
        self.glove.initialize()
        self.glove.start_listening()
        # 60Hz 定时器把最新手套数据发出去（原 UDEGloveSDK 是被动存，这里主动 publish）
        self.create_timer(1.0/60.0, self._publish)
        self.get_logger().info('glove_driver 已启动，发 /glove/data')

    def _publish(self):
        left = self.glove.get_finger_data('left')    # List[Vector3Float]×15
        right = self.glove.get_finger_data('right')
        flat = []
        for v in left:  flat += [v.x, v.y, v.z]
        for v in right: flat += [v.x, v.y, v.z]
        self.pub.publish(Float64MultiArray(data=flat))

def main():
    rclpy.init(); rclpy.spin(GloveDriverNode()); rclpy.shutdown()
```

#### ② `retargeter_node.py`（订阅 + 发布，可插拔）

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from orca_core.teleop_lib import GloveToHandMapper, SmoothController, Vector3Float
from orca_core.core import OrcaHand   # 只为拿 joint_roms/neutral/model_path

class RetargeterNode(Node):
    def __init__(self):
        super().__init__('retargeter')
        self.declare_parameter('hand', 'right')
        self.declare_parameter('model_path', 'orca_core/models/orcahand_v1_right')
        self.declare_parameter('motion_scale', 0.8)
        self.declare_parameter('smoothing', 0.3)

        hand = self.get_parameter('hand').value
        model_path = self.get_parameter('model_path').value
        # OrcaHand 只用来加载模型参数(joint_roms/neutral)，不 connect
        ref = OrcaHand(model_path=model_path)
        self.mapper = GloveToHandMapper(ref, hand=hand)
        self.mapper.motion_scale = self.get_parameter('motion_scale').value
        self.smoother = SmoothController(self.get_parameter('smoothing').value)

        self.create_subscription(Float64MultiArray, '/glove/data', self._cb, 10)
        self.pub = self.create_publisher(JointState, '/hand/joint_targets', 10)
        self.get_logger().info('retargeter 已启动（可插拔：换这里=换算法）')

    def _cb(self, msg):
        # 把 90 个 float reshape 回 List[Vector3Float]×2
        left = [Vector3Float(msg.data[i*3], msg.data[i*3+1], msg.data[i*3+2]) for i in range(15)]
        right= [Vector3Float(45 + i*3, 45 + i*3+1, 45 + i*3+2) for i in range(15)]  # 用值
        right= [Vector3Float(msg.data[45+i*3], msg.data[45+i*3+1], msg.data[45+i*3+2]) for i in range(15)]
        glove_data = right if self.mapper.hand == 'right' else left
        target = self.mapper.map_glove_to_hand(glove_data)
        smooth = self.smoother.smooth(target)
        js = JointState()
        js.name = list(smooth.keys())
        js.position = list(smooth.values())
        self.pub.publish(js)

def main():
    rclpy.init(); rclpy.spin(RetargeterNode()); rclpy.shutdown()
```

#### ③ `hand_controller_node.py`（订阅，控硬件）

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from orca_core.core import OrcaHand, MockOrcaHand   # 无硬件时换 MockOrcaHand

class HandControllerNode(Node):
    def __init__(self):
        super().__init__('hand_controller')
        self.declare_parameter('model_path', 'orca_core/models/orcahand_v1_right')
        self.declare_parameter('mock', False)   # ★ 无硬件联调时设 true

        mp = self.get_parameter('model_path').value
        cls = MockOrcaHand if self.get_parameter('mock').value else OrcaHand
        self.hand = cls(model_path=mp)
        ok, msg = self.hand.connect()
        if not ok:
            self.get_logger().error(f'连手失败: {msg}'); raise SystemExit
        self.hand.enable_torque()
        self.hand.set_control_mode('current_based_position')
        self.create_subscription(JointState, '/hand/joint_targets', self._cb, 10)
        self.get_logger().info('hand_controller 已启动，等 /hand/joint_targets')

    def _cb(self, msg):
        angles = dict(zip(msg.name, msg.position))
        self.hand.set_joint_pos(angles, num_steps=1)   # ★num_steps=1，别>1(会阻塞)

def main():
    rclpy.init(); rclpy.spin(HandControllerNode()); rclpy.shutdown()
```

### 5.3 setup.py 的 entry_points（让 3 节点能 `ros2 run`）

```python
entry_points={
    'console_scripts': [
        'glove_driver = orcahand_teleop_ros2.glove_driver_node:main',
        'retargeter = orcahand_teleop_ros2.retargeter_node:main',
        'hand_controller = orcahand_teleop_ros2.hand_controller_node:main',
    ],
},
```

### 5.4 补依赖（重要）

`pyproject.toml` 或包内说明需补：

- `rclpy`（ROS2 自带，不用 pip）
- **`pyserial`**（飞特 HL client 实际依赖，但原 pyproject 漏了——必补）
- 已有：`dynamixel-sdk`、`pyyaml`、`numpy`

### 5.5 写单机 launch（一键起 3 节点）

`launch/teleop.launch.py`：

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='orcahand_teleop_ros2', executable='glove_driver',  name='glove_driver'),
        Node(package='orcahand_teleop_ros2', executable='retargeter',    name='retargeter',
             parameters=[{'hand':'right','model_path':'orca_core/models/orcahand_v1_right'}]),
        Node(package='orcahand_teleop_ros2', executable='hand_controller',name='hand_controller',
             parameters=[{'model_path':'orca_core/models/orcahand_v1_right','mock': True}]),  # 先 mock
    ])
```

### 5.6 编译 + 跑

```bash
cd C:\Users\haoyuanwu\Desktop\wanren\FThand\FThand\FT_core
colcon build --packages-select orcahand_teleop_ros2
source install/setup.bash
ros2 launch orcahand_teleop_ros2 teleop.launch.py
```

### 5.7 验证（先 Mock，再真手）

1. **Mock 联调**（launch 里 `mock:True`）：戴手套动手指，另开终端 `ros2 topic echo /hand/joint_targets`——角度随手势变 = 数据链通；MockOrcaHand 会在日志打印目标角。
2. **接真手**：launch 里 `mock:False`，确认手接在 COM16（Windows）/ `/dev/ttyUSB0`（Linux），重跑 launch，手指应随手套弯。
3. **录制离线调**：`ros2 bag record /glove/data` 录一段手势，再 `ros2 bag play` 回放——不用一直戴手套也能调 retargeter 参数。

---

## 6. Phase 2：迁移到 RK3588（整机搬迁）

> 因为已经 3 节点化，迁移 = 把同一份包+orca_core 拷到 RK3588 跑。代码零改动，只改硬件相关配置。

### 6.1 RK3588 环境（Ubuntu 22.04 + ROS2 Humble，ARM64）

```bash
# RK3588 上
sudo apt install ros-humble-desktop python3-colcon-common-extensions
pip install pyserial pyyaml numpy dynamixel-sdk   # 补依赖（pyserial 必装）
```

### 6.2 拷贝代码

把 `FT_core/orcahand_teleop_ros2/` + `FT_core/orca_core/` 整个拷到 RK3588（U盘/scp/git 均可）。

### 6.3 改硬件配置（关键）

编辑 `orca_core/models/orcahand_v1_right/config.yaml`：

```yaml
port: /dev/ttyUSB0     # Linux（原 Windows 是 COM16）
baudrate: 1000000      # 不变
client_type: hl        # 不变（飞特）
```

> 端口号插拔会变，`ls /dev/ttyUSB*` 确认实际号。加串口权限：`sudo usermod -aG dialout $USER`。

### 6.4 接硬件

- **灵巧手**：飞特舵机总线的 USB-串口线插到 **RK3588**。
- **手套**：把手套服务的 UDP 目标 IP 改成 **RK3588 的 IP**（端口仍 8000）。

### 6.5 跑（和 Phase 1 完全一样）

```bash
cd ~/FT_core
colcon build --packages-select orcahand_teleop_ros2
source install/setup.bash
ros2 launch orcahand_teleop_ros2 teleop.launch.py   # mock 设 False
```

### 6.6 可选：跨机器分布（3 节点化的 bonus）

若以后想让 glove_driver 留 PC、retargeter+hand_controller 在 RK3588：

- PC 跑 glove_driver，RK3588 跑另两个，`/glove/data` 靠 DDS 跨机传。
- 条件：两机同 ROS_DOMAIN_ID（默认0）+ 同局域网组播通（参考 S2L 项目 WSL2 镜像模式经验）。
- **代码零改动**，只是 launch 在不同机器上起不同节点。

---

## 7. 参数与调参改造（把 stdin 命令换成 ROS2 参数）

原 `teleoperation.py` 的运行时 stdin 命令 → ROS2 化：
| 原命令 | 原"作用" | ROS2 做法 |
|---|---|---|
| `s<数>` (motion_scale) | 调整体幅度 | retargeter 的 ROS2 参数 `motion_scale`，`ros2 param set /retargeter motion_scale 1.0` 动态生效 |
| `m<数>` (smoothing) | 调平滑 | retargeter 参数 `smoothing` |
| `f<关节> <数>` | 单关节 factor | retargeter 参数（或 service），改 mapper.glove_mapping[joint]['factor'] |
| `p`/`d` (诊断) | 打印 raw→norm→angle | retargeter 参数 `debug`，或单独 diagnostic 节点订阅 `/glove/data` |
| `c` (量程探测) | 探测手套量程 | 独立 service 节点（可选，Phase 后期） |
| `n` (归中) | 回中性位 | hand_controller 的 service `/return_to_neutral` |

> Phase 1 先把 `motion_scale`/`smoothing` 做成 ROS2 参数（最常用），其余保留代码逻辑后续迁移。

---

## 8. 风险与注意事项

| 风险                              | 说明                                                         | 对策                                            |
| ------------------------------- | ---------------------------------------------------------- | --------------------------------------------- |
| `pyserial` 缺失                   | 原 pyproject.toml 没列，飞特 HL client 实际依赖                      | 改造时必补进依赖                                      |
| `set_joint_pos(num_steps>1)` 阻塞 | 内部 sleep，会卡住 callback                                      | 遥操一律 `num_steps=1`                            |
| 端口随插拔变                          | COM16/ttyUSB0 会变                                           | 启动前 `ls /dev/ttyUSB*` 或设备管理器确认，写进 config.yaml |
| HLClient 低电流 sleep              | `write_desired_pos` 低电流时主动 sleep 模拟软运动，影响频率                | 控制频率别设太高，60Hz 左右观察                            |
| 左右 glove_mapping.yaml 格式不同      | 右手扁平/左手紧凑                                                  | `load_full_glove_config` 已双兼容，直接复用，别自己解析      |
| MockOrcaHand 联调                 | 无硬件时                                                       | hand_controller 加 `mock` 参数，切 MockOrcaHand    |
| 60Hz 定时器 vs callback            | glove_driver 用定时器主动发；retargeter/hand_controller 用 callback | 数据驱动为主，glove_driver 定时器保证稳定帧率                 |

---

## 9. 验证清单

### Phase 1（单 PC = Windows + WSL2）

- [ ] **WSL 环境**：FThand 复制到 `~/FThand`（不在 /mnt/c）；`pip install pyserial` 装好
- [ ] **镜像网络**：`.wslconfig` 已设 `networkingMode=mirrored`，`wsl --shutdown` 重进过
- [ ] **手套 UDP 进 WSL**：WSL 里 `nc -lu 0.0.0.0 8000` 能收到手套包（验证镜像模式生效）
- [ ] `colcon build` 成功，`ros2 pkg list | grep orcahand` 能看到
- [ ] `ros2 launch orcahand_teleop_ros2 teleop.launch.py` 一键起 3 节点（单终端）
- [ ] `ros2 node list` 看到 `/glove_driver` `/retargeter` `/hand_controller`
- [ ] `ros2 topic list` 看到 `/glove/data` `/hand/joint_targets`
- [ ] **纯软件联调**（mock 手 + 假 glove publisher）：3 节点链通，`/hand/joint_targets` 随假数据变
- [ ] **接手套**（mock 手）：戴手套，`ros2 topic echo /hand/joint_targets` 角度随手势变
- [ ] **手的 USB 穿透**：`usbipd attach` 后 WSL 有 `/dev/ttyUSB0`，config.yaml 端口改对
- [ ] **接真手**（mock:False）：手指随手套弯曲
- [ ] `ros2 bag record /glove/data` 录制 + `ros2 bag play` 回放，retargeter 正常出目标

### Phase 2（RK3588）

- [ ] RK3588 上 `colcon build` 成功（ARM64 原生）
- [ ] config.yaml `port` 改成 `/dev/ttyUSB0`，`dialout` 权限已加
- [ ] 手接 RK3588，`ros2 launch` 后手指随手套动
- [ ] 手套 UDP 能发到 RK3588:8000（`ros2 topic hz /glove/data` 有频率）

---

## 10. 关键文件路径速查

- 待改造主文件：`FThand\FThand\FT_core\teleoperation.py`
- OrcaHand 类：`FThand\FThand\FT_core\orca_core\core.py`
- 手套监听模板：`FThand\FThand\FT_core\scripts\monitor_glove_udp.py`
- 平行 HTTP 实现（别重复）：`FThand\FThand\FT_core\orca_core\api\teleop_api.py`
- 右手模型配置：`FThand\FThand\FT_core\orca_core\models\orcahand_v1_right\config.yaml`
- 手套映射：`...\orcahand_v1_right\glove_mapping.yaml`（扁平）/ `...\orcahand_v1_left\glove_mapping.yaml`（紧凑）
- 新包将建于：`FThand\FThand\FT_core\orcahand_teleop_ros2\`

---

*本方案聚焦 `teleoperation.py` 的 ROS2 化；核心算法复用、节点化封装、两阶段落地。Phase 1 先在 PC 跑通（Mock→真手），Phase 2 整机搬 RK3588（改端口+权限）。*
