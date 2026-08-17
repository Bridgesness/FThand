# OrcaHand 遥操作 ROS2 改造 —— 实操全步骤记录

> 这是**实际执行过**的改造全流程，每一步的命令、最终代码、踩过的坑都记下来，可复现。
> 与 `ROS2改造方案.md`（设计）互补：那份讲"为什么这么设计"，本份讲"具体怎么一步步做出来"。
> 环境：Windows + WSL2(Ubuntu22.04 + ROS2 Humble)，目标硬件 = OrcaHand 灵巧手（USB-串口）。

---

## 0. 架构回顾（3 节点 + 2 topic）

```
[fake_glove / glove_driver] ──/glove/data──> [retargeter] ──/hand/joint_targets──> [hand_controller] ──> 真手
   Float64MultiArray, 90 float                  GloveToHandMapper+SmoothController        OrcaHand.set_joint_pos
```

- 假手套 `fake_glove` 是**开发期占位**（无真手套时模拟数据）；接真手套后换成 `glove_driver`（收 UDP）。
- `retargeter` 是**可插拔**中间节点（以后换 RL 策略就换它）。
- `hand_controller` 必须跑在**连着手的机器**上。

---

## 1. 前置：WSL2 环境（复用 lidar 那套）

### 1.1 把 FThand 复制进 WSL `~`（不在 /mnt/c）

```bash
cp -r /mnt/c/Users/haoyuanwu/Desktop/wanren/FThand ~/FThand
```

> 原因：`/mnt/c` 上 `colcon build` 慢、串口/SQLite 有坑。

### 1.2 装依赖（用 apt，不用 pip）

```bash
sudo apt update
sudo apt install -y python3-serial python3-yaml python3-numpy
```

> `python3-serial` = pyserial（飞特 HL 舵机依赖，原 pyproject.toml 漏了，必装）。
> 为什么不用 pip：①WSL Ubuntu 22.04 默认没 pip；②pip 装 numpy 2.x 可能撞坏 ROS2 的 ABI；apt 装进系统 Python（rclpy 所在），最稳。

### 1.3 验证 orca_core 能 import

```bash
cd ~/FThand/FThand/FT_core
python3 -c "from orca_core.core import OrcaHand; print('orca_core 导入 OK')"
```

✅ 打印 OK 才继续。

---

## 2. Step 1：假手套发布器（验证 WSL+ROS2 环境，零硬件）

**目的**：没真手套时，发假 `/glove/data`（60Hz），把整条链跑通。

### 代码 `fake_glove_publisher.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假手套数据发布器（开发联调用）。发 /glove/data @60Hz。
数据布局 90 float = 左手45(15向量×xyz) + 右手45。模拟双手食指弯曲。"""
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class FakeGloveNode(Node):
    def __init__(self):
        super().__init__('fake_glove')
        self.pub = self.create_publisher(Float64MultiArray, '/glove/data', 10)
        self.timer = self.create_timer(1.0 / 60.0, self._publish)
        self.t0 = time.time()
        self.get_logger().info('假手套已启动，发 /glove/data @60Hz（模拟双手食指弯曲）。Ctrl+C 停。')

    def _publish(self):
        t = time.time() - self.t0
        flex = -50.0 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.4 * t))   # 0~-50，0.4Hz
        data = [0.0] * 90
        for offset in (0, 45):   # 左右手食指都动
            data[offset + 3 * 3 + 0] = flex   # Index1.x (食指 MCP)
            data[offset + 4 * 3 + 0] = flex   # Index2.x (食指 PIP)
            data[offset + 5 * 3 + 0] = flex   # Index3.x (食指 DIP)
        self.pub.publish(Float64MultiArray(data=data))

def main():
    rclpy.init()
    try: rclpy.spin(FakeGloveNode())
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 跑 + 验证

```bash
cd ~/FThand/FThand/FT_core
python3 fake_glove_publisher.py
# 另开终端：
ros2 topic hz /glove/data          # 应 ≈60Hz
ros2 topic echo /glove/data --once # 90 个 float
```

---

## 3. Step 2：retargeter 节点（/glove/data → /hand/joint_targets）

**目的**：用 teleoperation.py 里现成的 `GloveToHandMapper`+`SmoothController`，把假数据映射成 17 个关节角度。

### 代码 `retargeter_node.py`（含自诊断）

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""retargeter 节点：订阅 /glove/data → 映射+平滑 → 发 /hand/joint_targets。"""
import os, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

FT_CORE = os.environ.get('FT_CORE_PATH', os.path.expanduser('~/FThand/FThand/FT_core'))
if FT_CORE not in sys.path:
    sys.path.insert(0, FT_CORE)
from teleoperation import GloveToHandMapper, SmoothController, Vector3Float  # noqa
from orca_core.core import OrcaHand                                          # noqa

class RetargeterNode(Node):
    def __init__(self):
        super().__init__('retargeter')
        hand = self.declare_parameter('hand', 'right').value
        model_path = self.declare_parameter(
            'model_path', os.path.join(FT_CORE, 'orca_core/models/orcahand_v1_right')).value
        motion_scale = self.declare_parameter('motion_scale', 0.8).value
        smoothing = self.declare_parameter('smoothing', 0.3).value

        ref = OrcaHand(model_path=model_path)   # 只加载配置，不 connect
        self.mapper = GloveToHandMapper(ref, hand=hand)
        self.mapper.motion_scale = motion_scale
        self.smoother = SmoothController(smoothing)

        self.create_subscription(Float64MultiArray, '/glove/data', self._cb, 10)
        self.pub = self.create_publisher(JointState, '/hand/joint_targets', 10)

        self.hand = hand
        n = len(self.mapper.glove_mapping)
        idx_cfg = self.mapper.glove_mapping.get('index_mcp', {})
        self.get_logger().info(f'retargeter 已启动 hand={hand} scale={motion_scale} smooth={smoothing}')
        self.get_logger().info(
            f'  映射关节数={n}  index_mcp配置={"✓" if idx_cfg else "❌缺失(映射没加载!)"}  {idx_cfg}')

    def _cb(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < 90: return
        def to_vecs(offset):
            return [Vector3Float(d[offset+i*3], d[offset+i*3+1], d[offset+i*3+2]) for i in range(15)]
        left, right = to_vecs(0), to_vecs(45)
        glove_data = right if self.mapper.hand == 'right' else left

        target = self.mapper.map_glove_to_hand(glove_data)
        smooth = self.smoother.smooth(target)

        diag = self.mapper.last_diagnostics.get('index_mcp', {})
        self.get_logger().info(
            f'[diag] index_mcp  raw={diag.get("raw",0):6.1f}  norm={diag.get("normalized",0):.2f}  '
            f'angle={diag.get("angle",0):5.1f}°  ->  publish={smooth.get("index_mcp",0):5.1f}°',
            throttle_duration_sec=1.0)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(smooth.keys())
        js.position = list(smooth.values())
        self.pub.publish(js)

def main():
    rclpy.init()
    try: rclpy.spin(RetargeterNode())
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 跑 + 验证

```bash
# 终端1：假手套
python3 fake_glove_publisher.py
# 终端2：retargeter
python3 retargeter_node.py
# 终端3：验证
ros2 topic echo /hand/joint_targets --once   # 17 关节名 + position
```

✅ `index_mcp` / `index_pip` 的 position 周期变化 = 映射生效。

---

## 4. Step 3：hand_controller 节点（mock 模式）

**目的**：订阅 `/hand/joint_targets` → `set_joint_pos`。mock 模式纯打印（不碰硬件，绕开 MockOrcaHand 的 dynamixel_sdk 依赖）。

### 代码 `hand_controller_node.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hand_controller 节点：订阅 /hand/joint_targets → set_joint_pos。
mock=True 纯打印；mock=False 真手（需 USB 穿透）。"""
import os, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

FT_CORE = os.environ.get('FT_CORE_PATH', os.path.expanduser('~/FThand/FThand/FT_core'))
if FT_CORE not in sys.path:
    sys.path.insert(0, FT_CORE)

class HandControllerNode(Node):
    def __init__(self):
        super().__init__('hand_controller')
        self.mock = self.declare_parameter('mock', True).value
        if not self.mock:
            from orca_core.core import OrcaHand
            model_path = self.declare_parameter(
                'model_path', os.path.join(FT_CORE, 'orca_core/models/orcahand_v1_right')).value
            self.hand = OrcaHand(model_path=model_path)
            ok, msg = self.hand.connect()
            if not ok:
                self.get_logger().error(f'连接失败: {msg}'); raise SystemExit(1)
            self.hand.enable_torque()
            self.hand.set_control_mode('current_based_position')
            self.get_logger().info('真手已连接，扭矩已使能')
        else:
            self.hand = None
            self.get_logger().info('Mock 模式：只打印目标角度，不碰硬件/不依赖 dynamixel_sdk')
        self.create_subscription(JointState, '/hand/joint_targets', self._cb, 10)
        self.get_logger().info('hand_controller 已启动，等待 /hand/joint_targets ...')

    def _cb(self, msg: JointState):
        angles = dict(zip(msg.name, msg.position))
        if not self.mock:
            try: self.hand.set_joint_pos(angles, num_steps=1)   # ★绝不 >1(会阻塞)
            except Exception as e:
                self.get_logger().warn(f'set_joint_pos 失败: {e}', throttle_duration_sec=1.0); return
        idx = angles.get('index_mcp', 0.0)
        tag = '(mock仅显示)' if self.mock else '(已下发真手)'
        self.get_logger().info(f'收到目标 index_mcp={idx:.1f}° {tag}', throttle_duration_sec=2.0)

def main():
    rclpy.init()
    try: rclpy.spin(HandControllerNode())
    except KeyboardInterrupt: pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 跑 + 验证（三终端 mock）

```bash
python3 fake_glove_publisher.py      # T1
python3 retargeter_node.py           # T2
python3 hand_controller_node.py      # T3，每~2秒打印 收到目标 index_mcp=XX° (mock仅显示)
ros2 node list                       # /fake_glove /retargeter /hand_controller
```

---

## 5. Step 4：打包成正式 ROS2 包 + launch 一键起

### 5.1 建工作空间 + 包

```bash
mkdir -p ~/fthand_ws/src && cd ~/fthand_ws/src
ros2 pkg create --build-type ament_python orcahand_teleop_ros2 --dependencies rclpy std_msgs sensor_msgs
```

### 5.2 把 3 节点 + setup.py + launch 放进包

```bash
PKG=~/fthand_ws/src/orcahand_teleop_ros2
CORE=~/FThand/FThand/FT_core
cp $CORE/fake_glove_publisher.py $CORE/retargeter_node.py $CORE/hand_controller_node.py $PKG/orcahand_teleop_ros2/
cp $CORE/setup.py $PKG/setup.py
mkdir -p $PKG/launch
cp $CORE/single_pc.launch.py $PKG/launch/
```

### 5.3 `setup.py`（entry_points + 装载 launch）

```python
import os
from glob import glob
from setuptools import setup
package_name = 'orcahand_teleop_ros2'
setup(
    name=package_name, version='0.0.1', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='haoyuanwu', maintainer_email='haoyuanwu@example.com',
    description='OrcaHand teleop ROS2 nodes', license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'fake_glove = orcahand_teleop_ros2.fake_glove_publisher:main',
        'retargeter = orcahand_teleop_ros2.retargeter_node:main',
        'hand_controller = orcahand_teleop_ros2.hand_controller_node:main',
    ]},
)
```

### 5.4 `single_pc.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='orcahand_teleop_ros2', executable='fake_glove', name='fake_glove', output='screen'),
        Node(package='orcahand_teleop_ros2', executable='retargeter', name='retargeter',
             output='screen', parameters=[{'hand': 'right'}]),
        Node(package='orcahand_teleop_ros2', executable='hand_controller', name='hand_controller',
             output='screen', parameters=[{'mock': True}]),
    ])
```

### 5.5 编译 + source + 一键 launch

```bash
cd ~/fthand_ws
colcon build --packages-select orcahand_teleop_ros2 --symlink-install   # --symlink-install: 改代码免重编
source install/setup.bash
ros2 launch orcahand_teleop_ros2 single_pc.launch.py
# 写进 .bashrc 一劳永逸：echo "source ~/fthand_ws/install/setup.bash" >> ~/.bashrc
```

---

## 6. Step 5：接真手（usbipd 穿透 + 左手模型）

### 6.1 WSL2 镜像网络（接真手套时也要，先开着）

`C:\Users\haoyuanwu\.wslconfig`：

```ini
[wsl2]
networkingMode=mirrored
```

PowerShell `wsl --shutdown` 重进。

### 6.2 用 usbipd 把手的 USB-串口穿透进 WSL

**Windows PowerShell（管理员），WSL 窗口先开**：

```powershell
usbipd list                      # 找手的 CH343(1a86:55d3, COM16)，记 BUSID（实测 6-2）
usbipd bind --force --busid 6-2
usbipd attach --wsl --busid 6-2
```

**WSL 里**：

```bash
ls /dev/ttyUSB* /dev/ttyACM*     # CH343 在 Linux 认成 CDC ACM → /dev/ttyACM0（不是 ttyUSB！）
dmesg | tail -10                 # 看到 cdc_acm ... ttyACM0
```

### 6.3 改 config.yaml 端口（左/右两只手都改）

```bash
sed -i 's|^port:.*|port: /dev/ttyACM0|' ~/FThand/FThand/FT_core/orca_core/models/orcahand_v1_left/config.yaml
sed -i 's|^port:.*|port: /dev/ttyACM0|' ~/FThand/FThand/FT_core/orca_core/models/orcahand_v1_right/config.yaml
groups                           # 确认有 dialout（串口权限）
```

### 6.4 先单独测中性位（确认通信 + 用对模型）

`hand_neutral_test.py`（接哪只手就跑哪只模型，默认左手）：

```bash
python3 hand_neutral_test.py        # 默认 left；右手: python3 hand_neutral_test.py right
```

确认：能连上、读出关节位置、慢慢移到中性位不乱拧。

### 6.5 跑 teleop 驱动真左手（三终端）

```bash
LEFT_MODEL=/home/haoyuanwu/FThand/FThand/FT_core/orca_core/models/orcahand_v1_left

# T1
ros2 run orcahand_teleop_ros2 fake_glove
# T2
ros2 run orcahand_teleop_ros2 retargeter --ros-args -p hand:=left -p model_path:=$LEFT_MODEL
# T3
ros2 run orcahand_teleop_ros2 hand_controller --ros-args -p mock:=false -p model_path:=$LEFT_MODEL
```

✅ 食指来回弯 = 整条链用真硬件跑通。

---

## 7. 踩坑总结（按出现顺序）

| 现象                                                                                      | 原因                                              | 解法                                                                        |
| --------------------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------- |
| `pip: command not found`                                                                | WSL Ubuntu 22.04 没装 pip                         | 改用 `apt install python3-serial python3-yaml python3-numpy`                |
| `MockDynamixelClient has no attribute '_connected'` / `No module named 'dynamixel_sdk'` | MockOrcaHand 依赖 dynamixel_sdk 且有 bug            | mock 模式改成**纯打印**，不用 MockOrcaHand                                          |
| `Package 'orcahand_teleop_ros2' not found`                                              | 新终端没 source 工作空间                                | `source ~/fthand_ws/install/setup.bash`（写进 .bashrc）                       |
| `PackageNotFoundError: No package metadata`                                             | 切 `--symlink-install` 后 install 有 stale 元数据     | `rm -rf build install log` 干净重 build，再 source                             |
| `usbipd attach` 成功但 `/dev/ttyUSB*` 不存在                                                  | CH343 在 Linux 是 **CDC ACM**，设备名是 `/dev/ttyACM0` | config.yaml port 改 `/dev/ttyACM0`                                         |
| 真手 3 号舵机乱动                                                                              | **接左手却用了右手模型**（电机映射错位）                          | 用对模型：`orcahand_v1_left`，retargeter/hand_controller 传 `hand:=left` + 左模型路径 |
| `set_joint_pos` 卡住                                                                      | `num_steps>1` 内部 sleep 阻塞                       | 一律 `num_steps=1`                                                          |

---

## 8. 文件清单 + 当前状态

### 代码文件（最终版，都在 `~/FThand/FThand/FT_core/`）

| 文件                        | 作用             |
| ------------------------- | -------------- |
| `fake_glove_publisher.py` | 假手套发布器（开发占位）   |
| `retargeter_node.py`      | 映射节点（含自诊断）     |
| `hand_controller_node.py` | 控手节点（mock/真手）  |
| `hand_neutral_test.py`    | 真手中性位测试（接手验证用） |
| `setup.py`                | 包安装脚本          |
| `single_pc.launch.py`     | 单机 launch      |

### ROS2 包

`~/fthand_ws/src/orcahand_teleop_ros2/`（3 节点 + setup.py + launch，`--symlink-install` 编译）

### 当前完成度

- ✅ Step1-4：3 节点架构零硬件跑通 + 正式包 + launch
- ✅ Step5：接真左手，食指由 ROS2 驱动（mock=False）
- ⏭ Step6：接真手套（写真 glove_driver 节点套 UDEGloveSDK，开镜像网络）
- ⏭ Phase2：整机迁移 RK3588

---

## 9. 关键认知（给将来的自己）

1. **WSL2 干 ROS2 活儿在 `~` 下**，不在 `/mnt/c`。
2. **硬件穿透用 usbipd**：雷达(CP210x→ttyUSB0)、手(CH343→ttyACM0)，设备名不同。
3. **接哪只手用哪只模型**，电机映射左右不同，混用会乱动甚至憋舵机。
4. **`--symlink-install`** 开发期改 .py 免重编。
5. **每个新终端 source 工作空间**（或写进 .bashrc）。
6. **mock 先行**：先用假数据+mock 把架构跑通，再接硬件，分阶段 debug。

---

*本文档为实操记录，代码以 `~/FThand/...` 与 `~/fthand_ws/...` 中的实际文件为准。*
