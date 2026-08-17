# scripts 目录功能详解

## 概述

`scripts/` 目录包含了 FT_core 项目的所有实用脚本，用于初始化、控制、测试和演示灵巧手。

---

## 脚本分类

### 一、初始化与校准脚本

| 脚本 | 功能 | 使用场景 |
|------|------|----------|
| `tension.py` | 张紧绳索 | 新组装或绳索松动后使用 |
| `calibrate.py` | 自动校准 | 首次使用或重新校准关节 |
| `neutral.py` | 移动到中性位置 | 日常使用，将手放在标准姿态 |
| `zero.py` | 移动到零位 | 将所有关节归零 |

#### 1.1 `tension.py` - 张紧绳索
```bash
python scripts/tension.py orca_core/models/orcahand_v1_right
python scripts/tension.py orca_core/models/orcahand_v1_right --move_motors
```
- **功能**: 使电机保持当前位置，方便手动张紧绳索
- **参数**: `--move_motors` - 先让电机正转3秒建立初始张力
- **工作流程**:
  1. 连接手
  2. 使能扭矩
  3. 设置为电流位置控制模式
  4. 按住 Ctrl+C 保持，手动张紧绳索
  5. 释放后自动失能

#### 1.2 `calibrate.py` - 自动校准
```bash
python scripts/calibrate.py orca_core/models/orcahand_v1_right
```
- **功能**: 自动校准所有关节，记录电机限位和传动比
- **工作流程**:
  1. 按 `config.yaml` 中的 `calib_sequence` 顺序逐个校准
  2. 每个关节先 flex（弯曲）到极限，再 extend（伸展）到极限
  3. 计算关节-电机传动比
  4. 保存到 `calibration.yaml`

#### 1.3 `neutral.py` - 中性位置
```bash
python scripts/neutral.py orca_core/models/orcahand_v1_right
```
- **功能**: 将手移动到配置的中性位置
- **用途**: 日常使用的标准起始姿态

#### 1.4 `zero.py` - 零位
```bash
python scripts/zero.py orca_core/models/orcahand_v1_right
```
- **功能**: 将所有关节移动到 0° 位置
- **用途**: 测试或统一参考位置

---

### 二、可视化控制脚本

| 脚本 | 功能 | 界面 |
|------|------|------|
| `slider_joint.py` | 关节空间滑块控制 | Tkinter GUI |
| `slider_motor.py` | 电机空间滑块控制 | Tkinter GUI |

#### 2.1 `slider_joint.py` - 关节控制
```bash
python scripts/slider_joint.py orca_core/models/orcahand_v1_right
```
- **功能**: GUI 滑块控制每个关节
- **特点**:
  - 滑块范围 = 关节活动范围 (ROM)
  - 实时显示关节角度
  - 使能/失能按钮

```
┌─────────────────────────────────────┐
│        Orca Hand Control            │
├─────────────────────────────────────┤
│ [Enable Torque] [Disable Torque]    │
├─────────────────────────────────────┤
│ thumb_mcp   |━━━━━━●━━━━| +12.5°    │
│ thumb_abd   |━━━━━━━━●━━| +35.0°    │
│ index_mcp   |━━●━━━━━━━━|   0.0°    │
│ ...                                 │
└─────────────────────────────────────┘
```

#### 2.2 `slider_motor.py` - 电机控制
```bash
python scripts/slider_motor.py orca_core/models/orcahand_v1_right
```
- **功能**: GUI 滑块直接控制电机
- **特点**:
  - 滑块范围 = 当前位置 ±1 弧度（精细控制）
  - 分辨率 0.1
  - 用于调试和测试单个电机

---

### 三、动作录制与回放脚本

| 脚本 | 功能 | 数据类型 |
|------|------|----------|
| `record_angles.py` | 录制关键帧姿势 | 离散 wayoints |
| `replay_angles.py` | 回放关键帧动作 | 插值平滑 |
| `record_continuous.py` | 连续录制关节角度 | 时间序列 |
| `replay_continuous.py` | 实时回放连续动作 | 原始频率 |

#### 3.1 `record_angles.py` - 关键帧录制
```bash
python scripts/record_angles.py orca_core/models/orcahand_v1_right
```
- **功能**: 手动录制一系列关键姿势
- **操作**:
  1. 手动调整手到目标姿势
  2. 按 Enter 捕获当前姿势
  3. 重复步骤 1-2
  4. Ctrl+C 结束录制
- **输出**: YAML 文件，包含 waypoints 列表

```yaml
waypoints:
  - [0, 0, 5, 10, ...]  # 第1个姿势
  - [10, 20, 30, 40, ...]  # 第2个姿势
  - ...
```

#### 3.2 `replay_angles.py` - 关键帧回放
```bash
python scripts/replay_angles.py orca_core/models/orcahand_v1_right --replay_file my_capture.yaml
python scripts/replay_angles.py orca_core/models/orcahand_v1_right --replay_file my_capture.yaml --step_time 0.01
```
- **功能**: 回放关键帧动作，waypoints 之间自动插值
- **参数**:
  - `--replay_file`: 录制文件路径
  - `--step_time`: 插值步长（默认 0.02s）
- **插值模式**: ease_in_out（平滑过渡）

#### 3.3 `record_continuous.py` - 连续录制
```bash
python scripts/record_continuous.py orca_core/models/orcahand_v1_right --frequency 50 --duration 10
```
- **功能**: 以固定频率连续录制关节角度
- **参数**:
  - `--frequency`: 采样频率（默认 50Hz）
  - `--duration`: 录制时长（不指定则手动停止）
- **输出**: 包含元数据和角度序列的 YAML

```yaml
metadata:
  type: continuous
  sampling_frequency_hz: 50.0
angles:
  - [0, 5, 10, ...]
  - [1, 6, 11, ...]
  ...
```

#### 3.4 `replay_continuous.py` - 连续回放
```bash
python scripts/replay_continuous.py orca_core/models/orcahand_v1_right --replay_file my_continuous.yaml
```
- **功能**: 按原始录制频率实时回放
- **特点**:
  - 自动验证手型匹配
  - 精确时间控制

---

### 四、测试与调试脚本

| 脚本 | 功能 | 使用场景 |
|------|------|----------|
| `check_motor.py` | 单电机测试 | 测试单个电机是否正常 |
| `main_demo.py` | 演示程序 | 展示手指波浪动作 |

#### 4.1 `check_motor.py` - 单电机测试
```bash
python scripts/check_motor.py --port COM12 --motor_id 1 --baudrate 1000000
python scripts/check_motor.py --port COM12 --motor_id 17 --wrist
```
- **功能**: 测试单个电机，每次移动 ±0.1 弧度
- **参数**:
  - `--port`: 串口
  - `--motor_id`: 电机 ID
  - `--baudrate`: 波特率
  - `--wrist`: 手腕电机用位置控制模式
  - `--reverse`: 反向移动

#### 4.2 `main_demo.py` - 手指波浪演示
```bash
python scripts/main_demo.py orca_core/models/orcahand_v1_right
```
- **功能**: 演示手指的波浪运动
- **特点**:
  - 四根手指依次弯曲/伸展（带相位差）
  - 拇指独立运动
  - Ctrl+C 退出并归零

---

### 五、其他脚本

| 脚本 | 功能 | 状态 |
|------|------|------|
| `calibrate_manual.py` | 手动校准 | 未实现 |
| `main_demo_abduction.py` | 外展演示 | - |
| `test.py` | 通用测试 | - |

---

## 典型使用流程

### 首次使用流程
```
1. tension.py           # 张紧绳索
2. calibrate.py         # 自动校准
3. neutral.py           # 移到中性位置
4. slider_joint.py      # 测试各关节
```

### 动作开发流程
```
1. neutral.py                          # 初始姿态
2. record_angles.py                    # 录制关键帧
3. replay_angles.py --replay_file xxx  # 测试回放
4. 调整 -> 重新录制
```

### 演示流程
```
1. neutral.py     # 初始姿态
2. main_demo.py   # 演示
3. zero.py        # 归零
```

---

## 输出文件存储位置

所有录制文件默认保存在:
```
G:\wanren\FThand\FT_core\replay_sequences\
```

文件命名格式:
- 关键帧: `{prefix}_replay_sequence_{timestamp}.yaml`
- 连续: `{prefix}_continuous_angles_{timestamp}.yaml`

---

## 参数说明

### 通用参数
所有脚本都支持:
```bash
python scripts/<script>.py [model_path]
```
- `model_path`: 模型配置文件夹路径，不填则使用默认

### 示例
```bash
# 使用默认路径
python scripts/neutral.py

# 指定路径
python scripts/neutral.py orca_core/models/orcahand_v1_right

# 使用相对路径
python scripts/neutral.py ../models/orcahand_v1_right
```
