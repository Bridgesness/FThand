# FT_core 项目结构梳理

## 项目概述

**ORCA Hand** 是一个开源的灵巧手项目，`FT_core` 是其核心控制包。

**功能**：抽象硬件层，提供校准和张紧脚本，通过简单的高级 API 在关节空间控制灵巧手。

---

## 目录结构总览

```
G:\wanren\FThand\FT_core\
│
├── orca_core/          # 核心代码包
│   ├── core.py         # 主控制类 OrcaHand
│   ├── api/            # FastAPI 服务器接口
│   ├── hardware/       # 硬件驱动层
│   │   ├── dynamixel_client.py    # Dynamixel 舵机驱动
│   │   ├── hl_client.py          # 飞特舵机驱动（新增）
│   │   ├── mock_dynamixel_client.py  # 模拟驱动（测试用）
│   │   └── ft_sdk/              # 飞特舵机 SDK
│   ├── models/         # 手型配置文件夹
│   │   └── orcahand_v1_right/   # 右手 v1 配置
│   │       ├── config.yaml      # 配置文件
│   │       └── calibration.yaml # 校准数据
│   └── utils/          # 工具函数
│
├── scripts/            # 实用脚本
├── tests/              # 单元测试
├── docs/               # 文档
├── demo/               # 演示数据
├── pyproject.toml      # 项目依赖配置
└── README.md           # 项目说明
```

---

## 各模块详细说明

### 1. `orca_core/` - 核心代码包

#### 1.1 `core.py` - OrcaHand 主类
**功能**：灵巧手的最高层抽象接口

| 方法 | 功能 |
|------|------|
| `connect()` / `disconnect()` | 连接/断开硬件 |
| `enable_torque()` / `disable_torque()` | 使能/失能电机扭矩 |
| `get_motor_pos()` / `get_joint_pos()` | 获取电机/关节位置 |
| `set_joint_pos()` | 设置关节位置（主要控制方法） |
| `calibrate()` | 自动校准 |
| `tension()` | 张紧绳索 |

#### 1.2 `api/` - FastAPI 服务器
**功能**：提供 HTTP API 接口，可通过网络控制灵巧手

```
api/api.py - FastAPI 应用入口
```

#### 1.3 `hardware/` - 硬件驱动层

| 文件 | 功能 |
|------|------|
| `dynamixel_client.py` | Dynamixel 舵机驱动（原版） |
| `hl_client.py` | 飞特舵机驱动（新增，兼容 Dynamixel 接口） |
| `mock_dynamixel_client.py` | 模拟驱动，用于无硬件测试 |
| `ft_sdk/` | 飞特舵机底层 SDK |

**`ft_sdk/` 结构**：
```
ft_sdk/
├── scservo_sdk/        # 飞特 SCS 舵机 SDK
│   ├── port_handler.py           # 串口处理
│   ├── protocol_packet_handler.py # 协议包处理
│   ├── group_sync_read.py        # 批量读取
│   ├── group_sync_write.py       # 批量写入
│   └── hls.py                    # HLS 系列舵机接口
└── hls/                # HLS 舵机辅助功能
```

#### 1.4 `models/` - 手型配置
```
models/orcahand_v1_right/
├── config.yaml         # 硬件配置
└── calibration.yaml    # 校准数据
```

**config.yaml 关键配置**：
```yaml
motor_ids: [1, 2, 3, ...]        # 电机 ID 列表
joint_ids: [thumb_mcp, ...]      # 关节名称列表
joint_to_motor_map:              # 关节→电机映射
joint_roms:                      # 关节活动范围
neutral_position:                # 中性位置
calib_sequence:                  # 校准序列
client_type: hl/dynamixel        # 客户端类型
port: COM12                      # 串口
baudrate: 1000000                # 波特率
```

#### 1.5 `utils/` - 工具函数
```python
utils/utils.py
├── get_model_path()     # 查找模型路径
├── read_yaml()         # 读取 YAML 配置
├── update_yaml()       # 更新 YAML 配置
└── ...                 # 其他辅助函数
```

---

### 2. `scripts/` - 实用脚本

| 脚本 | 功能 |
|------|------|
| `calibrate.py` | 自动校准所有关节 |
| `calibrate_manual.py` | 手动校准 |
| `tension.py` | 张紧绳索 |
| `neutral.py` | 移动到中性位置 |
| `zero.py` | 移动到零位 |
| `slider_joint.py` | 滑块控制关节 |
| `slider_motor.py` | 滑块控制电机 |
| `record_angles.py` | 记录关节角度序列 |
| `replay_angles.py` | 回放关节角度序列 |
| `record_continuous.py` | 连续记录 |
| `replay_continuous.py` | 连续回放 |
| `check_motor.py` | 检查电机状态 |
| `main_demo.py` | 主演示程序 |

**使用方式**：
```bash
python scripts/neutral.py orca_core/models/orcahand_v1_right
```

---

### 3. `tests/` - 单元测试

| 文件 | 测试内容 |
|------|----------|
| `test_calibration.py` | 校准功能测试 |
| `test_core.py` | 核心功能测试 |
| `test_tension.py` | 张紧功能测试 |
| `test_yaml.py` | 配置文件测试 |

---

### 4. `docs/` - 文档

```
docs/
├── pages/
│   ├── getting-started-docs/    # 入门教程
│   │   ├── initial-tensioning-and-calibration.md
│   │   ├── quickstart-with-core-package.md
│   │   ├── setting-up-config.md
│   │   └── setting-up-dynamixels.md
│   └── orca-core-docs/          # API 文档
│       ├── orca-core-scripts.md
│       ├── orca-core-structure.md
│       └── orcahand-api.md
└── img/                         # 图片资源
```

---

### 5. `demo/` - 演示数据
存储录制的动作序列数据

---

## 核心工作流程

```
┌─────────────────────────────────────────────────────────────┐
│                      用户代码 / 脚本                          │
│                    (scripts/*.py)                           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   OrcaHand 类 (core.py)                      │
│   - 提供高级 API: set_joint_pos(), calibrate() 等           │
│   - 管理 joint <-> motor 坐标转换                            │
│   - 加载 config.yaml 和 calibration.yaml                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              硬件驱动层 (hardware/)                          │
│   DynamixelClient / HLClient / MockDynamixelClient          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    物理舵机硬件                               │
│              Dynamixel XL430 / 飞特 HTS                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 坐标系统

### 1. 电机空间 (Motor Space)
- 单位：弧度 (rad)
- 直接来自舵机编码器
- 范围取决于电机型号

### 2. 关节空间 (Joint Space)
- 单位：度 (°)
- 人类可理解的角度
- 需要通过校准建立的映射关系转换

### 转换公式
```
joint_pos = rom[0] + (motor_pos - motor_limit[0]) / ratio
```

---

## 配置文件说明

### config.yaml（固定配置）
- 硬件连接信息（端口、波特率、电机 ID）
- 关节定义（名称、活动范围）
- 关节-电机映射
- 校准序列
- 控制参数

### calibration.yaml（运行时生成）
- 电机物理限位
- 关节-电机传动比
- 校准状态标志

---

## 扩展飞特舵机的修改

### 新增文件
1. `hardware/hl_client.py` - 飞特舵机驱动
2. `hardware/ft_sdk/` - 飞特 SDK

### 修改文件
1. `core.py` - 添加 `client_type` 配置支持
2. `config.yaml` - 添加 `client_type: hl` 配置项

---

## 依赖项

```toml
dynamixel-sdk   # Dynamixel 舵机 SDK
pyyaml          # YAML 配置解析
numpy           # 数值计算
fastapi         # Web API
uvicorn         # API 服务器
pytest          # 单元测试
```

---

## 快速开始

```bash
# 1. 安装
pip install -e .

# 2. 张紧
python scripts/tension.py orca_core/models/orcahand_v1_right

# 3. 校准
python scripts/calibrate.py orca_core/models/orcahand_v1_right

# 4. 测试
python scripts/neutral.py orca_core/models/orcahand_v1_right
```
