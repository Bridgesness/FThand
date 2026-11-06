# FThand

**力反馈遥操作手**：把 ORCA Hand 的控制核心移植到 RK3588，并配一个重定向器。

## 结构

| 路径 | 内容 |
|---|---|
| `FT_core/` | ORCA Hand 控制核心（`orca_core`）的移植版：硬件抽象、标定与张紧脚本、关节空间高层控制。含 RK3588 移植实录/教程、ROS2 改造方案与实操步骤 |
| `orca_retargeter/` | 人手 → 机械手的动作重定向 |

## 相关

- 上游项目：ORCA Hand（`orcahand/orca_core`，论文 arXiv:2504.04259）
- `FT_core/docs/` 是可发布的文档站（mkdocs 结构）

## 边界

- `FT_core/` 是上游代码的移植副本，改动集中在本仓库，不反向污染上游。
- 标定数据、日志、模型不进 git。
