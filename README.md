# 星敏感器设计项目 (Star Sensor Design)

本项目提供了一个完整的星敏感器（Star Sensor）设计资源，包括需求文档、设计文档、硬件设计代码、核心算法原型以及相关参考文献。所有文件均托管于 GitHub，便于协作与版本管理。

## 目录结构
├── documents/ # 项目文档
│ ├── 01-需求阶段/ # 需求分析文档
│ ├── 02-设计阶段/ # 设计文档（光学、机械、硬件、系统架构等）
│ ├── 03-测试阶段/ # 测试计划与用例
│ └── 04-项目管理/ # 项目计划、总结、风险管理
├── hardware/ # 硬件设计代码（Python）
│ ├── design/ # 硬件架构设计
│ └── tools/ # 设计工具脚本
├── prototype/ # 原型算法代码
│ └── python/software/ # Python 实现
│ ├── algorithms/ # 核心算法（质心提取、QUEST 姿态确定、星图识别）
│ ├── hardware_emulation/# 硬件仿真（CMOS 仿真器等）
│ ├── pipeline/ # 星敏感器处理流水线
│ ├── simulation/ # 星图仿真
│ └── data/raw/ # 仿真数据（如需生成请自行运行脚本）
├── simulator/ # 独立仿真工具
│ ├── Simu.py # 主仿真脚本
│ └── src/simulation/ # 仿真模块
└── 论文/ # 相关参考文献（PDF）


## 主要功能

- **文档管理**：覆盖从需求到测试的全流程文档（Word、Excel、光学设计文件 .zmx/.ZDA）。
- **硬件设计**：硬件架构与工具脚本（Python）。
- **核心算法**：
  - 质心提取 (`centroid.py`)
  - 星图识别 (`star_identification.py`)
  - 姿态确定 (`quest.py`)
- **仿真与验证**：
  - 星图生成 (`star_simulation.py`)
  - CMOS 仿真 (`cmos_emulator.py`)
  - 完整处理流水线 (`star_tracker_pipeline.py`)
- **参考文献**：星敏感器设计相关学术论文。

## 环境依赖

- Python 3.8+
- 所需 Python 库：
numpy
scipy
opencv-python
matplotlib

注意事项
本仓库不包含 STM32 固件库（体积过大），如需编译固件请自行从 ST 官网下载并放置于 firmware/CubeIDE/STM32Cube_FW_H7_V1.12.1/ 路径下。

仿真生成的大数据文件（如 .npy）默认被 .gitignore 忽略，如有需要请自行生成或使用 Git LFS 管理。


欢迎通过 Issue 或 Pull Request 提出改进建议。
