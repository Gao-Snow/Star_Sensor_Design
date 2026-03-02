# 星敏感器设计项目 (Star Sensor Design)

本项目提供了一个完整的星敏感器（Star Sensor）设计资源，包括需求文档、设计文档、硬件设计代码、核心算法原型以及相关参考文献。所有文件均托管于 GitHub，便于协作与版本管理。

## 目录结构

```bash
.
├── documents/                # 项目文档
│   ├── 01-需求阶段/           # 需求分析文档
│   ├── 02-设计阶段/           # 设计文档（光学、机械、硬件、系统架构等）
│   ├── 03-测试阶段/           # 测试计划与用例
│   └── 04-项目管理/           # 项目计划、总结、风险管理
├── hardware/                 # 硬件设计代码（Python）
│   ├── design/               # 硬件架构设计
│   └── tools/                # 设计工具脚本
├── prototype/                # 原型算法代码
│   └── python/software/      # Python 实现
│       ├── algorithms/       # 核心算法（质心提取、QUEST 姿态确定、星图识别）
│       ├── hardware_emulation/# 硬件仿真（CMOS 仿真器等）
│       ├── pipeline/         # 星敏感器处理流水线
│       ├── simulation/       # 星图仿真
│       └── data/raw/         # 仿真数据（如需生成请自行运行脚本）
├─firmware					  #固件，烧写进stm32单片机的c语言代码
│  └─modules
│      ├─Davenport_q		  #核心识别算法，davenport_q算法
│      │      Davenport_q.cpp #c语言算法
│      │      Davenport_q.h   #头文件    
│      └─pipeline			  #程序流水线
│         ├──star_tracker_pipeline.cpp #流水线c语言代码
│         └──Star_Tracker_pipeline.h   #流水线头文件
├── simulator/                # 独立仿真工具
│   ├── Simu.py               # 主仿真脚本
│   └── src/simulation/       # 仿真模块
└── 论文/                      # 相关参考文献（PDF）
```

本星敏感器固件功能按照如下逻辑层次展开：
```bash
.
application/           (主程序，包含 main 函数，调用 pipeline)
firmware/
├── modules/           (独立功能模块)
│   ├── centroid/      (质心提取，待实现)
│   ├── identifier/    (星图识别，待实现)
│   ├── catalog/       (星表查询，新增)
│   └── quest/         (姿态解算，已优化2026.2.26)
├── pipeline/          (流水线调度层，已完成2026.2.26)
│   ├── star_tracker.h/c  (调用各模块完成一帧处理)
│   └── examples/      (演示 pipeline 使用的示例 main)
└── platform/          (硬件平台相关)
```

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
本仓库不包含 STM32 固件库，如需编译固件请自行从 ST 官网下载并放置于 firmware/CubeIDE/STM32Cube_FW_H7_V1.12.1/ 路径下。

仿真生成的大数据文件（如 .npy）默认被 .gitignore 忽略，如有需要请自行生成或使用 Git LFS 管理。

欢迎通过 Issue 或 Pull Request 提出改进建议。

## 修改记录
日期2026.2.28
主要修改点总结

1.新增质心模块 (centroid.h, centroid.c)：
实现了平方加权质心算法，包含背景/噪声估计和阈值处理。
提供配置结构体和默认初始化。包括错误码

2.集成到 pipeline：
在 star_tracker.c 中增加 CentroidConfig 成员，并在 star_tracker_create 中初始化。
修改 star_tracker_process_frame，直接调用 centroid_compute 进行质心提取，移除了原有的 centroid_extract 占位函数。
调整了 ROI 提取和坐标转换逻辑。
3.接口设计：
centroid_compute 只处理 ROI 内的亚像素定位，不关心全局坐标，保持单一职责。
全局坐标转换由调用者完成。
可调参数：
1.阈值系数 threshold_sigma 和窗口大小 window_size 可通过配置修改，适应不同光学设计和噪声水平。

日期：2026.2.26
1 修改 pipeline 的调用逻辑
在 star_tracker_process_frame 中，完成星图识别得到 star_ids 后，需要：
调用质心转换函数（例如 pixel_to_vector，需要相机焦距等参数）将 centroids 转换为 QuestVector3 body_vectors[]。
调用星表查询函数（例如 catalog_lookup）将 star_ids 转换为 QuestVector3 ref_vectors[]。
构造 QuestConfig 和 QuestResult，调用 quest_solve。
从 QuestResult 中提取四元数，填入输出参数。

2 统一数据类型
star_tracker.h 中应包含 davenport.h，并使用其定义的类型，避免重复定义。

3 错误处理整合
star_tracker.h 中的错误码应细化，并能映射子模块返回的错误码（如 quest_solve 返回的 QuestStatus 可转换为相应的 StarTrackerStatus）。

4 示例代码分离
目前代码在pipeline和davenport模块中有各自的 main.c，本次修改分别放入对应模块的 examples/ 目录，并确保它们能正确编译运行，后续将与其它模块合并成一个main文件


