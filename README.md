# 星敏感器设计项目 (Star Sensor Design)

本项目基于 STM32H743 芯片和 Davenport_q 算法，实现了一套完整的星敏感器（Star Sensor）嵌入式系统。包含 STM32 固件源代码、星表生成与仿真测试工具，以及相关设计文档。

## 项目状态

- **STM32 固件**：已验证，姿态解算准确度较高（实测成功率约 48.7%，姿态误差约 5.4°）
- **工具代码**：提供 Python 脚本用于星表生成、通信测试和仿真验证
- **文档**：包含需求、设计、测试等全流程文档

## 目录结构
1StarTracker/
├── .gitignore # Git 忽略规则
├── README.md # 项目说明
├── LICENSE # 开源许可证
├── requirements.txt # Python 依赖列表
│
├── STM32/ # 【主代码】STM32CubeIDE 工程
│ ├── Core/ # 核心代码（.c/.h）
│ ├── Drivers/ # 硬件驱动（CMSIS、HAL）
│ ├── Middlewares/ # USB 等中间件
│ └── USB_DEVICE/ # USB 设备代码
│
├── CatalogPython/ # Python 辅助工具
│ ├── CatalogGeneration.py # 从第谷星表生成导航星表
│ ├── TestDataFromSTM.py # 生成测试数据，验证链路与算法
│ ├── RealSkyEvaluator.py # 模拟真实星图，评估姿态解算性能
│ └── tycho2_bright_stars.csv# 生成的亮星星表（星等 < 3.5）
│
├── TychoRowData/ # 【原始数据，不上传】第谷星表原始文件（约503MB，且涉及版权，访问链接见后）
│ └── ...（本地保留，不推送到 GitHub）
│
├── documents/ # 项目文档（Word、Excel、光学设计等）
├── prototype/ # Python 原型代码（早期验证，已归档）
├── firmware/ # 早期 C 代码（已过时，保留作为参考）
├── hardware/ # 硬件设计文件（已废弃，可删除）
└── 论文/ # 参考论文（涉及版权，已删除）

> **注意**：`TychoRowData/` 文件夹体积较大且涉及版权，**已通过 .gitignore 排除**，不会上传至 GitHub。如需使用，请自行从 [国家地球系统科学数据中心](https://www.geodata.cn/data/datadetails.html?dataguid=247691264164504&docId=23698) 获取。

## 快速开始

### 1. 准备星表数据

- 将第谷星表原始文件（`tyc2.dat.00` ~ `tyc2.dat.19`）放置于 `TychoRowData/` 目录下。（或自定义目录，但需要修改python代码的路径）
- 运行 `CatalogPython/CatalogGeneration.py`，生成：
  - `tycho2_bright_stars.csv`（亮星星表）
  - `star_catalog_nav.c/h`（导航星表 C 文件和头文件）
  - `triangle_db.c`（三角形数据库）
- 将生成的 `.c/.h` 文件手动复制到 `STM32/Core/Src` 和 `STM32/Core/Inc` 中。

### 2. 烧录 STM32 固件

- 使用 STM32CubeIDE 打开 `STM32/` 目录下的工程。
- 编译并烧录到开发板（我使用的是STM32H743 系列）。

### 3. 测试通信与算法

- 运行 `CatalogPython/TestDataFromSTM.py`，该脚本会读取 `triangle_db.c` 中的数据，生成测试帧并通过串口发送给开发板，验证解算算法正确性（理论成功率应为 100%）。

### 4. 真实星表仿真评估

- 运行 `CatalogPython/RealSkyEvaluator.py`，它会：
  - 读取 `tycho2_bright_stars.csv` 中的恒星数据
  - 模拟不同姿态下的 CMOS 星图
  - 通过串口与开发板通信，接收姿态解算结果
  - 输出成功率与姿态误差统计

  示例输出：
正在读取星表: D:\1StarTracker\CatalogPython\tycho2_bright_stars.csv
成功读取 116 颗恒星！
串口 COM5 打开成功

开始【真实星表评估】，帧率 2 Hz
======================================================================
[帧   1] 视野内星数:  4 | ❌ 解算失败: ERR: Det=9, Mat=0
[帧   2] 视野内星数:  6 | Det:8 Mat:5
      真值 Q: w=0.1697, x=0.0487, y=0.9168, z=0.3582
      估计 Q: w=0.1649,  x=0.0346,  y=0.9100,  z=0.3788
      姿态误差: 3.0173 度 ⚠️ 偏差较大
----------------------------------------------------------------------
[帧  3] 视野内星数:  7 | Det:7 Mat:7
      真值 Q: w=0.5175, x=-0.7873, y=-0.3079, z=0.1325
      估计 Q: w=0.5239,  x=-0.7808,  y=-0.3117,  z=0.1370
      姿态误差: 1.2499 度 ⚠️ 偏差较大
----------------------------------------------------------------------
================= 评估总结 =================
总发送测试帧: 39
成功解算帧数: 19 (成功率: 48.7%)
平均姿态误差: 5.3894 度
============================================

进程已结束，退出代码为 0


## 环境依赖

- **嵌入式开发**：STM32CubeIDE（或任意支持 STM32H7 的工具链）
- **Python 环境**：Python 3.8+
- 所需库见 `requirements.txt`（可通过 `pip install -r requirements.txt` 安装）

## 主要功能

- **星表生成**：从第谷星表提取亮星（星等 < 3.5），生成 C 语言格式的导航星表和三角形数据库。
- **质心提取**：平方加权质心算法，支持亚像素定位。
- **星识别**：三角形匹配算法，识别视野内恒星。
- **姿态解算**：基于 Davenport_q 算法（QUEST 改进版）求解四元数。
- **通信协议**：自定义图像帧协议（帧头 + 尺寸 + 图像数据 + CRC），支持 PC 模拟星图发送。
- **仿真评估**：Python 工具可模拟真实星图并统计解算性能。

## 版本历史（详细修改日志见后）

- **2026.03.30**：修正内存溢出、姿态未识别返回 0xFFFF、四元数手性问题。
- **2026.03.13**：增加 PC 图像传输协议，修改主循环逻辑。
- **2026.03.01**：新增 config、parameter 模块，剥离通信配置。
- **2026.02.28**：集成质心模块，优化 pipeline 调用逻辑。
- **2026.02.26**：统一数据类型，细化错误处理，分离示例代码。

## 注意事项

- 本仓库**不包含** STM32 固件库（如 STM32Cube_FW_H7），请自行从 ST 官网下载并放置于正确路径（参见工程配置）。
- 原始星表数据（`TychoRowData/`）**不推送至 GitHub**，请根据 `.gitignore` 中的规则自行准备。
- 仿真生成的大文件（如 `.npy`）默认被忽略，如需版本管理请使用 Git LFS。

## 联系方式

如有问题或建议，请联系：gaoxuerui2024@163.com

## 开源协议

本项目采用 [MIT License](LICENSE) 开源。


## 修改记录
日期：2026.3.1
完成通信模块，在实现过程中发现参数和配置过多，因此额外添加config模块和parameter模块剥离通信模块相关的代码（如波特率，帧格式，掉电保存等等），因此本次上传communication，config，parameter三个模块
将commu，config，para集成到pipeline中，并更新目录结构

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

日期26.3.13
修改内容：
1.定义 PC → STM32 的图像传输协议。由于本设计最终测试时，采用电脑生成星图，模拟CMOS传感器数据，使用type-c发送给单片机，单片机计算姿态结果后返回，现有 comm 模块是为 RS422 指令和遥测设计的，不适配该情景
将单独定义一套简单的图像帧协议，
帧头 (2B) | 图像宽度 (2B) | 图像高度 (2B) | 图像数据 (N bytes) | CRC16 (2B)
其中：
帧头用一个固定值（如 0xAA55）来识别图像帧。
宽度、高度可以用 uint16_t，这样图像尺寸可配置。
数据直接使用的 uint16_t 像素值（16位灰度，每个像素 2 字节），因为代码中使用 uint16_t 存储图像。
CRC 可选，用于校验。
这个协议实现在 main.c 的串口接收处理中（原来的 comm_process 只处理指令帧，你需要再增加一个图像帧解析分支）。

2. 修改主循环逻辑
原代码的 star_tracker_loop 是一个无限循环，假设图像连续到来。现在改为：等待接收完整图像 → 调用一次 star_tracker_process_frame → 发送结果 → 继续等待下一帧。

日期：26.3.30
修改内容：
1.修正了内存溢出和栈溢出的问题：主要现象是：使用python代码进行测试时，帧数据发送3份以上后，程序卡死。排查定位为star identifier中一些代码使用malloc动态内存分配，修改该部分，且将更多变量纳入static范围
2.修正了姿态未识别却给出姿态解的问题：先前版本中如果未匹配任何星，则返回0，但是0同样对应了星表中第一个数据，本次修改将未匹配任何星调整为0xFFFF
3.修正了手性问题：注意到由于main函数中和pipeline函数中未仔细考虑四元数在天球坐标系和镜头中可能存在手性差异，导致测试值和真实值相差180度，增加了q[3]= -w（即增加一个负号）