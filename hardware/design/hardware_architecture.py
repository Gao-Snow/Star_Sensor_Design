# src/hardware/design/hardware_architecture.py
"""
星敏感器硬件架构设计
定义系统架构、元器件选型、接口规范
"""

import json
import os
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional, Tuple
import numpy as np


@dataclass
class ProcessorSpec:
    """处理器规格"""
    name: str
    manufacturer: str
    model: str
    core: str  # 如ARM Cortex-M7, RISC-V等
    frequency_mhz: float
    flash_kb: int
    ram_kb: int
    fpga_integrated: bool
    interfaces: List[str]  # 如SPI, I2C, UART, CAN, USB等
    power_consumption_mw: float
    package: str
    operating_temperature: str
    radiation_tolerant: bool
    unit_price_usd: float
    availability: str  # 如"商用现货", "需要申请"等


@dataclass
class ImageSensorSpec:
    """图像传感器规格"""
    name: str
    manufacturer: str
    model: str
    sensor_type: str  # 如CMOS, sCMOS, CCD
    resolution: Tuple[int, int]  # (宽, 高)
    pixel_size_um: float
    pixel_architecture: str  # 如全局快门, 卷帘快门
    quantum_efficiency: float  # 量子效率
    read_noise_e: float  # 读出噪声（电子）
    dark_current_e_per_s: float  # 暗电流（电子/秒/像素）
    full_well_capacity_e: int  # 满阱容量（电子）
    adc_bits: int  # ADC位数
    data_interface: str  # 如LVDS, MIPI, parallel
    frame_rate_fps: float
    power_consumption_mw: float
    operating_temperature: str
    radiation_tolerant: bool
    unit_price_usd: float


@dataclass
class OpticalSystemSpec:
    """光学系统规格"""
    lens_type: str  # 如定焦、变焦
    focal_length_mm: float
    f_number: float
    fov_deg: float
    aperture_diameter_mm: float
    mount_type: str  # 如C-mount, CS-mount, M42
    filter_type: str  # 如带通滤光片
    transmission_efficiency: float  # 透过率
    distortion: float  # 畸变（%）
    mtf: float  # 调制传递函数
    material: str  # 材料
    coating: str  # 镀膜
    unit_price_usd: float


@dataclass
class PowerSystemSpec:
    """电源系统规格"""
    input_voltage_v: Tuple[float, float]  # 输入电压范围
    output_voltages: List[Tuple[float, float]]  # 输出电压及电流 [(电压, 最大电流), ...]
    efficiency_percent: float
    regulation: str  # 线性/LDO, 开关
    protection_features: List[str]  # 如OVP, OCP, UVLO
    power_good_signal: bool
    quiescent_current_ua: float
    operating_temperature: str


@dataclass
class InterfaceSpec:
    """接口规格"""
    type: str  # 如RS422, CAN, SpaceWire, LVDS
    standard: str  # 如RS422, CAN 2.0B
    data_rate_bps: float
    isolation: bool  # 是否隔离
    connector_type: str
    termination: bool  # 是否需要终端电阻


@dataclass
class MechanicalSpec:
    """机械结构规格"""
    enclosure_material: str  # 外壳材料
    dimensions_mm: Tuple[float, float, float]  # 长宽高
    weight_g: float
    mounting_points: List[Tuple[float, float]]  # 安装孔位置
    thermal_management: str  # 热管理方式
    sealing: str  # 密封等级
    surface_treatment: str  # 表面处理


@dataclass
class StarTrackerHardwareArchitecture:
    """星敏感器硬件架构"""

    # 项目信息
    project_name: str = "StarTracker v1.0"
    version: str = "1.0"
    design_date: str = ""

    # 系统级指标（从需求文档继承）
    system_requirements: Dict = None

    # 子系统规格
    processor: Optional[ProcessorSpec] = None
    image_sensor: Optional[ImageSensorSpec] = None
    optical_system: Optional[OpticalSystemSpec] = None
    power_system: Optional[PowerSystemSpec] = None
    interfaces: List[InterfaceSpec] = None
    mechanical: Optional[MechanicalSpec] = None

    # 其他组件
    memory: Optional[Dict] = None  # Flash, RAM等
    clock_system: Optional[Dict] = None  # 时钟、晶振
    thermal_management: Optional[Dict] = None  # 散热、温控
    shielding: Optional[Dict] = None  # 电磁屏蔽

    # 成本估算
    total_cost_estimate_usd: float = 0.0
    development_cost_usd: float = 0.0

    def __post_init__(self):
        """初始化后处理"""
        if self.system_requirements is None:
            self.system_requirements = self._default_requirements()
        if self.interfaces is None:
            self.interfaces = []

    def _default_requirements(self) -> Dict:
        """默认系统需求"""
        return {
            "accuracy_cross_boresight_arcsec": 10.0,
            "accuracy_roll_arcsec": 30.0,
            "update_rate_hz": 1.0,
            "fov_deg": 10.0,
            "detection_limit_magnitude": 5.0,
            "power_consumption_w": 5.0,
            "mass_g": 1500.0,
            "dimensions_mm": [100, 100, 80],
            "operating_temperature_c": [-10, 50],
            "storage_temperature_c": [-30, 70],
            "vibration_spec": "5-500Hz, 5g RMS",
            "shock_spec": "50g, 11ms half-sine",
            "lifetime_years": 1,
            "mtbf_hours": 10000
        }

    def select_processor(self, budget_usd: float = 50.0) -> ProcessorSpec:
        """选择处理器"""
        # 候选处理器列表
        candidates = [
            ProcessorSpec(
                name="STM32H743VI",
                manufacturer="STMicroelectronics",
                model="STM32H743VIT6",
                core="ARM Cortex-M7",
                frequency_mhz=480,
                flash_kb=2048,
                ram_kb=1056,
                fpga_integrated=False,
                interfaces=["SPI", "I2C", "UART", "CAN", "USB", "Ethernet", "SDIO"],
                power_consumption_mw=250,
                package="LQFP100",
                operating_temperature="-40 to 85°C",
                radiation_tolerant=False,
                unit_price_usd=15.0,
                availability="商用现货"
            ),
            ProcessorSpec(
                name="STM32H750VB",
                manufacturer="STMicroelectronics",
                model="STM32H750VBT6",
                core="ARM Cortex-M7",
                frequency_mhz=480,
                flash_kb=128,
                ram_kb=1056,
                fpga_integrated=False,
                interfaces=["SPI", "I2C", "UART", "CAN", "USB", "Ethernet", "SDIO"],
                power_consumption_mw=200,
                package="LQFP100",
                operating_temperature="-40 to 85°C",
                radiation_tolerant=False,
                unit_price_usd=12.0,
                availability="商用现货"
            ),
            ProcessorSpec(
                name="Kendryte K210",
                manufacturer="Canaan Creative",
                model="K210",
                core="64-bit RISC-V Dual Core",
                frequency_mhz=400,
                flash_kb=8192,
                ram_kb=8192,
                fpga_integrated=True,
                interfaces=["SPI", "I2C", "UART", "I2S", "FPIOA"],
                power_consumption_mw=300,
                package="QFN88",
                operating_temperature="-40 to 85°C",
                radiation_tolerant=False,
                unit_price_usd=8.0,
                availability="商用现货"
            ),
            ProcessorSpec(
                name="Xilinx Zynq-7010",
                manufacturer="Xilinx",
                model="XC7Z010",
                core="ARM Cortex-A9 + FPGA",
                frequency_mhz=667,
                flash_kb=0,  # 外部存储
                ram_kb=512,
                fpga_integrated=True,
                interfaces=["SPI", "I2C", "UART", "CAN", "USB", "Ethernet", "GPIO"],
                power_consumption_mw=1500,
                package="CLG400",
                operating_temperature="-40 to 100°C",
                radiation_tolerant=False,
                unit_price_usd=45.0,
                availability="商用现货"
            )
        ]

        # 根据预算和需求选择
        selected = None
        for candidate in candidates:
            if candidate.unit_price_usd <= budget_usd:
                # 检查是否满足需求
                if candidate.frequency_mhz >= 200:  # 需要足够计算能力
                    selected = candidate
                    break

        if selected is None:
            # 默认选择第一个
            selected = candidates[0]

        self.processor = selected
        return selected

    def select_image_sensor(self, budget_usd: float = 100.0) -> ImageSensorSpec:
        """选择图像传感器"""
        candidates = [
            ImageSensorSpec(
                name="AR0134CS",
                manufacturer="ON Semiconductor",
                model="AR0134CSSC00SPCA0-DRBR",
                sensor_type="CMOS",
                resolution=(1280, 960),
                pixel_size_um=3.75,
                pixel_architecture="全局快门",
                quantum_efficiency=0.42,
                read_noise_e=12.0,
                dark_current_e_per_s=15.0,
                full_well_capacity_e=10000,
                adc_bits=12,
                data_interface="并行/LVDS",
                frame_rate_fps=60,
                power_consumption_mw=300,
                operating_temperature="-30 to 70°C",
                radiation_tolerant=False,
                unit_price_usd=25.0
            ),
            ImageSensorSpec(
                name="IMX273LLR",
                manufacturer="Sony",
                model="IMX273LLR",
                sensor_type="CMOS",
                resolution=(1456, 1088),
                pixel_size_um=3.45,
                pixel_architecture="全局快门",
                quantum_efficiency=0.68,
                read_noise_e=5.2,
                dark_current_e_per_s=8.5,
                full_well_capacity_e=12000,
                adc_bits=12,
                data_interface="MIPI CSI-2",
                frame_rate_fps=60,
                power_consumption_mw=280,
                operating_temperature="-30 to 70°C",
                radiation_tolerant=False,
                unit_price_usd=35.0
            ),
            ImageSensorSpec(
                name="MT9V034",
                manufacturer="ON Semiconductor",
                model="MT9V034C12STM",
                sensor_type="CMOS",
                resolution=(752, 480),
                pixel_size_um=6.0,
                pixel_architecture="全局快门",
                quantum_efficiency=0.38,
                read_noise_e=25.0,
                dark_current_e_per_s=30.0,
                full_well_capacity_e=15000,
                adc_bits=10,
                data_interface="并行",
                frame_rate_fps=60,
                power_consumption_mw=180,
                operating_temperature="-30 to 70°C",
                radiation_tolerant=False,
                unit_price_usd=18.0
            ),
            ImageSensorSpec(
                name="OV9281",
                manufacturer="OmniVision",
                model="OV9281",
                sensor_type="CMOS",
                resolution=(1280, 800),
                pixel_size_um=3.0,
                pixel_architecture="全局快门",
                quantum_efficiency=0.45,
                read_noise_e=8.0,
                dark_current_e_per_s=12.0,
                full_well_capacity_e=8000,
                adc_bits=10,
                data_interface="MIPI CSI-2",
                frame_rate_fps=120,
                power_consumption_mw=200,
                operating_temperature="-30 to 85°C",
                radiation_tolerant=False,
                unit_price_usd=15.0
            )
        ]

        # 根据仿真结果选择
        # 从仿真中获取的推荐参数
        recommended_pixel_size = 3.45  # um，来自仿真
        recommended_resolution = (1024, 1024)  # 来自需求

        selected = None
        best_score = -1

        for candidate in candidates:
            if candidate.unit_price_usd <= budget_usd:
                # 计算匹配分数
                score = 0

                # 像素尺寸匹配（越接近推荐值得分越高）
                pixel_size_diff = abs(candidate.pixel_size_um - recommended_pixel_size)
                score += max(0, 10 - pixel_size_diff * 2)

                # 分辨率匹配
                resolution_product = candidate.resolution[0] * candidate.resolution[1]
                recommended_product = recommended_resolution[0] * recommended_resolution[1]
                resolution_ratio = min(resolution_product, recommended_product) / max(resolution_product,
                                                                                      recommended_product)
                score += resolution_ratio * 20

                # 量子效率（越高越好）
                score += candidate.quantum_efficiency * 15

                # 读出噪声（越低越好）
                score += max(0, 15 - candidate.read_noise_e / 2)

                # 功耗（越低越好）
                score += max(0, 10 - candidate.power_consumption_mw / 30)

                if score > best_score:
                    best_score = score
                    selected = candidate

        if selected is None:
            selected = candidates[0]

        self.image_sensor = selected
        return selected

    def select_optical_system(self, budget_usd: float = 200.0) -> OpticalSystemSpec:
        """选择光学系统"""
        # 根据需求计算光学参数
        fov_deg = self.system_requirements["fov_deg"]
        detection_limit = self.system_requirements["detection_limit_magnitude"]

        # 假设我们选择了IMX273传感器
        pixel_size = 3.45e-3  # mm
        resolution_x = 1456  # 假设使用这个传感器

        # 计算焦距
        # f = (pixel_size * resolution_x) / (2 * tan(FOV/2))
        fov_rad = np.radians(fov_deg)
        focal_length_mm = (pixel_size * resolution_x) / (2 * np.tan(fov_rad / 2))

        # 计算光圈（基于探测星等需求）
        # 简化的计算：需要足够的通光量探测+5等星
        aperture_diameter_mm = 25.0  # 初始估计，可根据灵敏度计算优化

        # F数
        f_number = focal_length_mm / aperture_diameter_mm

        selected = OpticalSystemSpec(
            lens_type="定焦镜头",
            focal_length_mm=round(focal_length_mm, 1),
            f_number=round(f_number, 1),
            fov_deg=fov_deg,
            aperture_diameter_mm=aperture_diameter_mm,
            mount_type="C-mount",
            filter_type="400-700nm带通滤光片",
            transmission_efficiency=0.85,
            distortion=0.1,  # 0.1%
            mtf=0.6,  # 在Nyquist频率处
            material="玻璃",
            coating="多层增透膜",
            unit_price_usd=150.0
        )

        self.optical_system = selected
        return selected

    def design_power_system(self) -> PowerSystemSpec:
        """设计电源系统"""
        # 估算各模块功耗
        processor_power = self.processor.power_consumption_mw if self.processor else 250
        sensor_power = self.image_sensor.power_consumption_mw if self.image_sensor else 300
        other_power = 100  # 接口、时钟等

        total_power_mw = processor_power + sensor_power + other_power

        selected = PowerSystemSpec(
            input_voltage_v=(4.5, 5.5),  # 标准5V输入
            output_voltages=[
                (3.3, 1.0),  # 3.3V, 1A - 数字逻辑
                (2.5, 0.5),  # 2.5V, 0.5A - 传感器模拟
                (1.8, 0.5),  # 1.8V, 0.5A - 传感器数字
                (12.0, 0.1),  # 12V, 0.1A - 可选（如TEC）
            ],
            efficiency_percent=85.0,
            regulation="开关电源+线性稳压",
            protection_features=["OVP", "OCP", "UVLO", "反接保护"],
            power_good_signal=True,
            quiescent_current_ua=50.0,
            operating_temperature="-40 to 85°C"
        )

        self.power_system = selected
        return selected

    def design_interfaces(self) -> List[InterfaceSpec]:
        """设计接口"""
        interfaces = [
            InterfaceSpec(
                type="RS422",
                standard="RS-422",
                data_rate_bps=115200,
                isolation=True,
                connector_type="D-SUB 9针",
                termination=True
            ),
            InterfaceSpec(
                type="CAN",
                standard="CAN 2.0B",
                data_rate_bps=500000,
                isolation=True,
                connector_type="DB9或端子",
                termination=True
            ),
            InterfaceSpec(
                type="UART",
                standard="3.3V TTL",
                data_rate_bps=115200,
                isolation=False,
                connector_type="2.54mm排针",
                termination=False
            ),
            InterfaceSpec(
                type="同步信号",
                standard="LVTTL",
                data_rate_bps=0,  # 脉冲信号
                isolation=False,
                connector_type="SMA或排针",
                termination=False
            )
        ]

        self.interfaces = interfaces
        return interfaces

    def design_mechanical_structure(self) -> MechanicalSpec:
        """设计机械结构"""
        selected = MechanicalSpec(
            enclosure_material="铝合金6061",
            dimensions_mm=(100.0, 100.0, 80.0),
            weight_g=800,
            mounting_points=[
                (10.0, 10.0),
                (90.0, 10.0),
                (10.0, 90.0),
                (90.0, 90.0)
            ],
            thermal_management="被动散热+导热垫",
            sealing="IP54（防尘防水）",
            surface_treatment="黑色阳极氧化"
        )

        self.mechanical = selected
        return selected

    def calculate_cost(self) -> float:
        """计算总成本估算"""
        component_cost = 0.0

        if self.processor:
            component_cost += self.processor.unit_price_usd
        if self.image_sensor:
            component_cost += self.image_sensor.unit_price_usd
        if self.optical_system:
            component_cost += self.optical_system.unit_price_usd

        # 其他组件估算
        component_cost += 50.0  # 电源、接口、被动元件等
        component_cost += 30.0  # PCB、连接器
        component_cost += 40.0  # 机械结构

        # 开发成本估算（一次性）
        self.development_cost_usd = 500.0

        self.total_cost_estimate_usd = component_cost
        return component_cost

    def generate_system_block_diagram(self) -> Dict:
        """生成系统框图数据"""
        block_diagram = {
            "system_name": self.project_name,
            "subsystems": [
                {
                    "name": "光学头部",
                    "components": [
                        "镜头",
                        "滤光片",
                        "图像传感器",
                        "传感器接口电路"
                    ],
                    "interfaces": ["光学接口", "电接口"]
                },
                {
                    "name": "处理单元",
                    "components": [
                        "主处理器",
                        "存储器",
                        "电源管理",
                        "时钟电路"
                    ],
                    "interfaces": ["电源输入", "数据接口", "调试接口"]
                },
                {
                    "name": "接口单元",
                    "components": [
                        "RS422接口",
                        "CAN接口",
                        "同步信号接口",
                        "调试接口"
                    ],
                    "interfaces": ["外部连接器"]
                },
                {
                    "name": "机械结构",
                    "components": [
                        "外壳",
                        "安装接口",
                        "遮光罩",
                        "散热结构"
                    ],
                    "interfaces": ["安装孔", "光学窗口"]
                }
            ],
            "data_flow": [
                {"from": "光学头部", "to": "处理单元", "type": "图像数据", "interface": "并行/LVDS"},
                {"from": "处理单元", "to": "接口单元", "type": "姿态数据", "interface": "内部总线"},
                {"from": "接口单元", "to": "外部", "type": "RS422/CAN", "interface": "连接器"}
            ],
            "power_flow": [
                {"from": "外部电源", "to": "电源管理", "voltage": "5V"},
                {"from": "电源管理", "to": "处理单元", "voltage": "3.3V, 1.8V, 2.5V"},
                {"from": "电源管理", "to": "光学头部", "voltage": "3.3V, 2.5V, 1.8V"}
            ]
        }

        return block_diagram

    def export_specifications(self, output_dir: str):
        """导出规格文档"""
        os.makedirs(output_dir, exist_ok=True)

        # 导出JSON格式
        spec_dict = {
            "project_info": {
                "name": self.project_name,
                "version": self.version,
                "design_date": self.design_date
            },
            "system_requirements": self.system_requirements,
            "component_specs": {
                "processor": asdict(self.processor) if self.processor else None,
                "image_sensor": asdict(self.image_sensor) if self.image_sensor else None,
                "optical_system": asdict(self.optical_system) if self.optical_system else None,
                "power_system": asdict(self.power_system) if self.power_system else None,
                "interfaces": [asdict(i) for i in self.interfaces] if self.interfaces else [],
                "mechanical": asdict(self.mechanical) if self.mechanical else None
            },
            "cost_analysis": {
                "component_cost_usd": self.total_cost_estimate_usd,
                "development_cost_usd": self.development_cost_usd,
                "total_cost_usd": self.total_cost_estimate_usd + self.development_cost_usd
            },
            "block_diagram": self.generate_system_block_diagram()
        }

        # 保存JSON文件
        json_path = os.path.join(output_dir, "hardware_specifications.json")
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(spec_dict, f, indent=2, ensure_ascii=False)

        print(f"硬件规格已保存到: {json_path}")

        # 生成Markdown报告
        md_path = os.path.join(output_dir, "hardware_design_report.md")
        self._generate_markdown_report(md_path, spec_dict)

        return spec_dict

    def _generate_markdown_report(self, filepath: str, spec_dict: Dict):
        """生成Markdown格式的设计报告"""
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(f"# 星敏感器硬件设计报告\n\n")
            f.write(f"**项目名称**: {spec_dict['project_info']['name']}\n")
            f.write(f"**版本**: {spec_dict['project_info']['version']}\n")
            f.write(f"**设计日期**: {spec_dict['project_info']['design_date']}\n\n")

            f.write("## 1. 系统需求\n")
            f.write("| 参数 | 指标 | 备注 |\n")
            f.write("|------|------|------|\n")
            for key, value in spec_dict['system_requirements'].items():
                f.write(f"| {key} | {value} | - |\n")

            f.write("\n## 2. 处理器选型\n")
            if spec_dict['component_specs']['processor']:
                proc = spec_dict['component_specs']['processor']
                f.write(f"- **型号**: {proc['manufacturer']} {proc['model']}\n")
                f.write(f"- **核心**: {proc['core']} @ {proc['frequency_mhz']}MHz\n")
                f.write(f"- **存储**: {proc['flash_kb']}KB Flash, {proc['ram_kb']}KB RAM\n")
                f.write(f"- **接口**: {', '.join(proc['interfaces'])}\n")
                f.write(f"- **功耗**: {proc['power_consumption_mw']}mW\n")
                f.write(f"- **价格**: ${proc['unit_price_usd']}\n")

            f.write("\n## 3. 图像传感器选型\n")
            if spec_dict['component_specs']['image_sensor']:
                sensor = spec_dict['component_specs']['image_sensor']
                f.write(f"- **型号**: {sensor['manufacturer']} {sensor['model']}\n")
                f.write(f"- **分辨率**: {sensor['resolution'][0]}×{sensor['resolution'][1]}\n")
                f.write(f"- **像元尺寸**: {sensor['pixel_size_um']}μm\n")
                f.write(f"- **量子效率**: {sensor['quantum_efficiency'] * 100:.1f}%\n")
                f.write(f"- **读出噪声**: {sensor['read_noise_e']:.1f}e-\n")
                f.write(f"- **数据接口**: {sensor['data_interface']}\n")
                f.write(f"- **价格**: ${sensor['unit_price_usd']}\n")

            f.write("\n## 4. 光学系统设计\n")
            if spec_dict['component_specs']['optical_system']:
                optical = spec_dict['component_specs']['optical_system']
                f.write(f"- **焦距**: {optical['focal_length_mm']}mm\n")
                f.write(f"- **F数**: F/{optical['f_number']}\n")
                f.write(f"- **视场角**: {optical['fov_deg']}°\n")
                f.write(f"- **孔径**: {optical['aperture_diameter_mm']}mm\n")
                f.write(f"- **畸变**: {optical['distortion']}%\n")
                f.write(f"- **价格**: ${optical['unit_price_usd']}\n")

            f.write("\n## 5. 成本分析\n")
            cost = spec_dict['cost_analysis']
            f.write(f"- **元器件成本**: ${cost['component_cost_usd']:.2f}\n")
            f.write(f"- **开发成本**: ${cost['development_cost_usd']:.2f}\n")
            f.write(f"- **总成本**: ${cost['total_cost_usd']:.2f}\n")

            f.write("\n## 6. 系统框图\n")
            f.write("```\n")
            # 简单的文本框图
            f.write("外部电源 (5V) → 电源管理 → 各子系统\n")
            f.write("光学头部 → 图像数据 → 处理单元 → 姿态数据 → 接口单元 → 外部接口\n")
            f.write("```\n")

        print(f"设计报告已保存到: {filepath}")


def design_hardware_architecture():
    """设计硬件架构"""
    print("=" * 60)
    print("星敏感器硬件架构设计")
    print("=" * 60)

    # 创建硬件架构对象
    hardware = StarTrackerHardwareArchitecture(
        project_name="StarTracker v1.0",
        version="1.0",
        design_date="2025-01-20"
    )

    print("\n1. 处理器选型...")
    processor = hardware.select_processor(budget_usd=50.0)
    print(f"   选定: {processor.manufacturer} {processor.model}")
    print(f"   核心: {processor.core} @ {processor.frequency_mhz}MHz")
    print(f"   价格: ${processor.unit_price_usd}")

    print("\n2. 图像传感器选型...")
    sensor = hardware.select_image_sensor(budget_usd=100.0)
    print(f"   选定: {sensor.manufacturer} {sensor.model}")
    print(f"   分辨率: {sensor.resolution[0]}×{sensor.resolution[1]}")
    print(f"   像元尺寸: {sensor.pixel_size_um}μm")
    print(f"   价格: ${sensor.unit_price_usd}")

    print("\n3. 光学系统设计...")
    optical = hardware.select_optical_system(budget_usd=200.0)
    print(f"   焦距: {optical.focal_length_mm}mm")
    print(f"   F数: F/{optical.f_number}")
    print(f"   视场角: {optical.fov_deg}°")
    print(f"   孔径: {optical.aperture_diameter_mm}mm")
    print(f"   价格: ${optical.unit_price_usd}")

    print("\n4. 电源系统设计...")
    power = hardware.design_power_system()
    print(f"   输入: {power.input_voltage_v[0]}-{power.input_voltage_v[1]}V")
    print(f"   输出: {len(power.output_voltages)}路")
    print(f"   效率: {power.efficiency_percent}%")

    print("\n5. 接口设计...")
    interfaces = hardware.design_interfaces()
    print(f"   设计了 {len(interfaces)} 种接口:")
    for interface in interfaces:
        print(f"     - {interface.type} @ {interface.data_rate_bps / 1000:.1f}kbps")

    print("\n6. 机械结构设计...")
    mechanical = hardware.design_mechanical_structure()
    print(f"   尺寸: {mechanical.dimensions_mm[0]}×{mechanical.dimensions_mm[1]}×{mechanical.dimensions_mm[2]}mm")
    print(f"   重量: {mechanical.weight_g}g")
    print(f"   材料: {mechanical.enclosure_material}")

    print("\n7. 成本估算...")
    total_cost = hardware.calculate_cost()
    print(f"   元器件成本: ${total_cost:.2f}")
    print(f"   开发成本: ${hardware.development_cost_usd:.2f}")
    print(f"   总成本: ${total_cost + hardware.development_cost_usd:.2f}")

    print("\n8. 导出规格文档...")
    output_dir = r"D:\1StarTracker\hardware\design"
    spec_dict = hardware.export_specifications(output_dir)

    print("\n硬件架构设计完成!")
    print("=" * 60)

    return hardware, spec_dict


def create_component_selection_guide():
    """创建元器件选型指南"""
    print("\n" + "=" * 60)
    print("元器件选型指南")
    print("=" * 60)

    guide = """
    ## 星敏感器关键元器件选型指南

    ### 1. 处理器选型要点
    - **计算能力**: 需要≥200MHz主频，用于实时图像处理和姿态解算
    - **存储空间**: ≥512KB RAM用于图像缓存，≥1MB Flash用于程序和星库
    - **接口需求**: 至少需要SPI/I2C控制传感器，UART/CAN输出数据
    - **功耗考虑**: 工作功耗应<500mW
    - **推荐型号**: 
      * 入门级: STM32H750 (480MHz Cortex-M7, 价格适中)
      * 高性能: Xilinx Zynq-7010 (FPGA+ARM，适合复杂算法)
      * 低成本: Kendryte K210 (RISC-V双核，内置AI加速)

    ### 2. 图像传感器选型要点
    - **快门类型**: 必须使用全局快门，避免拖影
    - **像元尺寸**: 3-5μm为宜，过小则灵敏度低，过大则分辨率低
    - **量子效率**: >40% @ 550nm，越高越好
    - **读出噪声**: <15e- RMS，越低越好
    - **满阱容量**: >8000e-，避免亮星饱和
    - **数据接口**: 优先选择并行或LVDS接口，MIPI需要转接
    - **推荐型号**:
      * 性价比: ON Semi AR0134 (1280×960, 全局快门)
      * 高性能: Sony IMX273 (1456×1088, 高量子效率)
      * 低成本: OmniVision OV9281 (1280×800, 全局快门)

    ### 3. 光学系统选型要点
    - **焦距计算**: f = (像元尺寸 × 像元数) / (2 × tan(FOV/2))
    - **F数选择**: F/1.4-F/2.8，平衡通光量和像差
    - **畸变控制**: <0.2%全视场
    - **透过率**: >80% @ 400-700nm
    - **接口标准**: C-mount或CS-mount，便于安装
    - **滤光片**: 400-700nm带通，抑制背景光

    ### 4. 采购建议
    1. **处理器和传感器**: 从Digi-Key、Mouser等正规代理商采购
    2. **光学镜头**: 考虑定制或从Edmund Optics、Thorlabs采购
    3. **PCB加工**: 使用JLCPCB或PCBWay快速打样
    4. **结构件**: 本地机加工或3D打印
    5. **测试设备**: 考虑租用或使用学校实验室设备
    """

    # 保存指南
    output_dir = r"D:\1StarTracker\hardware\design"
    os.makedirs(output_dir, exist_ok=True)

    guide_path = os.path.join(output_dir, "component_selection_guide.md")
    with open(guide_path, 'w', encoding='utf-8') as f:
        f.write(guide)

    print(f"元器件选型指南已保存到: {guide_path}")
    print("=" * 60)


if __name__ == "__main__":
    # 设计硬件架构
    hardware, spec_dict = design_hardware_architecture()

    # 创建选型指南
    create_component_selection_guide()

    print("\n硬件设计第一阶段完成！")
    print("下一步：开始原理图设计")