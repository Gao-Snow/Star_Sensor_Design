# src/hardware/tools/design_tools.py
"""
硬件设计辅助工具
包含电路计算、热分析、机械计算等
"""

import numpy as np
from typing import Dict, Tuple, List
import matplotlib.pyplot as plt


class CircuitCalculator:
    """电路计算工具"""

    @staticmethod
    def calculate_power_requirements(components: Dict[str, Dict]) -> Dict:
        """
        计算电源需求

        参数:
        ----------
        components : Dict[str, Dict]
            元器件功耗信息，格式:
            {
                "processor": {"voltage_v": 3.3, "current_a": 0.3, "duty_cycle": 1.0},
                "sensor": {"voltage_v": 2.5, "current_a": 0.1, "duty_cycle": 0.8},
                ...
            }

        返回:
        -------
        Dict : 电源需求汇总
        """
        total_power_w = 0.0
        voltage_currents = {}

        for name, spec in components.items():
            voltage = spec.get("voltage_v", 0)
            current = spec.get("current_a", 0)
            duty_cycle = spec.get("duty_cycle", 1.0)

            power = voltage * current * duty_cycle
            total_power_w += power

            if voltage not in voltage_currents:
                voltage_currents[voltage] = 0.0
            voltage_currents[voltage] += current * duty_cycle

        # 考虑效率损失（假设85%效率）
        input_power_w = total_power_w / 0.85

        return {
            "total_power_w": total_power_w,
            "input_power_w": input_power_w,
            "voltage_currents": voltage_currents,
            "efficiency_assumed": 0.85
        }

    @staticmethod
    def calculate_decoupling_capacitors(frequency_hz: float,
                                        current_a: float,
                                        voltage_v: float,
                                        ripple_mv: float = 50.0) -> Dict:
        """
        计算去耦电容需求

        参数:
        ----------
        frequency_hz : float
            开关频率或信号频率
        current_a : float
            瞬态电流需求
        voltage_v : float
            工作电压
        ripple_mv : float
            允许的纹波电压

        返回:
        -------
        Dict : 电容需求
        """
        ripple_v = ripple_mv / 1000.0

        # 基本计算公式: C = I * dt / dV
        dt = 1.0 / frequency_hz  # 周期
        capacitance_f = current_a * dt / ripple_v

        # 转换为常用单位
        capacitance_uf = capacitance_f * 1e6

        # 实际应用建议
        recommendations = {
            "bulk_capacitance_uf": max(10.0, capacitance_uf * 10),  # 大电容
            "decoupling_capacitance_uf": max(0.1, capacitance_uf),  # 去耦电容
            "high_freq_capacitance_nf": 100.0,  # 高频电容
            "recommended_capacitors": [
                {"value": "100uF", "type": "电解电容", "purpose": "大容量滤波"},
                {"value": "10uF", "type": "陶瓷电容", "purpose": "中频去耦"},
                {"value": "100nF", "type": "陶瓷电容", "purpose": "高频去耦"},
                {"value": "10nF", "type": "陶瓷电容", "purpose": "极高频去耦"}
            ]
        }

        return recommendations

    @staticmethod
    def calculate_pcb_trace_width(current_a: float,
                                  temperature_rise_c: float = 10.0,
                                  copper_thickness_oz: float = 1.0) -> float:
        """
        计算PCB走线宽度（基于IPC-2221标准）

        参数:
        ----------
        current_a : float
            电流（A）
        temperature_rise_c : float
            允许温升（°C）
        copper_thickness_oz : float
            铜厚（盎司），1oz = 35μm

        返回:
        -------
        float : 最小线宽（mil）
        """
        # 简化计算公式
        # 对于1oz铜厚，每10°C温升，每安培需要约10mil线宽
        base_width_per_amp = 10.0  # mil/安培

        min_width_mil = current_a * base_width_per_amp * (10.0 / temperature_rise_c)

        # 转换为mm
        min_width_mm = min_width_mil * 0.0254

        return {
            "min_width_mil": min_width_mil,
            "min_width_mm": min_width_mm,
            "recommended_width_mm": max(0.2, min_width_mm * 1.5),  # 留有余量
            "notes": "基于IPC-2221标准简化计算，实际设计应考虑更多因素"
        }


class ThermalCalculator:
    """热分析工具"""

    @staticmethod
    def calculate_heatsink_requirements(power_w: float,
                                        max_junction_temp_c: float,
                                        ambient_temp_c: float,
                                        junction_to_case_rth: float,
                                        case_to_heatsink_rth: float = 0.5) -> Dict:
        """
        计算散热器需求

        参数:
        ----------
        power_w : float
            功耗（W）
        max_junction_temp_c : float
            最大结温（°C）
        ambient_temp_c : float
            环境温度（°C）
        junction_to_case_rth : float
            结到壳热阻（°C/W）
        case_to_heatsink_rth : float
            壳到散热器热阻（°C/W）

        返回:
        -------
        Dict : 散热需求
        """
        # 计算最大允许热阻
        max_total_rth = (max_junction_temp_c - ambient_temp_c) / power_w

        # 计算散热器最大热阻
        max_heatsink_rth = max_total_rth - junction_to_case_rth - case_to_heatsink_rth

        # 估算散热器尺寸
        if max_heatsink_rth > 0:
            # 简化模型：热阻与表面积成反比
            # 典型值：10cm²/W @ 自然对流
            base_area_per_w = 10.0  # cm²/W
            required_area_cm2 = base_area_per_w / max_heatsink_rth

            # 转换为实际尺寸
            side_length_cm = np.sqrt(required_area_cm2)

            recommendations = {
                "max_heatsink_rth_c_per_w": max_heatsink_rth,
                "estimated_area_cm2": required_area_cm2,
                "estimated_size_cm": side_length_cm,
                "recommendation": f"需要至少{required_area_cm2:.1f}cm²的散热面积"
            }
        else:
            recommendations = {
                "error": "热阻需求为负，需要强制冷却",
                "recommendation": "考虑使用风扇或更大的散热器"
            }

        return recommendations

    @staticmethod
    def analyze_thermal_profile(components: List[Dict],
                                enclosure_volume_cm3: float,
                                ambient_temp_c: float) -> Dict:
        """
        分析系统热分布

        参数:
        ----------
        components : List[Dict]
            元器件列表，每个包含位置和功耗
        enclosure_volume_cm3 : float
            外壳容积（cm³）
        ambient_temp_c : float
            环境温度（°C）

        返回:
        -------
        Dict : 热分析结果
        """
        total_power_w = sum(comp.get("power_w", 0) for comp in components)

        # 简化热模型：假设均匀散热
        # 典型值：自然对流时，温升约10°C/W每100cm³
        volume_factor = enclosure_volume_cm3 / 100.0
        temp_rise_per_w = 10.0 / volume_factor

        estimated_temp_rise = total_power_w * temp_rise_per_w
        estimated_internal_temp = ambient_temp_c + estimated_temp_rise

        # 热点分析（假设最热元器件在中心）
        hotspot_temp = estimated_internal_temp + 5.0  # 热点比平均温度高5°C

        return {
            "total_power_w": total_power_w,
            "estimated_temp_rise_c": estimated_temp_rise,
            "estimated_internal_temp_c": estimated_internal_temp,
            "hotspot_temp_c": hotspot_temp,
            "volume_cm3": enclosure_volume_cm3,
            "recommendations": [
                "确保良好通风",
                "高热部件靠近外壳",
                "考虑使用导热垫将热量传导到外壳"
            ]
        }


class MechanicalCalculator:
    """机械计算工具"""

    @staticmethod
    def calculate_vibration_response(natural_frequency_hz: float,
                                     damping_ratio: float = 0.05,
                                     input_vibration_g: float = 5.0) -> Dict:
        """
        计算振动响应

        参数:
        ----------
        natural_frequency_hz : float
            固有频率（Hz）
        damping_ratio : float
            阻尼比
        input_vibration_g : float
            输入振动加速度（g）

        返回:
        -------
        Dict : 振动响应
        """
        # 转换g为m/s²
        input_accel_ms2 = input_vibration_g * 9.81

        # 计算放大因子（谐振时）
        if damping_ratio > 0:
            amplification_factor = 1.0 / (2.0 * damping_ratio)
        else:
            amplification_factor = 10.0  # 默认值

        # 谐振响应
        resonant_response_g = input_vibration_g * amplification_factor

        # 计算位移（谐振时）
        # x = a / ω²，其中ω = 2πf
        omega = 2.0 * np.pi * natural_frequency_hz
        if omega > 0:
            displacement_m = input_accel_ms2 * amplification_factor / (omega ** 2)
            displacement_mm = displacement_m * 1000.0
        else:
            displacement_mm = 0.0

        return {
            "natural_frequency_hz": natural_frequency_hz,
            "amplification_factor": amplification_factor,
            "resonant_response_g": resonant_response_g,
            "displacement_mm": displacement_mm,
            "recommendations": [
                f"设计固有频率 > {2 * natural_frequency_hz:.0f}Hz以避免谐振",
                "考虑使用减震器或阻尼材料"
            ]
        }

    @staticmethod
    def calculate_stress_concentration(force_n: float,
                                       area_mm2: float,
                                       stress_concentration_factor: float = 2.5) -> Dict:
        """
        计算应力集中

        参数:
        ----------
        force_n : float
            作用力（N）
        area_mm2 : float
            截面积（mm²）
        stress_concentration_factor : float
            应力集中系数

        返回:
        -------
        Dict : 应力分析
        """
        # 计算标称应力
        area_m2 = area_mm2 * 1e-6
        nominal_stress_pa = force_n / area_m2 if area_m2 > 0 else 0

        # 计算最大应力（考虑应力集中）
        max_stress_pa = nominal_stress_pa * stress_concentration_factor

        # 转换为MPa
        nominal_stress_mpa = nominal_stress_pa * 1e-6
        max_stress_mpa = max_stress_pa * 1e-6

        # 常见材料屈服强度（MPa）
        material_strengths = {
            "铝合金6061": 276,
            "不锈钢304": 215,
            "碳钢": 250,
            "钛合金": 830
        }

        safety_factors = {}
        for material, yield_strength in material_strengths.items():
            if max_stress_mpa > 0:
                safety_factor = yield_strength / max_stress_mpa
                safety_factors[material] = safety_factor

        return {
            "nominal_stress_mpa": nominal_stress_mpa,
            "max_stress_mpa": max_stress_mpa,
            "safety_factors": safety_factors,
            "recommendations": [
                "安全系数应大于2",
                "避免尖锐转角以减少应力集中",
                "考虑使用加强筋增加刚度"
            ]
        }


def run_design_analysis():
    """运行设计分析"""
    print("=" * 60)
    print("硬件设计分析工具")
    print("=" * 60)

    # 电路分析
    print("\n1. 电路分析")
    circuit = CircuitCalculator()

    # 电源需求计算
    components = {
        "processor": {"voltage_v": 3.3, "current_a": 0.3, "duty_cycle": 1.0},
        "sensor": {"voltage_v": 2.5, "current_a": 0.12, "duty_cycle": 0.8},
        "interface": {"voltage_v": 3.3, "current_a": 0.05, "duty_cycle": 1.0},
        "memory": {"voltage_v": 1.8, "current_a": 0.08, "duty_cycle": 0.5}
    }

    power_req = circuit.calculate_power_requirements(components)
    print(f"   总功耗: {power_req['total_power_w']:.2f}W")
    print(f"   输入功率: {power_req['input_power_w']:.2f}W")

    # 去耦电容计算
    decoupling = circuit.calculate_decoupling_capacitors(
        frequency_hz=10e6,
        current_a=0.5,
        voltage_v=3.3,
        ripple_mv=50
    )
    print(f"   建议电容: {decoupling['recommended_capacitors']}")

    # PCB走线宽度
    trace_width = circuit.calculate_pcb_trace_width(
        current_a=1.0,
        temperature_rise_c=10,
        copper_thickness_oz=1.0
    )
    print(f"   最小线宽: {trace_width['min_width_mm']:.2f}mm")

    # 热分析
    print("\n2. 热分析")
    thermal = ThermalCalculator()

    heatsink_req = thermal.calculate_heatsink_requirements(
        power_w=2.0,
        max_junction_temp_c=85,
        ambient_temp_c=25,
        junction_to_case_rth=5.0
    )
    print(f"   散热器需求: {heatsink_req.get('recommendation', '检查计算')}")

    # 系统热分析
    system_components = [
        {"name": "processor", "power_w": 0.8, "location": "center"},
        {"name": "sensor", "power_w": 0.3, "location": "front"},
        {"name": "power_regulator", "power_w": 0.5, "location": "back"}
    ]

    thermal_profile = thermal.analyze_thermal_profile(
        components=system_components,
        enclosure_volume_cm3=800,  # 100×100×80mm = 800cm³
        ambient_temp_c=25
    )
    print(f"   预计内部温度: {thermal_profile['estimated_internal_temp_c']:.1f}°C")

    # 机械分析
    print("\n3. 机械分析")
    mechanical = MechanicalCalculator()

    # 振动分析
    vibration = mechanical.calculate_vibration_response(
        natural_frequency_hz=100,
        damping_ratio=0.05,
        input_vibration_g=5.0
    )
    print(f"   谐振响应: {vibration['resonant_response_g']:.1f}g")
    print(f"   建议: {vibration['recommendations'][0]}")

    # 应力分析
    stress = mechanical.calculate_stress_concentration(
        force_n=50,  # 50N力
        area_mm2=10.0,  # 10mm²截面积
        stress_concentration_factor=2.5
    )
    print(f"   最大应力: {stress['max_stress_mpa']:.1f}MPa")

    # 显示安全系数
    for material, factor in stress['safety_factors'].items():
        status = "✓" if factor > 2 else "✗"
        print(f"   {material}: 安全系数={factor:.1f} {status}")

    print("\n设计分析完成!")
    print("=" * 60)

    return {
        "power": power_req,
        "thermal": thermal_profile,
        "vibration": vibration,
        "stress": stress
    }


if __name__ == "__main__":
    # 运行设计分析
    results = run_design_analysis()

    # 生成设计检查清单
    checklist = """
    ## 硬件设计检查清单

    ### 电路设计
    - [ ] 电源系统满足所有电压/电流需求
    - [ ] 去耦电容靠近每个电源引脚
    - [ ] 信号完整性考虑（阻抗匹配、串扰）
    - [ ] ESD保护电路
    - [ ] 过流/过压保护

    ### PCB布局
    - [ ] 电源和地平面完整
    - [ ] 高速信号走线最短
    - [ ] 模拟和数字部分隔离
    - [ ] 热源远离敏感元件
    - [ ] 安装孔和机械固定

    ### 热设计
    - [ ] 高热元件有散热路径
    - [ ] 外壳有通风或散热设计
    - [ ] 温度敏感元件远离热源

    ### 机械设计
    - [ ] 结构强度满足振动要求
    - [ ] 安装接口可靠
    - [ ] 光学对准机制
    - [ ] 防尘防潮设计

    ### 可制造性
    - [ ] 元器件封装可用
    - [ ] 焊接工艺可行
    - [ ] 测试点充分
    - [ ] 维修考虑
    """

    output_dir = r"D:\1StarTracker\hardware\design"
    import os

    os.makedirs(output_dir, exist_ok=True)

    checklist_path = os.path.join(output_dir, "design_checklist.md")
    with open(checklist_path, 'w', encoding='utf-8') as f:
        f.write(checklist)

    print(f"\n设计检查清单已保存到: {checklist_path}")