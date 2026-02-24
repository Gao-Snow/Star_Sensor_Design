# src/software/simulation/star_simulator.py
"""
星图模拟器
根据论文第III节：Star light sensitivity, detection threshold, average number of stars in the FOV and sky coverage
生成包含多颗星、不同星等、添加噪声和PSF效应的模拟星图
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
import json
import os
import pandas as pd
import cv2

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


@dataclass
class Star:
    """恒星数据类"""
    ra_deg: float
    dec_deg: float
    magnitude: float
    spectral_type: str = "G2V"


@dataclass
class StarTrackerConfig:
    """星敏感器配置类"""
    # 光学参数
    focal_length_mm: float = 50.0
    aperture_diameter_mm: float = 25.0
    f_number: float = 2.0
    fov_deg: float = 10.0

    # 探测器参数
    pixel_size_um: float = 3.45
    resolution_x: int = 1024
    resolution_y: int = 1024
    quantum_efficiency: float = 0.5
    read_noise_e: float = 5.0
    dark_current_e_per_s: float = 10.0
    full_well_capacity_e: int = 10000
    adc_bits: int = 12

    # 曝光参数
    exposure_time_ms: float = 200.0

    # PSF参数
    psf_sigma_pixels: float = 1.0
    defocus_level: float = 0.8

    # 噪声参数
    enable_photon_noise: bool = True
    enable_read_noise: bool = True
    enable_dark_current: bool = True
    enable_adc_quantization: bool = True

    # 动态参数
    angular_velocity_deg_per_s: float = 0.0  # 角速度（°/s）

    def __post_init__(self):
        """计算派生参数"""
        self.focal_length_m = self.focal_length_mm / 1000.0
        self.aperture_area_m2 = np.pi * (self.aperture_diameter_mm / 2000.0) ** 2
        self.pixel_size_m = self.pixel_size_um * 1e-6
        self.pixel_scale_rad = self.pixel_size_m / self.focal_length_m
        self.pixel_scale_arcsec = self.pixel_scale_rad * 206265
        self.fov_pixels_x = int(self.fov_deg * 3600 / self.pixel_scale_arcsec)
        self.fov_pixels_y = self.fov_pixels_x
        self.exposure_time_s = self.exposure_time_ms / 1000.0
        self.focal_length_pixels = self.focal_length_mm / self.pixel_size_um


class StarSimulator:
    """星图模拟器主类"""

    def __init__(self, config: Optional[StarTrackerConfig] = None):
        self.config = config or StarTrackerConfig()
        self.flux_m0_photoelectrons = 19100
        self.gain_dn_per_e = (2 ** self.config.adc_bits) / self.config.full_well_capacity_e
        self.center_x = self.config.resolution_x / 2.0
        self.center_y = self.config.resolution_y / 2.0
        self.star_catalog = self._generate_star_catalog()

    def _generate_star_catalog(self, num_stars: int = 100) -> List[Star]:
        """生成模拟星表"""
        stars = []
        magnitudes = np.random.exponential(scale=2.0, size=num_stars) + 2.0
        magnitudes = np.clip(magnitudes, 2.0, 6.0)
        ras = np.random.uniform(-self.config.fov_deg / 2, self.config.fov_deg / 2, num_stars)
        decs = np.random.uniform(-self.config.fov_deg / 2, self.config.fov_deg / 2, num_stars)

        for i in range(num_stars):
            stars.append(Star(ra_deg=ras[i], dec_deg=decs[i], magnitude=magnitudes[i]))
        return stars

    def star_position_to_pixel(self, ra_deg: float, dec_deg: float) -> Tuple[float, float]:
        """将天球坐标转换为像素坐标"""
        ra_rad, dec_rad = np.deg2rad(ra_deg), np.deg2rad(dec_deg)
        x_pixel = self.center_x + self.config.focal_length_pixels * np.tan(ra_rad)
        y_pixel = self.center_y + self.config.focal_length_pixels * np.tan(dec_rad)
        return x_pixel, y_pixel

    def calculate_star_flux(self, magnitude: float) -> float:
        """计算恒星光电子数"""
        flux_m0 = self.flux_m0_photoelectrons
        aperture_area_mm2 = self.config.aperture_area_m2 * 1e6
        magnitude_factor = 2.512 ** (-magnitude)
        return (flux_m0 * aperture_area_mm2 * self.config.exposure_time_s *
                magnitude_factor * self.config.quantum_efficiency)

    def create_ideal_star_image(self, stars: Optional[List[Star]] = None) -> np.ndarray:
        """创建理想星图"""
        stars = stars or self.star_catalog
        image = np.zeros((self.config.resolution_y, self.config.resolution_x), dtype=np.float32)

        for star in stars:
            x, y = self.star_position_to_pixel(star.ra_deg, star.dec_deg)
            if 0 <= x < self.config.resolution_x and 0 <= y < self.config.resolution_y:
                flux = self.calculate_star_flux(star.magnitude)
                ix, iy = int(round(x)), int(round(y))
                if 0 <= ix < self.config.resolution_x and 0 <= iy < self.config.resolution_y:
                    image[iy, ix] += flux
        return image

    def apply_motion_blur(self, image: np.ndarray) -> np.ndarray:
        """应用运动模糊（根据新论文）"""
        if self.config.angular_velocity_deg_per_s == 0:
            return image

        # 计算模糊长度（像素）
        blur_length_pixels = (self.config.angular_velocity_deg_per_s *
                              self.config.exposure_time_s *
                              self.config.focal_length_pixels / 206265)

        kernel_size = max(3, int(abs(blur_length_pixels)))
        angle = np.random.uniform(0, 360)  # 随机运动方向

        # 创建运动模糊核
        kernel = np.zeros((kernel_size, kernel_size))
        center = kernel_size // 2
        if blur_length_pixels >= 0:
            # 线性运动模糊
            kernel[center, :] = 1.0 / kernel_size
        else:
            # 考虑方向
            pass

        # 旋转核到指定角度
        M = cv2.getRotationMatrix2D((center, center), angle, 1)
        kernel = cv2.warpAffine(kernel, M, (kernel_size, kernel_size))

        # 应用模糊
        blurred = cv2.filter2D(image, -1, kernel)
        return blurred

    def apply_psf(self, image: np.ndarray) -> np.ndarray:
        """应用PSF"""
        sigma = self.config.psf_sigma_pixels * self.config.defocus_level
        return gaussian_filter(image, sigma=sigma, mode='constant')

    def add_photon_noise(self, image: np.ndarray) -> np.ndarray:
        """添加光子噪声"""
        if not self.config.enable_photon_noise:
            return image
        return np.random.poisson(image).astype(np.float32)

    def add_dark_current(self, image: np.ndarray) -> np.ndarray:
        """添加暗电流"""
        if not self.config.enable_dark_current:
            return image
        dark_current = self.config.dark_current_e_per_s * self.config.exposure_time_s
        return image + np.random.poisson(dark_current, image.shape)

    def add_read_noise(self, image: np.ndarray) -> np.ndarray:
        """添加读出噪声"""
        if not self.config.enable_read_noise:
            return image
        return image + np.random.normal(0, self.config.read_noise_e, image.shape)

    def apply_adc_quantization(self, image: np.ndarray) -> np.ndarray:
        """应用ADC量化"""
        if not self.config.enable_adc_quantization:
            return image
        dn_image = image * self.gain_dn_per_e
        max_dn = 2 ** self.config.adc_bits - 1
        dn_image = np.clip(dn_image, 0, max_dn)
        return np.round(dn_image).astype(np.uint16)

    def generate_star_image(self, apply_motion_blur: bool = True) -> np.ndarray:
        """生成完整的模拟星图"""
        # 1. 理想星图
        ideal = self.create_ideal_star_image()

        # 2. PSF
        psf = self.apply_psf(ideal)

        # 3. 运动模糊（可选）
        if apply_motion_blur and self.config.angular_velocity_deg_per_s > 0:
            psf = self.apply_motion_blur(psf)

        # 4. 噪声
        with_dark = self.add_dark_current(psf)
        with_photon = self.add_photon_noise(with_dark)
        with_read = self.add_read_noise(with_photon)

        # 5. ADC量化
        final = self.apply_adc_quantization(with_read)

        return final

    def detect_stars_count(self, image: np.ndarray, threshold_factor: float = 5.0) -> int:
        """检测星点数量"""
        # 估计背景噪声
        background = image[image < np.percentile(image, 80)]
        noise_std = np.std(background)
        threshold = np.mean(background) + threshold_factor * noise_std

        # 连通域分析
        from scipy import ndimage
        labeled_array, num_features = ndimage.label(image > threshold)
        return num_features

    def calculate_snr(self, image: np.ndarray) -> float:
        """计算信噪比"""
        # 简单估算：最高信号与背景噪声之比
        background = image[image < np.percentile(image, 90)]
        signal_peak = np.max(image)
        noise_rms = np.std(background)
        return signal_peak / noise_rms if noise_rms > 0 else 0

    def estimate_centroid_error(self, image: np.ndarray) -> float:
        """估计质心误差（像素）"""
        # 简单估算：根据SNR的倒数
        snr = self.calculate_snr(image)
        if snr > 0:
            # 经验公式：误差 ∝ 1/SNR
            return 0.5 / snr  # 单位：像素
        return 1.0  # 默认误差

    def analyze_image(self, image: np.ndarray) -> Dict[str, Any]:
        """分析图像性能"""
        stars_detected = self.detect_stars_count(image)
        snr = self.calculate_snr(image)
        centroid_error = self.estimate_centroid_error(image)

        # 理论精度（根据论文公式1）
        theoretical_error_arcsec = (self.config.fov_deg / self.config.resolution_x *
                                    centroid_error / np.sqrt(max(1, stars_detected))) * 206265

        return {
            'stars_detected': stars_detected,
            'snr': snr,
            'centroid_error_pixels': centroid_error,
            'theoretical_error_arcsec': theoretical_error_arcsec,
            'angular_velocity': self.config.angular_velocity_deg_per_s,
            'exposure_time_ms': self.config.exposure_time_ms
        }

    def visualize_image(self, image: np.ndarray, title: str = "模拟星图",
                        save_path: Optional[str] = None, show_stars: bool = True):
        """可视化星图"""
        plt.figure(figsize=(12, 10))
        log_image = np.log1p(image)

        plt.imshow(log_image, cmap='gray', origin='lower')
        plt.colorbar(label='log(1 + DN)')
        plt.title(title)
        plt.xlabel('像素 (X)')
        plt.ylabel('像素 (Y)')

        if show_stars:
            for star in self.star_catalog:
                x, y = self.star_position_to_pixel(star.ra_deg, star.dec_deg)
                if 0 <= x < self.config.resolution_x and 0 <= y < self.config.resolution_y:
                    plt.plot(x, y, 'r+', markersize=5, markeredgewidth=1)
                    plt.text(x + 5, y + 5, f'{star.magnitude:.1f}',
                             color='yellow', fontsize=8, alpha=0.7)

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"图像已保存到: {save_path}")

        plt.show()


def test_static_simulation():
    """测试静态星图模拟"""
    print("=" * 60)
    print("静态星图模拟测试")
    print("=" * 60)

    # 创建配置
    config = StarTrackerConfig(
        focal_length_mm=50.0,
        aperture_diameter_mm=25.0,
        fov_deg=10.0,
        resolution_x=1024,
        resolution_y=1024,
        exposure_time_ms=200,
        psf_sigma_pixels=0.8,
        defocus_level=1.0,
        angular_velocity_deg_per_s=0.0  # 静态
    )

    # 创建模拟器
    simulator = StarSimulator(config)

    # 生成星图
    print("生成静态星图...")
    star_image = simulator.generate_star_image(apply_motion_blur=False)

    # 分析性能
    analysis = simulator.analyze_image(star_image)
    print(f"\n性能分析:")
    print(f"  检测到星点数量: {analysis['stars_detected']}")
    print(f"  信噪比(SNR): {analysis['snr']:.2f}")
    print(f"  质心估计误差: {analysis['centroid_error_pixels']:.3f} 像素")
    print(f"  理论姿态误差: {analysis['theoretical_error_arcsec']:.2f} 角秒")

    # 可视化
    output_dir = r"D:\1StarTracker\data\raw"
    os.makedirs(output_dir, exist_ok=True)

    simulator.visualize_image(
        star_image,
        title=f"静态模拟星图 - FOV: {config.fov_deg}°, 曝光: {config.exposure_time_ms}ms",
        save_path=os.path.join(output_dir, "static_star_field.png")
    )

    # 保存数据
    np.save(os.path.join(output_dir, "static_star_field.npy"), star_image)

    # 保存星表
    star_info = []
    for star in simulator.star_catalog:
        x, y = simulator.star_position_to_pixel(star.ra_deg, star.dec_deg)
        flux = simulator.calculate_star_flux(star.magnitude)
        star_info.append({
            'ra_deg': star.ra_deg,
            'dec_deg': star.dec_deg,
            'magnitude': star.magnitude,
            'pixel_x': x,
            'pixel_y': y,
            'flux_electrons': flux
        })

    df = pd.DataFrame(star_info)
    df.to_csv(os.path.join(output_dir, "static_star_catalog.csv"), index=False)

    print(f"\n数据已保存到: {output_dir}")
    print("静态模拟测试完成!")
    print("=" * 60)


def test_dynamic_simulation():
    """测试动态星图模拟"""
    print("\n" + "=" * 60)
    print("动态星图模拟测试")
    print("=" * 60)

    angular_velocities = [0, 1, 3, 5, 10]  # °/s
    results = []

    for w in angular_velocities:
        print(f"\n测试角速度: {w} °/s")

        config = StarTrackerConfig(
            focal_length_mm=50.0,
            aperture_diameter_mm=25.0,
            fov_deg=10.0,
            resolution_x=1024,
            resolution_y=1024,
            exposure_time_ms=200,
            psf_sigma_pixels=0.8,
            defocus_level=1.0,
            angular_velocity_deg_per_s=w
        )

        simulator = StarSimulator(config)

        # 生成星图（应用运动模糊）
        star_image = simulator.generate_star_image(apply_motion_blur=True)

        # 分析性能
        analysis = simulator.analyze_image(star_image)
        results.append(analysis)

        print(f"  检测到星点: {analysis['stars_detected']}")
        print(f"  SNR: {analysis['snr']:.2f}")
        print(f"  理论误差: {analysis['theoretical_error_arcsec']:.2f} 角秒")

        # 保存动态图像
        if w in [0, 3, 10]:  # 保存几个关键速度的图像
            output_dir = r"D:\1StarTracker\data\raw"
            simulator.visualize_image(
                star_image,
                title=f"动态模拟星图 - {w}°/s",
                save_path=os.path.join(output_dir, f"dynamic_{w}deg_per_s.png"),
                show_stars=False
            )

    # 保存动态测试结果
    results_df = pd.DataFrame(results)
    output_dir = r"D:\1StarTracker\data\raw"
    results_df.to_csv(os.path.join(output_dir, "dynamic_performance.csv"), index=False)

    # 绘制性能变化图
    plt.figure(figsize=(10, 8))

    plt.subplot(2, 2, 1)
    plt.plot(results_df['angular_velocity'], results_df['stars_detected'], 'bo-')
    plt.xlabel('角速度 (°/s)')
    plt.ylabel('检测到星点数量')
    plt.title('星点数量 vs 角速度')
    plt.grid(True, alpha=0.3)

    plt.subplot(2, 2, 2)
    plt.plot(results_df['angular_velocity'], results_df['snr'], 'ro-')
    plt.xlabel('角速度 (°/s)')
    plt.ylabel('信噪比 (SNR)')
    plt.title('信噪比 vs 角速度')
    plt.grid(True, alpha=0.3)

    plt.subplot(2, 2, 3)
    plt.plot(results_df['angular_velocity'], results_df['centroid_error_pixels'], 'go-')
    plt.xlabel('角速度 (°/s)')
    plt.ylabel('质心误差 (像素)')
    plt.title('质心误差 vs 角速度')
    plt.grid(True, alpha=0.3)

    plt.subplot(2, 2, 4)
    plt.plot(results_df['angular_velocity'], results_df['theoretical_error_arcsec'], 'mo-')
    plt.xlabel('角速度 (°/s)')
    plt.ylabel('理论姿态误差 (角秒)')
    plt.title('姿态误差 vs 角速度')
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "dynamic_performance_analysis.png"), dpi=150)

    print(f"\n动态测试结果已保存到: {output_dir}")
    print("动态模拟测试完成!")
    print("=" * 60)


def main():
    """主函数"""
    # 创建数据目录
    data_dir = r"D:\1StarTracker\data"
    raw_dir = os.path.join(data_dir, "raw")
    processed_dir = os.path.join(data_dir, "processed")

    for dir_path in [data_dir, raw_dir, processed_dir]:
        os.makedirs(dir_path, exist_ok=True)

    # 运行测试
    test_static_simulation()
    test_dynamic_simulation()

    print("\n所有测试完成！")
    print(f"数据保存在: {data_dir}")


if __name__ == "__main__":
    main()