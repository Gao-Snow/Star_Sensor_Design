# src/software/algorithms/centroid.py
"""
质心算法实现
根据论文第IV节：Star centroiding algorithms
实现加权质心、高斯拟合等算法，验证亚像素精度
"""
import numpy as np
from typing import Tuple, Optional, List, Dict
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.ndimage import center_of_mass, label
import os

# ========== 修复中文字体问题 ==========
import matplotlib

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']  # 用黑体或微软雅黑
matplotlib.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


# 检查并设置中文字体
def set_chinese_font():
    """设置中文字体"""
    # Windows系统常见中文字体路径
    font_paths = [
        'C:/Windows/Fonts/simhei.ttf',  # 黑体
        'C:/Windows/Fonts/msyh.ttc',  # 微软雅黑
        'C:/Windows/Fonts/simsun.ttc',  # 宋体
    ]

    for font_path in font_paths:
        if os.path.exists(font_path):
            try:
                font_prop = matplotlib.font_manager.FontProperties(fname=font_path)
                matplotlib.rcParams['font.sans-serif'] = [font_prop.get_name(), 'DejaVu Sans']
                print(f"已设置中文字体: {font_prop.get_name()}")
                return
            except:
                continue

    print("警告: 未找到中文字体，图表中文字符可能无法正常显示")
    matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 使用默认字体


# 调用字体设置
set_chinese_font()


# =====================================


def weighted_centroid(image: np.ndarray, threshold: float = None) -> Tuple[float, float]:
    """
    加权质心算法（论文公式9-11）

    参数:
    ----------
    image : np.ndarray
        输入图像区域（通常3x3, 5x5, 7x7）
    threshold : float, optional
        阈值，低于此值的像素不参与计算

    返回:
    -------
    (x_centroid, y_centroid) : 质心坐标（相对于ROI中心）
    """
    h, w = image.shape

    # 如果没有提供阈值，使用自适应阈值
    if threshold is None:
        threshold = np.mean(image) + 2 * np.std(image)

    # 创建权重掩码
    mask = image > threshold

    if not np.any(mask):
        return w / 2, h / 2  # 返回中心

    # 计算像素坐标网格
    y_coords, x_coords = np.mgrid[0:h, 0:w]

    # 加权质心计算
    total_intensity = np.sum(image[mask])
    x_centroid = np.sum(x_coords[mask] * image[mask]) / total_intensity
    y_centroid = np.sum(y_coords[mask] * image[mask]) / total_intensity

    return x_centroid, y_centroid


def gaussian_2d_fit(image: np.ndarray) -> Tuple[float, float, Dict]:
    """
    二维高斯拟合质心算法（更高精度）

    拟合函数: f(x,y) = A * exp(-((x-x0)²/(2σx²) + (y-y0)²/(2σy²))) + B

    返回:
    -------
    (x_centroid, y_centroid) : 拟合质心
    params : 拟合参数字典
    """
    h, w = image.shape

    # 准备数据
    x = np.arange(w)
    y = np.arange(h)
    X, Y = np.meshgrid(x, y)
    Z = image.ravel()

    # 初始猜测 - 使用加权质心
    x0_guess, y0_guess = weighted_centroid(image)

    # 确保初始猜测在图像范围内
    x0_guess = np.clip(x0_guess, 0.5, w - 0.5)
    y0_guess = np.clip(y0_guess, 0.5, h - 0.5)

    A_guess = np.max(image) - np.min(image)
    sigma_guess = 1.0  # 像素
    B_guess = np.min(image)

    # 定义2D高斯函数
    def gaussian_2d(xy, A, x0, y0, sigma_x, sigma_y, B):
        x, y = xy
        return (A * np.exp(-((x - x0) ** 2 / (2 * sigma_x ** 2) + (y - y0) ** 2 / (2 * sigma_y ** 2))) + B).ravel()

    try:
        # 更宽松的边界，确保拟合可行
        bounds = ([0, 0.5, 0.5, 0.2, 0.2, 0],
                  [np.inf, w - 0.5, h - 0.5, 3, 3, np.inf])  # 缩小sigma范围

        # 确保初始猜测在边界内
        initial_guess = [
            A_guess,
            np.clip(x0_guess, 0.5, w - 0.5),
            np.clip(y0_guess, 0.5, h - 0.5),
            np.clip(sigma_guess, 0.2, 2.0),
            np.clip(sigma_guess, 0.2, 2.0),
            np.clip(B_guess, 0, np.inf)
        ]

        # 如果图像太弱，直接返回加权质心
        if A_guess < 10 or np.sum(image) < 50:
            raise ValueError("Signal too weak for fitting")

        popt, pcov = curve_fit(gaussian_2d, (X.ravel(), Y.ravel()), Z,
                               p0=initial_guess, bounds=bounds, maxfev=10000)

        # 提取参数
        A, x0, y0, sigma_x, sigma_y, B = popt

        return x0, y0, {
            'amplitude': A,
            'x0': x0,
            'y0': y0,
            'sigma_x': sigma_x,
            'sigma_y': sigma_y,
            'background': B,
            'covariance': pcov
        }

    except (RuntimeError, ValueError) as e:
        # 拟合失败，退回加权质心
        print(f"高斯拟合失败，使用加权质心: {e}")
        x0, y0 = weighted_centroid(image)
        return x0, y0, {'method': 'weighted_fallback', 'error': str(e)}


def thresholded_centroid(image: np.ndarray, n_sigma: float = 3.0) -> Tuple[float, float]:
    """
    阈值质心算法（工业常用）

    使用 n_sigma * sigma 作为阈值，只使用高信噪比像素
    """
    # 估计背景和噪声
    background = np.percentile(image, 30)
    noise = np.std(image[image < np.percentile(image, 70)])

    threshold = background + n_sigma * noise

    h, w = image.shape
    y_coords, x_coords = np.mgrid[0:h, 0:w]

    mask = image > threshold

    if not np.any(mask):
        return w / 2, h / 2

    # 减去背景
    signal = image[mask] - background

    total_signal = np.sum(signal)
    x_centroid = np.sum(x_coords[mask] * signal) / total_signal
    y_centroid = np.sum(y_coords[mask] * signal) / total_signal

    return x_centroid, y_centroid


def centroid_error_simulation():
    """
    质心精度仿真（论文图13-15）
    模拟不同光子数、PSF大小、噪声水平下的质心精度
    """
    np.random.seed(42)

    results = []

    # 测试不同光子数
    photon_counts = [100, 500, 1000, 5000, 10000, 50000]
    psf_sigma = 1.0
    roi_size = 9

    for photons in photon_counts:
        errors = []
        for _ in range(100):  # 蒙特卡洛仿真
            # 生成理想星点
            center = roi_size // 2
            x = np.arange(roi_size)
            y = np.arange(roi_size)
            X, Y = np.meshgrid(x, y)

            # 高斯PSF
            ideal = photons * np.exp(-((X - center) ** 2 + (Y - center) ** 2) / (2 * psf_sigma ** 2))

            # 添加泊松噪声
            noisy = np.random.poisson(ideal).astype(float)

            # 计算质心
            x_est, y_est = weighted_centroid(noisy)

            # 计算误差
            error = np.sqrt((x_est - center) ** 2 + (y_est - center) ** 2)
            errors.append(error)

        results.append({
            'photons': photons,
            'mean_error': np.mean(errors),
            'std_error': np.std(errors),
            'method': 'weighted'
        })

    return results


def visualize_centroid_performance():
    """可视化质心算法性能"""
    # 生成测试图像
    roi_size = 9
    center = roi_size // 2

    # 创建理想星点（轻微离焦）
    x = np.arange(roi_size)
    y = np.arange(roi_size)
    X, Y = np.meshgrid(x, y)

    # 不同PSF大小
    psf_sigmas = [0.5, 0.8, 1.0, 1.2, 1.5]

    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.ravel()

    for i, sigma in enumerate(psf_sigmas[:6]):
        if i >= len(axes):
            break

        # 生成图像
        image = 10000 * np.exp(-((X - center) ** 2 + (Y - center) ** 2) / (2 * sigma ** 2))

        # 添加噪声
        noisy = np.random.poisson(image).astype(float)
        noisy = noisy + np.random.normal(0, 5, noisy.shape)  # 读出噪声

        # 计算不同算法的质心
        x_w, y_w = weighted_centroid(noisy)
        x_g, y_g, _ = gaussian_2d_fit(noisy)
        x_t, y_t = thresholded_centroid(noisy)

        # 显示
        ax = axes[i]
        im = ax.imshow(noisy, cmap='hot', origin='lower')
        ax.plot(center, center, 'b+', markersize=10, label='真实中心')
        ax.plot(x_w, y_w, 'go', markersize=8, label='加权质心')
        ax.plot(x_g, y_g, 'rx', markersize=8, label='高斯拟合')
        ax.plot(x_t, y_t, 'c*', markersize=8, label='阈值质心')

        ax.set_title(f'PSF σ={sigma}像素')
        ax.legend(fontsize=8)
        plt.colorbar(im, ax=ax)

    plt.tight_layout()
    plt.savefig(r"D:\1StarTracker\data\processed\centroid_algorithms.png", dpi=150)
    plt.show()


def test_centroid_algorithms():
    """测试质心算法"""
    print("=" * 60)
    print("质心算法测试")
    print("=" * 60)

    # 1. 测试加权质心
    print("\n1. 加权质心算法测试")
    test_image = np.zeros((7, 7))
    test_image[3, 3] = 100  # 中心像素

    x, y = weighted_centroid(test_image)
    print(f"   单点测试: ({x:.3f}, {y:.3f}) - 期望 (3.0, 3.0)")

    # 2. 测试高斯拟合
    print("\n2. 高斯拟合算法测试")
    # 生成高斯星点
    x_coords, y_coords = np.mgrid[0:9, 0:9]
    center = 4.0
    test_gaussian = 1000 * np.exp(-((x_coords - center) ** 2 + (y_coords - center) ** 2) / (2 * 1.0 ** 2))
    test_gaussian = np.random.poisson(test_gaussian)  # 添加噪声

    x_fit, y_fit, params = gaussian_2d_fit(test_gaussian)
    print(f"   高斯拟合: ({x_fit:.3f}, {y_fit:.3f})")
    print(f"   拟合参数: A={params.get('amplitude', 0):.1f}, "
          f"σ=({params.get('sigma_x', 0):.2f}, {params.get('sigma_y', 0):.2f})")

    # 3. 精度仿真
    print("\n3. 质心精度仿真")
    results = centroid_error_simulation()

    print("   光子数 | 平均误差(像素) | 误差标准差")
    print("   " + "-" * 40)
    for r in results:
        print(f"   {r['photons']:6d} | {r['mean_error']:12.4f} | {r['std_error']:11.4f}")

    # 4. 可视化
    print("\n4. 生成可视化图表")
    visualize_centroid_performance()

    print("\n质心算法测试完成!")
    print("=" * 60)


if __name__ == "__main__":
    # 创建输出目录
    import os

    output_dir = r"D:\1StarTracker\data\processed"
    os.makedirs(output_dir, exist_ok=True)

    # 运行测试
    test_centroid_algorithms()