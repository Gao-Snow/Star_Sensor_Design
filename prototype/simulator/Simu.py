import numpy as np
import matplotlib.pyplot as plt
import matplotlib

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


def generate_star_image_with_parameters(image_size=512, sigma=2.0, noise_level=0.02, star_intensity=1.0):
    """
    改进的星图生成函数

    参数：
    image_size: 图像大小
    sigma: 高斯星点的大小（标准差）
    noise_level: 噪声强度（标准差）
    star_intensity: 星点峰值强度
    """
    # 创建全黑图像
    image = np.zeros((image_size, image_size))

    center = image_size // 2

    # 创建坐标网格
    x, y = np.meshgrid(np.arange(image_size), np.arange(image_size))

    # 生成高斯星点
    # 注意：这里生成的是完美的高斯分布，峰值在中心为1.0
    image = star_intensity * np.exp(-((x - center) ** 2 + (y - center) ** 2) / (2.0 * sigma ** 2))

    # 保存无噪声的原始图像
    clean_image = image.copy()

    # 添加噪声（减小噪声强度）
    noise = np.random.normal(0, noise_level, image.shape)
    noisy_image = image + noise

    return clean_image, noisy_image


if __name__ == "__main__":
    # 生成星图
    clean_img, noisy_img = generate_star_image_with_parameters(
        image_size=512,
        sigma=3.0,  # 增大sigma让星点更大
        noise_level=0.01,  # 减小噪声
        star_intensity=5.0  # 增强星点亮度
    )

    # 创建对比图
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    # 1. 无噪声星图
    im1 = axes[0, 0].imshow(clean_img, cmap='gray', vmin=0, vmax=1)
    axes[0, 0].set_title("无噪声星图")
    axes[0, 0].set_xlabel(f"中心像素值: {clean_img[256, 256]:.3f}")
    plt.colorbar(im1, ax=axes[0, 0])

    # 2. 有噪声星图（自动缩放）
    im2 = axes[0, 1].imshow(noisy_img, cmap='gray')
    axes[0, 1].set_title("有噪声星图（自动缩放）")
    axes[0, 1].set_xlabel(f"范围: [{noisy_img.min():.3f}, {noisy_img.max():.3f}]")
    plt.colorbar(im2, ax=axes[0, 1])

    # 3. 有噪声星图（固定范围0-1）
    im3 = axes[0, 2].imshow(noisy_img, cmap='gray', vmin=0, vmax=1)
    axes[0, 2].set_title("有噪声星图（固定范围0-1）")
    plt.colorbar(im3, ax=axes[0, 2])

    # 4. 中心行截面图
    center_row = clean_img[256, :]
    noisy_center_row = noisy_img[256, :]

    axes[1, 0].plot(center_row, 'b-', label='无噪声', linewidth=2)
    axes[1, 0].plot(noisy_center_row, 'r-', label='有噪声', alpha=0.7)
    axes[1, 0].set_title("中心行光强分布")
    axes[1, 0].set_xlabel("像素位置")
    axes[1, 0].set_ylabel("光强")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 5. 三维表面图
    from mpl_toolkits.mplot3d import Axes3D

    X, Y = np.meshgrid(np.arange(200, 312), np.arange(200, 312))  # 中心区域
    Z = noisy_img[200:312, 200:312]

    ax3d = fig.add_subplot(1, 3, 3, projection='3d')
    surf = ax3d.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)
    ax3d.set_title("星点三维形貌")
    ax3d.set_xlabel("X像素")
    ax3d.set_ylabel("Y像素")
    ax3d.set_zlabel("光强")
    fig.colorbar(surf, ax=ax3d, shrink=0.5, aspect=5)

    plt.tight_layout()
    plt.savefig("improved_star_image.png", dpi=300, bbox_inches='tight')
    plt.show()

    # 统计信息
    print("=" * 50)
    print("星图统计信息：")
    print("=" * 50)
    print(f"图像尺寸: {clean_img.shape}")
    print(f"\n无噪声图像:")
    print(f"  最小值: {clean_img.min():.6f}")
    print(f"  最大值: {clean_img.max():.6f} (中心亮度)")
    print(f"  平均值: {clean_img.mean():.6f}")
    print(f"  标准差: {clean_img.std():.6f}")

    print(f"\n有噪声图像:")
    print(f"  最小值: {noisy_img.min():.6f}")
    print(f"  最大值: {noisy_img.max():.6f}")
    print(f"  平均值: {noisy_img.mean():.6f}")
    print(f"  标准差: {noisy_img.std():.6f}")

    # 信噪比计算
    signal_power = np.mean(clean_img ** 2)
    noise_power = np.mean((noisy_img - clean_img) ** 2)
    snr = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else float('inf')
    print(f"\n信噪比(SNR): {snr:.2f} dB")

    # 星点参数
    print(f"\n星点参数:")
    print(f"  高斯σ: 3.0像素")
    print(f"  半高全宽(FWHM): {2.355 * 3.0:.2f}像素")
    print(f"  星点直径(3σ): {6 * 3.0:.2f}像素")