# src/software/algorithms/quest.py
"""
QUEST (QUaternion ESTimator) 姿态解算算法
根据论文[34]：Shuster, M.D., & Oh, S.D. (1981). Three-axis attitude determination from vector observations.
实现从星点观测向量到姿态四元数的解算
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
import numpy.linalg as LA
from dataclasses import dataclass
import matplotlib.pyplot as plt

import matplotlib
import os

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

@dataclass
class AttitudeResult:
    """姿态解算结果"""
    quaternion: np.ndarray  # 四元数 [q0, q1, q2, q3]，q0为标量部分
    rotation_matrix: np.ndarray  # 旋转矩阵 3x3
    euler_angles: Tuple[float, float, float]  # 欧拉角 (roll, pitch, yaw) 或 (φ, θ, ψ)，单位：度
    error_estimate: float  # 误差估计
    valid: bool  # 解是否有效
    num_stars_used: int  # 使用的星点数


class QUEST:
    """QUEST姿态解算器"""

    def __init__(self, use_davenport_q: bool = True):
        """
        初始化QUEST解算器

        参数:
        ----------
        use_davenport_q : bool
            是否使用Davenport Q方法（更稳定），否则使用原始QUEST
        """
        self.use_davenport_q = use_davenport_q

    @staticmethod
    def normalize_vectors(vectors: np.ndarray) -> np.ndarray:
        """归一化向量"""
        norms = LA.norm(vectors, axis=1, keepdims=True)
        norms[norms == 0] = 1  # 避免除零
        return vectors / norms

    def davenport_q_method(self,
                           body_vectors: np.ndarray,
                           inertial_vectors: np.ndarray,
                           weights: Optional[np.ndarray] = None) -> AttitudeResult:
        """
        Davenport Q方法（更稳定的QUEST实现）

        求解 Wahba's problem: min ∑ w_i || b_i - A r_i ||^2
        其中：b_i是星敏感器坐标系观测向量，r_i是天球坐标系参考向量

        参数:
        ----------
        body_vectors : np.ndarray (n x 3)
            星敏感器坐标系下的观测向量
        inertial_vectors : np.ndarray (n x 3)
            天球坐标系下的参考向量
        weights : np.ndarray (n,), optional
            权重向量，通常与星等有关

        返回:
        -------
        AttitudeResult : 姿态解算结果
        """
        n_stars = len(body_vectors)

        # 默认等权重
        if weights is None:
            weights = np.ones(n_stars) / n_stars

        # 归一化向量
        b = self.normalize_vectors(body_vectors)
        r = self.normalize_vectors(inertial_vectors)

        # 计算加权观测矩阵
        B = np.zeros((3, 3))
        for i in range(n_stars):
            B += weights[i] * np.outer(b[i], r[i])

        # 计算 Davenport 矩阵
        S = B + B.T
        sigma = np.trace(B)
        Z = np.array([
            B[1, 2] - B[2, 1],
            B[2, 0] - B[0, 2],
            B[0, 1] - B[1, 0]
        ])

        # 构造 K 矩阵
        K_top = np.hstack([S - sigma * np.eye(3), Z.reshape(3, 1)])
        K_bottom = np.hstack([Z.reshape(1, 3), [[sigma]]])
        K = np.vstack([K_top, K_bottom])

        # 求解最大特征值对应的特征向量（最优四元数）
        eigenvalues, eigenvectors = LA.eig(K)
        max_eigenvalue_idx = np.argmax(eigenvalues.real)
        q_raw = eigenvectors[:, max_eigenvalue_idx].real

        # 确保四元数是单位四元数且标量部分非负
        q = q_raw / LA.norm(q_raw)
        if q[3] < 0:  # q[3]是标量部分（在K矩阵中是第4个元素）
            q = -q

        # 重新排序为 [q0, q1, q2, q3]，其中q0是标量
        quaternion = np.array([q[3], q[0], q[1], q[2]])

        # 从四元数计算旋转矩阵
        R = self.quaternion_to_rotation_matrix(quaternion)

        # 计算欧拉角
        euler = self.rotation_matrix_to_euler(R)

        # 估计误差（残差）
        error = self.calculate_attitude_error(b, r, R, weights)

        return AttitudeResult(
            quaternion=quaternion,
            rotation_matrix=R,
            euler_angles=euler,
            error_estimate=error,
            valid=True,
            num_stars_used=n_stars
        )

    def original_quest(self,
                       body_vectors: np.ndarray,
                       inertial_vectors: np.ndarray,
                       weights: Optional[np.ndarray] = None) -> AttitudeResult:
        """
        原始QUEST算法（数值稳定性较差，但历史意义重要）

        参数:
        ----------
        同上

        返回:
        -------
        AttitudeResult : 姿态解算结果
        """
        n_stars = len(body_vectors)

        if weights is None:
            weights = np.ones(n_stars) / n_stars

        b = self.normalize_vectors(body_vectors)
        r = self.normalize_vectors(inertial_vectors)

        # 计算B矩阵
        B = np.zeros((3, 3))
        for i in range(n_stars):
            B += weights[i] * np.outer(b[i], r[i])

        # 计算辅助变量
        S = B + B.T
        sigma = np.trace(B)

        # 构造矩阵用于求解最优四元数
        delta = LA.det(S)
        kappa = np.trace(LA.adjoint(S))  # adjoint是伴随矩阵

        alpha = sigma ** 2 - kappa
        beta = sigma - LA.trace(LA.adjoint(S))
        gamma = LA.det(np.vstack([
            np.hstack([S, np.array([[sigma]]).T]),
            np.array([np.trace(B), 1])
        ]))

        # 求解特征方程
        a = 1.0
        b_coeff = -(alpha + sigma ** 2)
        c = -(delta + sigma * beta - LA.trace(LA.adjoint(S)) * sigma)
        d = sigma * delta - LA.det(B) * LA.trace(LA.inv(B))

        # 解三次方程求最大特征值
        # 这里简化：使用数值方法
        coeffs = [a, b_coeff, c, d]
        roots = np.roots(coeffs)
        lambda_opt = np.max(roots.real[abs(roots.imag) < 1e-10])

        # 计算最优四元数
        # 原始QUEST算法中的公式
        omega = (lambda_opt + sigma) * np.eye(3) - S
        omega_inv = LA.inv(omega)
        z = np.array([B[1, 2] - B[2, 1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]])

        q_vec = np.dot(omega_inv, z)
        q_scalar = 1.0 / np.sqrt(1 + np.dot(q_vec, q_vec))
        q_vec = q_vec * q_scalar

        quaternion = np.array([q_scalar, q_vec[0], q_vec[1], q_vec[2]])

        # 转换为旋转矩阵
        R = self.quaternion_to_rotation_matrix(quaternion)
        euler = self.rotation_matrix_to_euler(R)
        error = self.calculate_attitude_error(b, r, R, weights)

        return AttitudeResult(
            quaternion=quaternion,
            rotation_matrix=R,
            euler_angles=euler,
            error_estimate=error,
            valid=error < 1.0,  # 简单有效性判断
            num_stars_used=n_stars
        )

    @staticmethod
    def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        四元数转换为旋转矩阵

        参数:
        ----------
        q : np.ndarray (4,)
            四元数 [q0, q1, q2, q3]，q0为标量部分

        返回:
        -------
        np.ndarray (3x3) : 旋转矩阵
        """
        q0, q1, q2, q3 = q

        R = np.array([
            [q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2, 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2]
        ])

        return R

    @staticmethod
    def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
        """
        旋转矩阵转换为欧拉角（Z-Y-X顺序，航空航天常用）

        返回:
        -------
        (roll, pitch, yaw) : 单位：度
        """
        # 检查万向锁
        if abs(R[2, 0]) > 0.999999:
            # 万向锁情况
            yaw = 0
            if R[2, 0] < 0:
                pitch = 90
                roll = np.degrees(np.arctan2(R[0, 1], R[0, 2]))
            else:
                pitch = -90
                roll = np.degrees(np.arctan2(-R[0, 1], -R[0, 2]))
        else:
            pitch = np.degrees(-np.arcsin(R[2, 0]))
            roll = np.degrees(np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch)))
            yaw = np.degrees(np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch)))

        return roll, pitch, yaw

    @staticmethod
    def calculate_attitude_error(body_vectors: np.ndarray,
                                 inertial_vectors: np.ndarray,
                                 R: np.ndarray,
                                 weights: np.ndarray) -> float:
        """
        计算姿态解算误差（损失函数值）
        """
        error = 0
        for i in range(len(body_vectors)):
            b_pred = np.dot(R, inertial_vectors[i])
            diff = body_vectors[i] - b_pred
            error += weights[i] * np.dot(diff, diff)

        return np.sqrt(error / len(body_vectors))

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        欧拉角转换为四元数
        """
        cr = np.cos(np.radians(roll) / 2)
        sr = np.sin(np.radians(roll) / 2)
        cp = np.cos(np.radians(pitch) / 2)
        sp = np.sin(np.radians(pitch) / 2)
        cy = np.cos(np.radians(yaw) / 2)
        sy = np.sin(np.radians(yaw) / 2)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        return np.array([q0, q1, q2, q3])

    def solve(self,
              body_vectors: np.ndarray,
              inertial_vectors: np.ndarray,
              weights: Optional[np.ndarray] = None) -> AttitudeResult:
        """
        主求解函数

        参数:
        ----------
        body_vectors : np.ndarray (n x 3)
            星敏感器坐标系观测向量
        inertial_vectors : np.ndarray (n x 3)
            天球坐标系参考向量
        weights : np.ndarray (n,), optional
            权重

        返回:
        -------
        AttitudeResult : 姿态解算结果
        """
        if len(body_vectors) < 2:
            raise ValueError(f"至少需要2颗星，当前只有{len(body_vectors)}颗")

        if self.use_davenport_q:
            return self.davenport_q_method(body_vectors, inertial_vectors, weights)
        else:
            return self.original_quest(body_vectors, inertial_vectors, weights)


def test_quest_algorithm():
    """测试QUEST算法"""
    print("=" * 60)
    print("QUEST姿态解算算法测试")
    print("=" * 60)

    # 创建QUEST解算器
    quest = QUEST(use_davenport_q=True)

    # 测试1：简单两星测试
    print("\n1. 简单两星测试")

    # 已知姿态：绕Z轴旋转30度
    true_roll, true_pitch, true_yaw = 0, 0, 30
    true_quaternion = quest.euler_to_quaternion(true_roll, true_pitch, true_yaw)
    true_R = quest.quaternion_to_rotation_matrix(true_quaternion)

    # 创建两颗星的天球坐标
    star1_inertial = np.array([1, 0, 0])  # 指向X轴
    star2_inertial = np.array([0, 1, 0])  # 指向Y轴

    # 计算它们在星敏感器坐标系中的坐标（应用旋转）
    star1_body = np.dot(true_R, star1_inertial)
    star2_body = np.dot(true_R, star2_inertial)

    # 添加微小噪声
    noise_level = 0.001
    star1_body += np.random.normal(0, noise_level, 3)
    star2_body += np.random.normal(0, noise_level, 3)

    # 归一化
    star1_body = star1_body / LA.norm(star1_body)
    star2_body = star2_body / LA.norm(star2_body)

    # 准备输入
    body_vectors = np.vstack([star1_body, star2_body])
    inertial_vectors = np.vstack([star1_inertial, star2_inertial])

    # 求解姿态
    result = quest.solve(body_vectors, inertial_vectors)

    print(f"   真实姿态: roll={true_roll:.2f}°, pitch={true_pitch:.2f}°, yaw={true_yaw:.2f}°")
    print(
        f"   解算姿态: roll={result.euler_angles[0]:.2f}°, pitch={result.euler_angles[1]:.2f}°, yaw={result.euler_angles[2]:.2f}°")
    print(f"   误差估计: {result.error_estimate:.6f}")
    print(f"   使用星数: {result.num_stars_used}")

    # 测试2：多星测试（更真实场景）
    print("\n2. 多星测试（5颗星）")

    # 生成随机星点
    np.random.seed(42)
    n_stars = 5

    # 生成天球坐标系中的随机单位向量
    inertial_vectors_multi = np.random.randn(n_stars, 3)
    inertial_vectors_multi = quest.normalize_vectors(inertial_vectors_multi)

    # 应用随机旋转
    random_quaternion = quest.euler_to_quaternion(10, 20, 30)  # 随机姿态
    random_R = quest.quaternion_to_rotation_matrix(random_quaternion)

    # 计算星敏感器坐标系中的观测向量（添加噪声）
    body_vectors_multi = np.zeros((n_stars, 3))
    for i in range(n_stars):
        ideal_body = np.dot(random_R, inertial_vectors_multi[i])
        # 添加噪声（模拟质心误差）
        noise = np.random.normal(0, 0.001, 3)
        body_vectors_multi[i] = ideal_body + noise
        body_vectors_multi[i] = body_vectors_multi[i] / LA.norm(body_vectors_multi[i])

    # 添加权重（模拟亮星权重高）
    weights = np.random.uniform(0.5, 1.5, n_stars)
    weights = weights / np.sum(weights)  # 归一化

    # 求解
    result_multi = quest.solve(body_vectors_multi, inertial_vectors_multi, weights)

    # 计算真实欧拉角
    true_euler_multi = quest.rotation_matrix_to_euler(random_R)

    print(
        f"   真实姿态: roll={true_euler_multi[0]:.2f}°, pitch={true_euler_multi[1]:.2f}°, yaw={true_euler_multi[2]:.2f}°")
    print(
        f"   解算姿态: roll={result_multi.euler_angles[0]:.2f}°, pitch={result_multi.euler_angles[1]:.2f}°, yaw={result_multi.euler_angles[2]:.2f}°")
    print(f"   姿态误差: roll={abs(result_multi.euler_angles[0] - true_euler_multi[0]):.4f}°, "
          f"pitch={abs(result_multi.euler_angles[1] - true_euler_multi[1]):.4f}°, "
          f"yaw={abs(result_multi.euler_angles[2] - true_euler_multi[2]):.4f}°")
    print(f"   误差估计: {result_multi.error_estimate:.6f}")

    # 测试3：误差分析
    print("\n3. 误差分析：噪声水平对精度的影响")

    noise_levels = [0.0001, 0.001, 0.005, 0.01, 0.05]
    n_trials = 50
    n_stars_error = 4

    mean_errors = []
    std_errors = []

    for noise in noise_levels:
        errors = []
        for trial in range(n_trials):
            # 生成随机星点
            inertial_random = np.random.randn(n_stars_error, 3)
            inertial_random = quest.normalize_vectors(inertial_random)

            # 随机姿态
            rand_quat = quest.euler_to_quaternion(
                np.random.uniform(-180, 180),
                np.random.uniform(-90, 90),
                np.random.uniform(-180, 180)
            )
            rand_R = quest.quaternion_to_rotation_matrix(rand_quat)

            # 添加噪声
            body_random = np.zeros((n_stars_error, 3))
            for i in range(n_stars_error):
                ideal = np.dot(rand_R, inertial_random[i])
                noisy = ideal + np.random.normal(0, noise, 3)
                body_random[i] = noisy / LA.norm(noisy)

            # 求解
            result_trial = quest.solve(body_random, inertial_random)
            true_euler_trial = quest.rotation_matrix_to_euler(rand_R)

            # 计算角度误差
            error_angle = np.degrees(2 * np.arccos(abs(np.dot(
                result_trial.quaternion, rand_quat
            ))))
            errors.append(error_angle)

        mean_errors.append(np.mean(errors))
        std_errors.append(np.std(errors))
        print(f"   噪声 {noise:.4f}: 平均误差={np.mean(errors) * 3600:.2f}角秒, 标准差={np.std(errors) * 3600:.2f}角秒")

    # 可视化
    plt.figure(figsize=(10, 6))
    plt.errorbar(noise_levels, np.array(mean_errors) * 3600,
                 yerr=np.array(std_errors) * 3600,
                 fmt='o-', capsize=5)
    plt.xlabel('噪声水平（单位向量分量标准差）')
    plt.ylabel('姿态误差（角秒）')
    plt.title('QUEST算法噪声敏感性分析')
    plt.grid(True, alpha=0.3)
    plt.xscale('log')

    output_dir = r"D:\1StarTracker\data\processed"
    import os
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, "quest_noise_analysis.png"), dpi=150)
    plt.show()

    print("\nQUEST算法测试完成!")
    print("=" * 60)

    return quest, result_multi


def create_integration_demo():
    """创建与质心算法集成的演示"""
    print("\n" + "=" * 60)
    print("QUEST与质心算法集成演示")
    print("=" * 60)

    # 导入质心算法（假设在同一目录）
    try:
        from centroid import weighted_centroid, gaussian_2d_fit

        # 模拟一个简单场景：从像素坐标到姿态解算
        print("模拟从像素坐标到姿态解算的完整流程:")

        # 1. 假设我们有相机参数
        focal_length_pixels = 1000  # 像素
        image_center_x = 512
        image_center_y = 512

        # 2. 模拟检测到的星点像素坐标（已知对应天球坐标）
        # 星点1：像素坐标，对应天球向量
        pixel_coords = np.array([
            [600, 500],  # 星点1
            [450, 550],  # 星点2
            [550, 600],  # 星点3
        ])

        # 对应的天球坐标（从星表中获得）
        # 这里我们假设这些是已知的
        inertial_vectors = np.array([
            [0.8, 0.2, 0.5],  # 星点1
            [0.3, 0.9, 0.1],  # 星点2
            [0.1, 0.4, 0.9],  # 星点3
        ])
        inertial_vectors = QUEST.normalize_vectors(inertial_vectors)

        # 3. 像素坐标转换为观测向量（相机模型）
        body_vectors = []
        for px, py in pixel_coords:
            # 简单的针孔相机模型
            x = (px - image_center_x) / focal_length_pixels
            y = (py - image_center_y) / focal_length_pixels
            z = 1.0
            vec = np.array([x, y, z])
            vec = vec / LA.norm(vec)
            body_vectors.append(vec)

        body_vectors = np.array(body_vectors)

        # 4. 使用QUEST解算姿态
        quest = QUEST(use_davenport_q=True)
        result = quest.solve(body_vectors, inertial_vectors)

        print(f"   检测到星点数: {len(pixel_coords)}")
        print(f"   解算姿态: roll={result.euler_angles[0]:.2f}°, "
              f"pitch={result.euler_angles[1]:.2f}°, "
              f"yaw={result.euler_angles[2]:.2f}°")
        print(f"   四元数: [{result.quaternion[0]:.4f}, {result.quaternion[1]:.4f}, "
              f"{result.quaternion[2]:.4f}, {result.quaternion[3]:.4f}]")

        # 5. 验证：将天球向量转换回像素坐标
        print("\n验证（反投影误差）:")
        R_inv = result.rotation_matrix.T  # 旋转矩阵的逆是转置

        for i, (inertial_vec, pixel_orig) in enumerate(zip(inertial_vectors, pixel_coords)):
            # 将天球向量转换到星敏感器坐标系
            body_pred = np.dot(R_inv, inertial_vec)

            # 再将观测向量转换回像素坐标
            if abs(body_pred[2]) > 1e-10:  # 避免除零
                px_pred = body_pred[0] / body_pred[2] * focal_length_pixels + image_center_x
                py_pred = body_pred[1] / body_pred[2] * focal_length_pixels + image_center_y

                error_pixel = np.sqrt((px_pred - pixel_orig[0]) ** 2 + (py_pred - pixel_orig[1]) ** 2)
                print(f"   星点{i + 1}: 原始像素({pixel_orig[0]:.1f}, {pixel_orig[1]:.1f}), "
                      f"反投影像素({px_pred:.1f}, {py_pred:.1f}), 误差={error_pixel:.3f}像素")

        print("\n集成演示完成!")

    except ImportError as e:
        print(f"无法导入质心算法模块: {e}")
        print("请确保centroid.py在同一目录下")


if __name__ == "__main__":
    # 运行测试
    test_quest_algorithm()

    # 运行集成演示
    create_integration_demo()