# src/software/algorithms/star_identification.py
"""
星图识别算法 - 三角形匹配
实现从检测到的星点到星表匹配的算法
根据论文：三角形匹配、多边形匹配等算法
"""

import numpy as np
from typing import List, Tuple, Dict, Set, Optional, Any
from dataclasses import dataclass
import heapq
import itertools
from collections import defaultdict
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import pickle
import os


# ========== 中文字体设置 ==========
def set_chinese_font():
    """设置中文字体"""
    import matplotlib
    font_paths = [
        'C:/Windows/Fonts/simhei.ttf',
        'C:/Windows/Fonts/msyh.ttc',
        'C:/Windows/Fonts/simsun.ttc',
    ]

    for font_path in font_paths:
        if os.path.exists(font_path):
            try:
                matplotlib.font_manager.fontManager.addfont(font_path)
                font_name = matplotlib.font_manager.FontProperties(fname=font_path).get_name()
                matplotlib.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans']
                matplotlib.rcParams['axes.unicode_minus'] = False
                print(f"已设置中文字体: {font_name}")
                return
            except:
                continue

    print("警告: 未找到中文字体，使用默认字体")
    matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False


set_chinese_font()


# ==================================


@dataclass
class StarCatalogEntry:
    """星表条目"""
    star_id: int
    ra_deg: float  # 赤经（度）
    dec_deg: float  # 赤纬（度）
    magnitude: float  # 星等
    vector: np.ndarray  # 单位向量 [x, y, z]

    def __post_init__(self):
        """确保向量是单位向量"""
        if self.vector is not None:
            norm = np.linalg.norm(self.vector)
            if norm > 0:
                self.vector = self.vector / norm


@dataclass
class DetectedStar:
    """检测到的星点"""
    pixel_x: float  # 像素坐标X
    pixel_y: float  # 像素坐标Y
    magnitude_est: float  # 估计星等
    flux: float  # 流量
    roi: Optional[np.ndarray] = None  # 感兴趣区域图像

    def __str__(self):
        return f"Star(pixel=({self.pixel_x:.2f}, {self.pixel_y:.2f}), mag={self.magnitude_est:.2f})"


@dataclass
class TriangleFeature:
    """三角形特征"""
    star_indices: Tuple[int, int, int]  # 三颗星的索引
    angles: Tuple[float, float, float]  # 三个内角（弧度），已排序
    sides: Tuple[float, float, float]  # 三边长度（弧度）
    feature_key: str  # 特征键，用于快速匹配

    def __str__(self):
        return (f"Triangle(ids={self.star_indices}, "
                f"angles={np.degrees(self.angles)}, "
                f"sides={np.degrees(self.sides)})")


class TriangleStarIdentifier:
    """基于三角形匹配的星图识别算法"""

    def __init__(self,
                 catalog_entries: Optional[List[StarCatalogEntry]] = None,
                 max_angle_error_deg: float = 0.1,
                 min_angle_deg: float = 0.5,
                 max_angle_deg: float = 20.0):
        """
        初始化星图识别器

        参数:
        ----------
        catalog_entries : List[StarCatalogEntry], optional
            星表数据，如果为None则使用内置模拟星表
        max_angle_error_deg : float
            最大角度误差（度），用于匹配容忍度
        min_angle_deg : float
            三角形最小内角（度），避免太尖锐的三角形
        max_angle_deg : float
            三角形最大内角（度），避免太扁平的三角形
        """
        self.max_angle_error = np.radians(max_angle_error_deg)
        self.min_angle = np.radians(min_angle_deg)
        self.max_angle = np.radians(max_angle_deg)

        # 加载或生成星表
        if catalog_entries is None:
            print("使用模拟星表...")
            self.catalog = self._generate_simulated_catalog()
        else:
            self.catalog = catalog_entries

        print(f"星表包含 {len(self.catalog)} 颗恒星")

        # 构建特征数据库
        self.triangle_db = self._build_triangle_database()
        print(f"三角形数据库包含 {len(self.triangle_db)} 个特征")

        # 创建快速查找结构
        self._build_index()

    def _generate_simulated_catalog(self, n_stars: int = 1000) -> List[StarCatalogEntry]:
        """生成模拟星表"""
        stars = []

        # 星等分布：指数分布，亮星少，暗星多
        magnitudes = np.random.exponential(scale=2.0, size=n_stars) + 2.0
        magnitudes = np.clip(magnitudes, 0.0, 6.5)

        # 在全天球均匀分布（使用球面均匀分布）
        # 生成均匀分布的随机点
        for i in range(n_stars):
            # 球面均匀分布
            dec_rad = np.arcsin(2 * np.random.random() - 1)  # 赤纬弧度
            ra_rad = 2 * np.pi * np.random.random()  # 赤经弧度

            # 转换为单位向量
            dec_deg = np.degrees(dec_rad)
            ra_deg = np.degrees(ra_rad)

            # 计算单位向量
            x = np.cos(dec_rad) * np.cos(ra_rad)
            y = np.cos(dec_rad) * np.sin(ra_rad)
            z = np.sin(dec_rad)
            vector = np.array([x, y, z])

            stars.append(StarCatalogEntry(
                star_id=i,
                ra_deg=ra_deg,
                dec_deg=dec_deg,
                magnitude=magnitudes[i],
                vector=vector
            ))

        return stars

    def _build_triangle_database(self, neighbors_per_star: int = 15) -> Dict[str, List[Tuple[int, int, int]]]:
        """
        构建三角形特征数据库

        参数:
        ----------
        neighbors_per_star : int
            每颗星考虑的最邻近星数量

        返回:
        -------
        Dict[str, List[Tuple[int, int, int]]]
            特征键到三角形索引列表的映射
        """
        print("构建三角形数据库...")

        n_stars = len(self.catalog)
        vectors = np.array([star.vector for star in self.catalog])

        # 使用KDTree快速查找邻近星
        tree = KDTree(vectors)

        triangle_db = defaultdict(list)
        triangle_count = 0

        # 为每颗星生成特征三角形
        for i in range(n_stars):
            # 查找最近的neighbors_per_star颗星（包括自身）
            distances, indices = tree.query(vectors[i], k=neighbors_per_star + 1)

            # 排除自身，取最近的neighbors_per_star颗星
            neighbor_indices = indices[1:]

            # 生成所有可能的三角形组合
            for j_idx, j in enumerate(neighbor_indices):
                for k in neighbor_indices[j_idx + 1:]:
                    if i == j or i == k or j == k:
                        continue

                    # 计算三角形特征
                    triangle = self._compute_triangle_feature(i, j, k)

                    if triangle is not None:
                        # 使用特征键作为字典键
                        triangle_db[triangle.feature_key].append(triangle.star_indices)
                        triangle_count += 1

        print(f"生成了 {triangle_count} 个三角形特征")
        return dict(triangle_db)

    def _compute_triangle_feature(self, i: int, j: int, k: int) -> Optional[TriangleFeature]:
        """
        计算三角形特征

        参数:
        ----------
        i, j, k : int
            三颗星的索引

        返回:
        -------
        TriangleFeature or None
            如果三角形有效则返回特征，否则返回None
        """
        v_i = self.catalog[i].vector
        v_j = self.catalog[j].vector
        v_k = self.catalog[k].vector

        # 计算三边长度（角度）
        d_ij = np.arccos(np.clip(np.dot(v_i, v_j), -1.0, 1.0))
        d_ik = np.arccos(np.clip(np.dot(v_i, v_k), -1.0, 1.0))
        d_jk = np.arccos(np.clip(np.dot(v_j, v_k), -1.0, 1.0))

        # 检查边长是否在合理范围内
        if (d_ij < self.min_angle or d_ik < self.min_angle or d_jk < self.min_angle or
                d_ij > self.max_angle or d_ik > self.max_angle or d_jk > self.max_angle):
            return None

        # 计算三个内角（使用余弦定理）
        # 注意：我们使用的是球面三角形，但角度较小的情况下，平面近似足够
        # 对于小角度，平面三角形近似足够准确
        cos_angle_i = (np.cos(d_jk) - np.cos(d_ij) * np.cos(d_ik)) / (np.sin(d_ij) * np.sin(d_ik))
        cos_angle_j = (np.cos(d_ik) - np.cos(d_ij) * np.cos(d_jk)) / (np.sin(d_ij) * np.sin(d_jk))
        cos_angle_k = (np.cos(d_ij) - np.cos(d_ik) * np.cos(d_jk)) / (np.sin(d_ik) * np.sin(d_jk))

        # 防止数值误差导致超出[-1, 1]
        cos_angle_i = np.clip(cos_angle_i, -1.0, 1.0)
        cos_angle_j = np.clip(cos_angle_j, -1.0, 1.0)
        cos_angle_k = np.clip(cos_angle_k, -1.0, 1.0)

        angle_i = np.arccos(cos_angle_i)
        angle_j = np.arccos(cos_angle_j)
        angle_k = np.arccos(cos_angle_k)

        # 检查内角是否有效
        if (angle_i < self.min_angle or angle_j < self.min_angle or angle_k < self.min_angle or
                angle_i > np.pi - self.min_angle or angle_j > np.pi - self.min_angle or angle_k > np.pi - self.min_angle):
            return None

        # 对内角进行排序（确保特征与顺序无关）
        sorted_angles = tuple(sorted([angle_i, angle_j, angle_k]))

        # 创建特征键：将角度量化为字符串，用于快速匹配
        # 这里将角度量化为0.01度精度
        quantized_angles = tuple(int(angle * 10000) for angle in sorted_angles)  # 0.01度精度
        feature_key = f"{quantized_angles[0]}_{quantized_angles[1]}_{quantized_angles[2]}"

        return TriangleFeature(
            star_indices=(i, j, k),
            angles=sorted_angles,
            sides=(d_ij, d_ik, d_jk),
            feature_key=feature_key
        )

    def _build_index(self):
        """构建快速索引结构"""
        # 创建角度到特征键的映射
        self.angle_to_keys = {}

        for key in self.triangle_db.keys():
            # 从特征键解析角度
            parts = key.split('_')
            if len(parts) == 3:
                angles = tuple(int(p) / 10000.0 for p in parts)  # 转换回弧度
                self.angle_to_keys[angles] = key

    def pixel_to_vector(self, pixel_x: float, pixel_y: float,
                        focal_length_pixels: float,
                        center_x: float, center_y: float) -> np.ndarray:
        """
        将像素坐标转换为单位向量（针孔相机模型）

        参数:
        ----------
        pixel_x, pixel_y : float
            像素坐标
        focal_length_pixels : float
            焦距（像素）
        center_x, center_y : float
            主点坐标（像素）

        返回:
        -------
        np.ndarray : 单位向量 [x, y, z]
        """
        # 计算相对于主点的坐标
        x = (pixel_x - center_x) / focal_length_pixels
        y = (pixel_y - center_y) / focal_length_pixels

        # 计算z分量（假设相机朝向z轴正方向）
        z = 1.0

        # 归一化
        vector = np.array([x, y, z])
        norm = np.linalg.norm(vector)
        if norm > 0:
            vector = vector / norm

        return vector

    def identify_stars(self,
                       detected_stars: List[DetectedStar],
                       camera_params: Dict[str, float],
                       min_matches: int = 4,
                       debug: bool = False) -> Dict[int, int]:
        """
        识别星点 - 主函数

        参数:
        ----------
        detected_stars : List[DetectedStar]
            检测到的星点列表
        camera_params : Dict[str, float]
            相机参数，包括：
            - focal_length_pixels: 焦距（像素）
            - center_x: 主点X坐标
            - center_y: 主点Y坐标
        min_matches : int
            最小匹配星数要求
        debug : bool
            是否输出调试信息

        返回:
        -------
        Dict[int, int] : 匹配字典 {检测星点索引: 星表ID}
        """
        n_detected = len(detected_stars)
        if n_detected < 3:
            if debug:
                print(f"检测到的星点数量不足: {n_detected} < 3")
            return {}

        # 提取相机参数
        focal_length = camera_params.get('focal_length_pixels', 1000.0)
        center_x = camera_params.get('center_x', 512.0)
        center_y = camera_params.get('center_y', 512.0)

        # 将像素坐标转换为观测向量
        detected_vectors = []
        for star in detected_stars:
            vector = self.pixel_to_vector(
                star.pixel_x, star.pixel_y,
                focal_length, center_x, center_y
            )
            detected_vectors.append(vector)

        detected_vectors = np.array(detected_vectors)

        # 1. 生成观测三角形
        if debug:
            print(f"从 {n_detected} 颗检测星点生成候选三角形...")

        observed_triangles = []
        triangle_info = []  # 保存三角形信息

        # 使用组合生成所有可能的三角形
        for i, j, k in itertools.combinations(range(n_detected), 3):
            # 计算三角形特征
            triangle = self._compute_triangle_from_vectors(
                detected_vectors[i], detected_vectors[j], detected_vectors[k],
                (i, j, k)
            )

            if triangle is not None:
                observed_triangles.append(triangle)
                triangle_info.append((i, j, k, triangle))

        if debug:
            print(f"生成了 {len(observed_triangles)} 个有效观测三角形")

        if len(observed_triangles) == 0:
            if debug:
                print("没有生成有效的观测三角形")
            return {}

        # 2. 在数据库中匹配三角形
        if debug:
            print("开始三角形匹配...")

        # 投票机制：记录每对（检测星点，星表星）的投票数
        vote_matrix = np.zeros((n_detected, len(self.catalog)), dtype=int)

        for triangle_idx, (i, j, k, triangle) in enumerate(triangle_info):
            # 在数据库中查找匹配的三角形
            matches = self._find_matching_triangles(triangle)

            # 对每个匹配，为对应的星点投票
            for catalog_triangle in matches:
                # catalog_triangle是星表三角形的索引三元组
                # 假设对应关系：检测三角形(i,j,k)对应星表三角形(cat_i, cat_j, cat_k)
                # 注意：这里假设了对应顺序相同，实际中需要更复杂的匹配
                vote_matrix[i, catalog_triangle[0]] += 1
                vote_matrix[j, catalog_triangle[1]] += 1
                vote_matrix[k, catalog_triangle[2]] += 1

        # 3. 根据投票结果确定匹配
        matches = {}
        used_catalog_stars = set()

        # 对每个检测星点，选择投票数最高的星表星
        for det_idx in range(n_detected):
            votes = vote_matrix[det_idx]
            max_vote = np.max(votes)

            if max_vote > 0:  # 有投票
                # 找到所有获得最高投票的候选
                candidates = np.where(votes == max_vote)[0]

                # 排除已经被匹配的星表星
                available_candidates = [c for c in candidates if c not in used_catalog_stars]

                if available_candidates:
                    # 如果有多个候选，选择星等最接近的
                    best_candidate = available_candidates[0]
                    if len(available_candidates) > 1:
                        # 简单策略：选择第一个，实际中可以根据星等进行优化
                        best_candidate = available_candidates[0]

                    matches[det_idx] = best_candidate
                    used_catalog_stars.add(best_candidate)

        if debug:
            print(f"初步匹配到 {len(matches)} 颗星")

        # 4. 验证匹配结果（使用几何一致性）
        if len(matches) >= 3:
            valid_matches = self._verify_matches(matches, detected_vectors, debug)
        else:
            valid_matches = matches

        if debug:
            print(f"验证后匹配到 {len(valid_matches)} 颗星")

        # 检查是否满足最小匹配数要求
        if len(valid_matches) < min_matches:
            if debug:
                print(f"匹配星数不足: {len(valid_matches)} < {min_matches}")
            return {}

        return valid_matches

    def _compute_triangle_from_vectors(self,
                                       v1: np.ndarray,
                                       v2: np.ndarray,
                                       v3: np.ndarray,
                                       indices: Tuple[int, int, int]) -> Optional[TriangleFeature]:
        """
        从三个向量计算三角形特征

        参数:
        ----------
        v1, v2, v3 : np.ndarray
            三个单位向量
        indices : Tuple[int, int, int]
            三个向量的索引

        返回:
        -------
        TriangleFeature or None
        """
        # 计算三边长度（角度）
        d12 = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
        d13 = np.arccos(np.clip(np.dot(v1, v3), -1.0, 1.0))
        d23 = np.arccos(np.clip(np.dot(v2, v3), -1.0, 1.0))

        # 检查边长是否在合理范围内
        if (d12 < self.min_angle or d13 < self.min_angle or d23 < self.min_angle or
                d12 > self.max_angle or d13 > self.max_angle or d23 > self.max_angle):
            return None

        # 计算三个内角
        cos_angle1 = (np.cos(d23) - np.cos(d12) * np.cos(d13)) / (np.sin(d12) * np.sin(d13))
        cos_angle2 = (np.cos(d13) - np.cos(d12) * np.cos(d23)) / (np.sin(d12) * np.sin(d23))
        cos_angle3 = (np.cos(d12) - np.cos(d13) * np.cos(d23)) / (np.sin(d13) * np.sin(d23))

        # 防止数值误差
        cos_angle1 = np.clip(cos_angle1, -1.0, 1.0)
        cos_angle2 = np.clip(cos_angle2, -1.0, 1.0)
        cos_angle3 = np.clip(cos_angle3, -1.0, 1.0)

        angle1 = np.arccos(cos_angle1)
        angle2 = np.arccos(cos_angle2)
        angle3 = np.arccos(cos_angle3)

        # 检查内角是否有效
        if (angle1 < self.min_angle or angle2 < self.min_angle or angle3 < self.min_angle or
                angle1 > np.pi - self.min_angle or angle2 > np.pi - self.min_angle or angle3 > np.pi - self.min_angle):
            return None

        # 对内角进行排序
        sorted_angles = tuple(sorted([angle1, angle2, angle3]))

        # 创建特征键
        quantized_angles = tuple(int(angle * 10000) for angle in sorted_angles)  # 0.01度精度
        feature_key = f"{quantized_angles[0]}_{quantized_angles[1]}_{quantized_angles[2]}"

        return TriangleFeature(
            star_indices=indices,
            angles=sorted_angles,
            sides=(d12, d13, d23),
            feature_key=feature_key
        )

    def _find_matching_triangles(self, triangle: TriangleFeature) -> List[Tuple[int, int, int]]:
        """
        在数据库中查找匹配的三角形

        参数:
        ----------
        triangle : TriangleFeature
            观测三角形特征

        返回:
        -------
        List[Tuple[int, int, int]]
            匹配的星表三角形索引列表
        """
        # 直接使用特征键查找
        if triangle.feature_key in self.triangle_db:
            return self.triangle_db[triangle.feature_key]

        # 如果没有精确匹配，可以尝试容差匹配（这里简化，只返回精确匹配）
        return []

    def _verify_matches(self,
                        matches: Dict[int, int],
                        detected_vectors: np.ndarray,
                        debug: bool = False) -> Dict[int, int]:
        """
        验证匹配结果的几何一致性

        参数:
        ----------
        matches : Dict[int, int]
            初步匹配结果
        detected_vectors : np.ndarray
            检测星点的观测向量
        debug : bool
            是否输出调试信息

        返回:
        -------
        Dict[int, int] : 验证后的匹配结果
        """
        if len(matches) < 3:
            return matches

        # 提取匹配对
        det_indices = list(matches.keys())
        cat_indices = list(matches.values())

        # 计算所有匹配对之间的角度一致性
        errors = []

        # 对于每对匹配对(i,j)，计算检测向量之间的角度和星表向量之间的角度
        for idx1, det_i in enumerate(det_indices):
            for idx2, det_j in enumerate(det_indices[idx1 + 1:], idx1 + 1):
                cat_i = cat_indices[idx1]
                cat_j = cat_indices[idx2]

                # 计算检测向量之间的角度
                v_det_i = detected_vectors[det_i]
                v_det_j = detected_vectors[det_j]
                angle_det = np.arccos(np.clip(np.dot(v_det_i, v_det_j), -1.0, 1.0))

                # 计算星表向量之间的角度
                v_cat_i = self.catalog[cat_i].vector
                v_cat_j = self.catalog[cat_j].vector
                angle_cat = np.arccos(np.clip(np.dot(v_cat_i, v_cat_j), -1.0, 1.0))

                # 计算角度误差
                error = abs(angle_det - angle_cat)
                errors.append(error)

        # 计算平均误差
        if errors:
            mean_error = np.mean(errors)
            std_error = np.std(errors)

            if debug:
                print(f"几何验证: 平均角度误差={np.degrees(mean_error) * 3600:.2f}角秒, "
                      f"标准差={np.degrees(std_error) * 3600:.2f}角秒")

            # 如果误差太大，可能需要剔除一些匹配
            # 这里简化处理：如果平均误差超过阈值，返回空匹配
            if mean_error > self.max_angle_error * 2:
                if debug:
                    print(f"几何验证失败: 平均误差超过阈值")
                return {}

        return matches

    def visualize_identification(self,
                                 detected_stars: List[DetectedStar],
                                 matches: Dict[int, int],
                                 camera_params: Dict[str, float]):
        """
        可视化识别结果

        参数:
        ----------
        detected_stars : List[DetectedStar]
            检测到的星点
        matches : Dict[int, int]
            匹配结果
        camera_params : Dict[str, float]
            相机参数
        """
        fig, axes = plt.subplots(1, 2, figsize=(15, 7))

        # 左图：显示检测到的星点
        ax1 = axes[0]

        # 提取像素坐标
        pixel_x = [star.pixel_x for star in detected_stars]
        pixel_y = [star.pixel_y for star in detected_stars]

        ax1.scatter(pixel_x, pixel_y, c='blue', s=50, alpha=0.7, label='检测星点')

        # 标记匹配的星点
        matched_x = []
        matched_y = []
        for det_idx, cat_idx in matches.items():
            matched_x.append(detected_stars[det_idx].pixel_x)
            matched_y.append(detected_stars[det_idx].pixel_y)

            # 添加标签
            ax1.text(detected_stars[det_idx].pixel_x + 10,
                     detected_stars[det_idx].pixel_y + 10,
                     f'ID:{cat_idx}', fontsize=9, color='red')

        if matched_x:
            ax1.scatter(matched_x, matched_y, c='red', s=100,
                        edgecolors='yellow', linewidth=2, label='匹配星点')

        ax1.set_xlabel('像素X')
        ax1.set_ylabel('像素Y')
        ax1.set_title(f'星点检测与匹配 ({len(matches)}/{len(detected_stars)} 匹配)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.invert_yaxis()  # 图像坐标系Y轴向下

        # 右图：显示匹配的星在天球上的位置
        ax2 = axes[1]

        if matches:
            # 提取匹配的星表星的赤经赤纬
            ra_matched = []
            dec_matched = []

            for cat_idx in matches.values():
                star = self.catalog[cat_idx]
                ra_matched.append(star.ra_deg)
                dec_matched.append(star.dec_deg)

            # 绘制全天球星表（背景）
            ra_all = [star.ra_deg for star in self.catalog[:500]]  # 只显示前500颗作为背景
            dec_all = [star.dec_deg for star in self.catalog[:500]]

            ax2.scatter(ra_all, dec_all, c='gray', s=1, alpha=0.3, label='星表背景')
            ax2.scatter(ra_matched, dec_matched, c='red', s=50, label='匹配恒星')

            # 添加标签
            for i, (ra, dec) in enumerate(zip(ra_matched, dec_matched)):
                ax2.text(ra + 1, dec + 1,
                         f'ID:{list(matches.values())[i]}',
                         fontsize=8, color='darkred')

            ax2.set_xlabel('赤经 (度)')
            ax2.set_ylabel('赤纬 (度)')
            ax2.set_title('天球坐标系中的匹配恒星')
            ax2.legend()
            ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        # 保存图像
        output_dir = r"D:\1StarTracker\data\processed"
        os.makedirs(output_dir, exist_ok=True)
        plt.savefig(os.path.join(output_dir, "star_identification_result.png"), dpi=150)
        plt.show()


def test_star_identification():
    """测试星图识别算法"""
    print("=" * 60)
    print("星图识别算法测试")
    print("=" * 60)

    # 1. 创建星图识别器
    print("\n1. 初始化星图识别器...")
    identifier = TriangleStarIdentifier(
        max_angle_error_deg=0.2,
        min_angle_deg=0.5,
        max_angle_deg=15.0
    )

    # 2. 模拟检测到的星点（从星表中随机选择一些星，模拟观测）
    print("\n2. 模拟检测到的星点...")

    # 相机参数
    camera_params = {
        'focal_length_pixels': 1000.0,
        'center_x': 512.0,
        'center_y': 512.0
    }

    # 从星表中随机选择一些星作为"检测到的"星
    n_detected = 8
    catalog_size = len(identifier.catalog)

    # 随机选择星表索引
    np.random.seed(42)
    selected_indices = np.random.choice(catalog_size, n_detected, replace=False)

    # 模拟检测星点（添加噪声）
    detected_stars = []
    noise_level_pixels = 0.5  # 像素噪声

    for idx in selected_indices:
        star = identifier.catalog[idx]

        # 将天球向量转换为像素坐标（假设某个姿态）
        # 为了简化，我们假设一个简单的投影
        vector = star.vector

        # 简单的针孔投影（假设相机指向[0,0,1]）
        if vector[2] > 0:  # 在视场内
            # 计算像素坐标（理想情况）
            x_ideal = vector[0] / vector[2] * camera_params['focal_length_pixels'] + camera_params['center_x']
            y_ideal = vector[1] / vector[2] * camera_params['focal_length_pixels'] + camera_params['center_y']

            # 添加噪声
            x_noisy = x_ideal + np.random.normal(0, noise_level_pixels)
            y_noisy = y_ideal + np.random.normal(0, noise_level_pixels)

            # 估计星等（添加噪声）
            mag_noisy = star.magnitude + np.random.normal(0, 0.2)

            detected_stars.append(DetectedStar(
                pixel_x=x_noisy,
                pixel_y=y_noisy,
                magnitude_est=mag_noisy,
                flux=1000 * 10 ** (-0.4 * mag_noisy)  # 简单流量模型
            ))

    print(f"模拟了 {len(detected_stars)} 颗检测星点")

    # 3. 运行星图识别
    print("\n3. 运行星图识别...")
    matches = identifier.identify_stars(
        detected_stars=detected_stars,
        camera_params=camera_params,
        min_matches=3,
        debug=True
    )

    # 4. 分析结果
    print("\n4. 识别结果分析:")
    if matches:
        print(f"成功匹配 {len(matches)} 颗星:")

        correct_matches = 0
        for det_idx, cat_idx in matches.items():
            true_cat_idx = selected_indices[det_idx]
            is_correct = (cat_idx == true_cat_idx)

            if is_correct:
                correct_matches += 1
                status = "✓"
            else:
                status = "✗"

            print(f"  检测星{det_idx} -> 星表ID{cat_idx} {status}")

        accuracy = correct_matches / len(matches) * 100
        print(f"\n匹配准确率: {accuracy:.1f}% ({correct_matches}/{len(matches)})")

        # 5. 可视化
        print("\n5. 生成可视化图表...")
        identifier.visualize_identification(detected_stars, matches, camera_params)

    else:
        print("识别失败: 没有匹配到足够的星点")

    print("\n星图识别算法测试完成!")
    print("=" * 60)

    return identifier, matches, detected_stars


def create_integration_demo():
    """创建与QUEST算法集成的演示"""
    print("\n" + "=" * 60)
    print("星图识别与QUEST姿态解算集成演示")
    print("=" * 60)

    try:
        # 导入QUEST算法
        from quest import QUEST

        # 1. 创建星图识别器
        identifier = TriangleStarIdentifier(
            max_angle_error_deg=0.2,
            min_angle_deg=0.5,
            max_angle_deg=15.0
        )

        # 2. 模拟一个观测场景
        camera_params = {
            'focal_length_pixels': 1000.0,
            'center_x': 512.0,
            'center_y': 512.0
        }

        # 生成随机姿态
        quest = QUEST(use_davenport_q=True)
        true_roll, true_pitch, true_yaw = 10, 20, 30
        true_quaternion = quest.euler_to_quaternion(true_roll, true_pitch, true_yaw)
        true_R = quest.quaternion_to_rotation_matrix(true_quaternion)

        # 从星表中随机选择星点
        n_stars = 6
        catalog_size = len(identifier.catalog)
        selected_indices = np.random.choice(catalog_size, n_stars, replace=False)

        # 模拟检测星点
        detected_stars = []
        detected_vectors = []
        true_vectors = []

        noise_level_pixels = 0.3

        for idx in selected_indices:
            star = identifier.catalog[idx]

            # 天球向量
            inertial_vector = star.vector

            # 应用真实旋转得到观测向量
            body_vector_true = np.dot(true_R, inertial_vector)

            # 将观测向量投影到像素坐标
            if body_vector_true[2] > 0:
                x_ideal = body_vector_true[0] / body_vector_true[2] * camera_params['focal_length_pixels'] + \
                          camera_params['center_x']
                y_ideal = body_vector_true[1] / body_vector_true[2] * camera_params['focal_length_pixels'] + \
                          camera_params['center_y']

                # 添加噪声
                x_noisy = x_ideal + np.random.normal(0, noise_level_pixels)
                y_noisy = y_ideal + np.random.normal(0, noise_level_pixels)

                detected_stars.append(DetectedStar(
                    pixel_x=x_noisy,
                    pixel_y=y_noisy,
                    magnitude_est=star.magnitude,
                    flux=1000 * 10 ** (-0.4 * star.magnitude)
                ))

                # 将噪声像素坐标转换回向量（用于QUEST）
                vector_noisy = identifier.pixel_to_vector(
                    x_noisy, y_noisy,
                    camera_params['focal_length_pixels'],
                    camera_params['center_x'], camera_params['center_y']
                )

                detected_vectors.append(vector_noisy)
                true_vectors.append(inertial_vector)

        print(f"模拟观测到 {len(detected_stars)} 颗星")

        # 3. 星图识别
        print("\n运行星图识别...")
        matches = identifier.identify_stars(
            detected_stars=detected_stars,
            camera_params=camera_params,
            min_matches=3,
            debug=True
        )

        if not matches:
            print("星图识别失败，无法继续")
            return

        print(f"成功匹配 {len(matches)} 颗星")

        # 4. 准备QUEST输入
        body_vectors_quest = []
        inertial_vectors_quest = []

        for det_idx, cat_idx in matches.items():
            body_vectors_quest.append(detected_vectors[det_idx])
            inertial_vectors_quest.append(identifier.catalog[cat_idx].vector)

        body_vectors_quest = np.array(body_vectors_quest)
        inertial_vectors_quest = np.array(inertial_vectors_quest)

        # 5. 运行QUEST姿态解算
        print("\n运行QUEST姿态解算...")
        result = quest.solve(body_vectors_quest, inertial_vectors_quest)

        print(f"真实姿态: roll={true_roll:.2f}°, pitch={true_pitch:.2f}°, yaw={true_yaw:.2f}°")
        print(
            f"解算姿态: roll={result.euler_angles[0]:.2f}°, pitch={result.euler_angles[1]:.2f}°, yaw={result.euler_angles[2]:.2f}°")

        # 计算姿态误差
        error_roll = abs(result.euler_angles[0] - true_roll)
        error_pitch = abs(result.euler_angles[1] - true_pitch)
        error_yaw = abs(result.euler_angles[2] - true_yaw)

        print(f"姿态误差: roll={error_roll:.4f}°, pitch={error_pitch:.4f}°, yaw={error_yaw:.4f}°")
        print(f"误差估计: {result.error_estimate:.6f}")

        # 6. 可视化
        print("\n生成集成演示图表...")

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))

        # 左图：检测与匹配
        ax1 = axes[0]
        pixel_x = [star.pixel_x for star in detected_stars]
        pixel_y = [star.pixel_y for star in detected_stars]

        ax1.scatter(pixel_x, pixel_y, c='blue', s=50, alpha=0.7, label='检测星点')

        matched_x = [detected_stars[i].pixel_x for i in matches.keys()]
        matched_y = [detected_stars[i].pixel_y for i in matches.keys()]

        if matched_x:
            ax1.scatter(matched_x, matched_y, c='red', s=100,
                        edgecolors='yellow', linewidth=2, label='匹配星点')

            for det_idx, cat_idx in matches.items():
                ax1.text(detected_stars[det_idx].pixel_x + 10,
                         detected_stars[det_idx].pixel_y + 10,
                         f'ID:{cat_idx}', fontsize=9, color='red')

        ax1.set_xlabel('像素X')
        ax1.set_ylabel('像素Y')
        ax1.set_title('星点检测与匹配')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.invert_yaxis()

        # 中图：姿态对比
        ax2 = axes[1]
        angles = ['Roll', 'Pitch', 'Yaw']
        true_values = [true_roll, true_pitch, true_yaw]
        estimated_values = [result.euler_angles[0], result.euler_angles[1], result.euler_angles[2]]

        x = np.arange(len(angles))
        width = 0.35

        ax2.bar(x - width / 2, true_values, width, label='真实姿态', color='blue', alpha=0.7)
        ax2.bar(x + width / 2, estimated_values, width, label='解算姿态', color='red', alpha=0.7)

        ax2.set_xlabel('姿态轴')
        ax2.set_ylabel('角度 (°)')
        ax2.set_title('姿态解算结果对比')
        ax2.set_xticks(x)
        ax2.set_xticklabels(angles)
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis='y')

        # 右图：误差分析
        ax3 = axes[2]
        errors = [error_roll, error_pitch, error_yaw]
        error_arcsec = [e * 3600 for e in errors]  # 转换为角秒

        bars = ax3.bar(angles, error_arcsec, color=['red', 'green', 'blue'], alpha=0.7)

        # 添加误差值标签
        for bar, error in zip(bars, error_arcsec):
            height = bar.get_height()
            ax3.text(bar.get_x() + bar.get_width() / 2., height + 10,
                     f'{error:.1f}"', ha='center', va='bottom', fontsize=10)

        ax3.set_xlabel('姿态轴')
        ax3.set_ylabel('误差 (角秒)')
        ax3.set_title('姿态解算误差分析')
        ax3.grid(True, alpha=0.3, axis='y')

        # 添加目标线（例如10角秒）
        ax3.axhline(y=10, color='r', linestyle='--', alpha=0.5, label='10角秒目标')
        ax3.legend()

        plt.tight_layout()

        # 保存图像
        output_dir = r"D:\1StarTracker\data\processed"
        os.makedirs(output_dir, exist_ok=True)
        plt.savefig(os.path.join(output_dir, "integrated_pipeline_demo.png"), dpi=150)
        plt.show()

        print("\n集成演示完成!")

    except ImportError as e:
        print(f"导入QUEST模块失败: {e}")
        print("请确保quest.py在同一目录下")


if __name__ == "__main__":
    # 运行测试
    identifier, matches, detected_stars = test_star_identification()

    # 运行集成演示
    create_integration_demo()