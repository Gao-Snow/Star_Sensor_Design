# src/software/pipeline/star_tracker_pipeline.py
"""
星敏感器完整处理流水线
集成：模拟器 → 质心提取 → 星图识别 → 姿态解算
"""


class StarTrackerPipeline:
    """星敏感器完整处理流水线"""

    def __init__(self, config):
        self.config = config
        self.simulator = StarSimulator(config)
        self.centroid_algorithms = CentroidAlgorithms()
        self.star_identifier = TriangleStarIdentifier()
        self.attitude_solver = QUEST()

    def process_frame(self, image):
        """处理一帧图像"""
        # 1. 星点检测
        stars = self.detect_stars(image)

        # 2. 质心提取
        centroids = self.extract_centroids(stars)

        # 3. 星图识别
        matches = self.identify_stars(centroids)

        # 4. 姿态解算
        attitude = self.solve_attitude(matches)

        return attitude

    def run_simulation_test(self):
        """运行完整仿真测试"""
        # 生成模拟星图
        image = self.simulator.generate_star_image()

        # 处理图像
        attitude = self.process_frame(image)

        # 与真实姿态对比（仿真中已知）
        error = self.calculate_error(attitude)

        return error