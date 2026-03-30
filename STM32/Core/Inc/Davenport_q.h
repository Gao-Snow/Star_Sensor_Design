/**
 * @file Davenport.h
 * @brief 姿态解算模块（Davenport q 方法）
 * 
 * 该模块实现从多对向量观测中估计最优姿态四元数。
 * 核心算法：构建姿态剖面矩阵 B -> 构建 Davenport 矩阵 K ->
 * 对 K 进行特征分解 -> 取最大特征值对应的特征向量作为最优四元数。
 * 
 */

#ifndef DAVENPORT_H
#define DAVENPORT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= 常量定义 ========================= */
#define DAVENPORT_MAX_STARS 50          ///< 最大支持星点数（用于静态数组）
#define DAVENPORT_ITER_MAX 100          ///< Jacobi 迭代最大次数
#define DAVENPORT_EPSILON 1e-6f         ///< 收敛阈值

/* ========================= 错误码 ========================= */
typedef enum {
    DAVENPORT_SUCCESS = 0,              ///< 成功
    DAVENPORT_ERROR_INVALID_PARAM = -1, ///< 无效参数（空指针、星数不足）
    DAVENPORT_ERROR_NOT_ENOUGH_STARS = -2, ///< 星数不足（至少需要 2 个不共线向量）
    DAVENPORT_ERROR_SINGULAR = -3,      ///< 矩阵奇异，无法解算
    DAVENPORT_ERROR_NO_CONVERGENCE = -4 ///< 特征分解未收敛
} DavenportStatus;

/* ========================= 数据结构 ========================= */
/**
 * @brief 三维向量（单精度）
 */
typedef struct {
    float x, y, z;
} DavenportVector3;

/**
 * @brief 四元数 [w, x, y, z] (w 为标量)
 */
typedef struct {
    float w, x, y, z;
} DavenportQuaternion;

/**
 * @brief 姿态解算配置
 */
typedef struct {
    int max_iterations;      ///< 最大迭代次数（建议 100）
    float convergence_thresh; ///< 收敛阈值（建议 1e-6f）
} DavenportConfig;

/**
 * @brief 姿态解算结果
 */
typedef struct {
    DavenportQuaternion quat;    ///< 最优四元数
    float error_estimate;    ///< 加权均方根误差
    int num_stars_used;      ///< 实际使用的星数
} DavenportResult;

/* ========================= 核心 API ========================= */

/**
 * @brief 初始化默认配置
 * @param config 输出配置结构体
 */
void davenport_config_default(DavenportConfig* config);

/**
 * @brief 使用 Davenport q 方法求解最优姿态
 * 
 * @param config       [in]  配置参数
 * @param body_vectors [in]  本体系观测向量数组（需已归一化）
 * @param ref_vectors  [in]  参考系向量数组（需已归一化）
 * @param weights      [in]  权重数组（可为 NULL，则等权重）
 * @param num_vectors  [in]  向量对数量（必须 >= 2 且 <= QUEST_MAX_STARS）
 * @param result       [out] 解算结果
 * @return DavenportStatus 状态码
 */
DavenportStatus davenport_solve(
    const DavenportConfig* config,
    const DavenportVector3 body_vectors[],
    const DavenportVector3 ref_vectors[],
    const float weights[],
    int num_vectors,
    DavenportResult* result
);

/**
 * @brief 将四元数转换为旋转矩阵（3x3，行主序）
 * @param quat 输入四元数
 * @param mat  输出 3x3 矩阵（数组长度为 9）
 */
void davenport_quaternion_to_matrix(const DavenportQuaternion* quat, float mat[9]);

/**
 * @brief 将四元数转换为欧拉角（ZYX 顺序，单位：度）
 * @param quat   输入四元数
 * @param roll   输出滚转角
 * @param pitch  输出俯仰角
 * @param yaw    输出偏航角
 */
void davenport_quaternion_to_euler(const DavenportQuaternion* quat,
                               float* roll, float* pitch, float* yaw);

#ifdef __cplusplus
}
#endif

#endif // DAVENPORT_H
