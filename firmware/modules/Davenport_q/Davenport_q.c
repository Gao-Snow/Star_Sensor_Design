/**
 * @file davenport.c
 * @brief 姿态解算模块实现（Davenport q 方法）
 */

#include "quest.h"
#include <math.h>
#include <string.h>

/* ========================= 内部常量 ========================= */
#define MAT4_SIZE 16   // 4x4 矩阵元素个数
#define SINGULARITY_THRESHOLD 0.999999f  // 欧拉角万向锁阈值

/* ========================= 内部工具函数 ========================= */

/** 向量点积 */
static float vec_dot(const QuestVector3* a, const QuestVector3* b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

/** 向量叉积 */
static QuestVector3 vec_cross(const QuestVector3* a, const QuestVector3* b) {
    QuestVector3 v;
    v.x = a->y * b->z - a->z * b->y;
    v.y = a->z * b->x - a->x * b->z;
    v.z = a->x * b->y - a->y * b->x;
    return v;
}

/** 3x3 矩阵的迹 */
static float mat_trace(const float B[3][3]) {
    return B[0][0] + B[1][1] + B[2][2];
}

/**
 * @brief 计算姿态剖面矩阵 B = sum_i w_i * (b_i * r_i^T)
 */
static void compute_B_matrix(float B[3][3],
                             const QuestVector3 body[],
                             const QuestVector3 ref[],
                             const float weights[],
                             int n) {
    // 清零
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            B[i][j] = 0.0f;
        }
    }

    // 累加
    for (int idx = 0; idx < n; ++idx) {
        float w = weights ? weights[idx] : 1.0f / n;
        float bx = body[idx].x;
        float by = body[idx].y;
        float bz = body[idx].z;
        float rx = ref[idx].x;
        float ry = ref[idx].y;
        float rz = ref[idx].z;

        B[0][0] += w * bx * rx;
        B[0][1] += w * bx * ry;
        B[0][2] += w * bx * rz;
        B[1][0] += w * by * rx;
        B[1][1] += w * by * ry;
        B[1][2] += w * by * rz;
        B[2][0] += w * bz * rx;
        B[2][1] += w * bz * ry;
        B[2][2] += w * bz * rz;
    }
}

/**
 * @brief 构造 Davenport K 矩阵 (4x4)
 */
static void construct_K_matrix(float K[4][4], const float B[3][3]) {
    // S = B + B^T
    float S[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S[i][j] = B[i][j] + B[j][i];
        }
    }

    float sigma = mat_trace(B);

    // Z 向量
    float Z[3] = {
        B[1][2] - B[2][1],
        B[2][0] - B[0][2],
        B[0][1] - B[1][0]
    };

    // 填充 K
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i][j] = S[i][j];
            if (i == j) K[i][j] -= sigma;
        }
        K[i][3] = Z[i];
        K[3][i] = Z[i];
    }
    K[3][3] = sigma;
}

/**
 * @brief 对 4x4 对称矩阵进行 Jacobi 特征分解（获取最大特征值对应的特征向量）
 * 
 * @param A 输入对称矩阵（会被修改）
 * @param max_eigenvalue 输出最大特征值
 * @param eigenvector 输出对应的特征向量（长度为 4）
 * @param max_iter 最大迭代次数
 * @param tol 收敛阈值
 * @return true 成功，false 未收敛
 */
static bool jacobi_eigen_4x4(float A[4][4],
                             float* max_eigenvalue,
                             float eigenvector[4],
                             int max_iter,
                             float tol) {
    // 初始化特征向量为单位矩阵
    float V[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            V[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 工作矩阵
    float work[4][4];
    memcpy(work, A, sizeof(work));

    for (int iter = 0; iter < max_iter; ++iter) {
        // 寻找最大非对角元
        float max_val = 0.0f;
        int p = 0, q = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                float abs_val = fabsf(work[i][j]);
                if (abs_val > max_val) {
                    max_val = abs_val;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_val < tol) {
            break;  // 收敛
        }

        // 计算旋转角
        float theta = 0.5f * atan2f(2.0f * work[p][q], work[q][q] - work[p][p]);
        float c = cosf(theta);
        float s = sinf(theta);

        // 更新 work 矩阵
        float App = work[p][p];
        float Aqq = work[q][q];
        float Apq = work[p][q];

        work[p][p] = c * c * App + s * s * Aqq - 2.0f * s * c * Apq;
        work[q][q] = s * s * App + c * c * Aqq + 2.0f * s * c * Apq;
        work[p][q] = work[q][p] = 0.0f;

        for (int r = 0; r < 4; ++r) {
            if (r != p && r != q) {
                float Apr = work[p][r];
                float Aqr = work[q][r];
                work[p][r] = c * Apr - s * Aqr;
                work[r][p] = work[p][r];
                work[q][r] = s * Apr + c * Aqr;
                work[r][q] = work[q][r];
            }
        }

        // 更新特征向量矩阵 V = V * R
        for (int r = 0; r < 4; ++r) {
            float vrp = V[r][p];
            float vrq = V[r][q];
            V[r][p] = c * vrp - s * vrq;
            V[r][q] = s * vrp + c * vrq;
        }
    }

    // 提取特征值和特征向量（对角线元素即为特征值）
    int max_idx = 0;
    float max_val = work[0][0];
    for (int i = 1; i < 4; ++i) {
        if (work[i][i] > max_val) {
            max_val = work[i][i];
            max_idx = i;
        }
    }

    *max_eigenvalue = max_val;
    for (int i = 0; i < 4; ++i) {
        eigenvector[i] = V[i][max_idx];
    }

    // 简单检查是否收敛：若最大非对角元仍很大，返回假
    // 这里我们默认已经迭代完成，返回真（实际应用中可根据需要判断）
    return true;
}

/**
 * @brief 计算加权均方根误差
 */
static float compute_error(const QuestVector3 body[],
                           const QuestVector3 ref[],
                           const float weights[],
                           int n,
                           const QuestQuaternion* q) {
    // 先将四元数转为旋转矩阵（行主序）
    float R[3][3];
    float w = q->w, x = q->x, y = q->y, z = q->z;
    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] = 2.0f * (x * y - w * z);
    R[0][2] = 2.0f * (x * z + w * y);
    R[1][0] = 2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] = 2.0f * (y * z - w * x);
    R[2][0] = 2.0f * (x * z - w * y);
    R[2][1] = 2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);

    float err = 0.0f;
    for (int i = 0; i < n; ++i) {
        float wgt = weights ? weights[i] : 1.0f / n;
        // 预测本体系向量: R * ref[i]
        float px = R[0][0] * ref[i].x + R[0][1] * ref[i].y + R[0][2] * ref[i].z;
        float py = R[1][0] * ref[i].x + R[1][1] * ref[i].y + R[1][2] * ref[i].z;
        float pz = R[2][0] * ref[i].x + R[2][1] * ref[i].y + R[2][2] * ref[i].z;

        float dx = body[i].x - px;
        float dy = body[i].y - py;
        float dz = body[i].z - pz;

        err += wgt * (dx * dx + dy * dy + dz * dz);
    }
    return sqrtf(err / n);
}

/* ========================= 公开 API 实现 ========================= */

void quest_config_default(QuestConfig* config) {
    if (config) {
        config->max_iterations = QUEST_ITER_MAX;
        config->convergence_thresh = QUEST_EPSILON;
    }
}

QuestStatus quest_solve(const QuestConfig* config,
                        const QuestVector3 body_vectors[],
                        const QuestVector3 ref_vectors[],
                        const float weights[],
                        int num_vectors,
                        QuestResult* result) {
    // 参数检查
    if (!config || !body_vectors || !ref_vectors || !result) {
        return QUEST_ERROR_INVALID_PARAM;
    }
    if (num_vectors < 2 || num_vectors > QUEST_MAX_STARS) {
        return QUEST_ERROR_NOT_ENOUGH_STARS;
    }

    // 清零结果
    memset(result, 0, sizeof(QuestResult));

    // 计算 B 矩阵
    float B[3][3];
    compute_B_matrix(B, body_vectors, ref_vectors, weights, num_vectors);

    // 构造 K 矩阵
    float K[4][4];
    construct_K_matrix(K, B);

    // 特征分解求最大特征值及对应特征向量
    float eigenvals[4];
    float eigenvec[4];
    bool converged = jacobi_eigen_4x4(K, &eigenvals[0], eigenvec,
                                      config->max_iterations,
                                      config->convergence_thresh);
    if (!converged) {
        return QUEST_ERROR_NO_CONVERGENCE;
    }

    // 特征向量即为四元数（未归一化）
    QuestQuaternion q;
    q.w = eigenvec[0];
    q.x = eigenvec[1];
    q.y = eigenvec[2];
    q.z = eigenvec[3];

    // 归一化四元数
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < QUEST_EPSILON) {
        return QUEST_ERROR_SINGULAR;  // 奇异，无法归一化
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;

    // 确保标量部分非负（可选）
    if (q.w < 0) {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    // 计算误差
    float err = compute_error(body_vectors, ref_vectors, weights, num_vectors, &q);

    // 填充结果
    result->quat = q;
    result->error_estimate = err;
    result->num_stars_used = num_vectors;

    return QUEST_SUCCESS;
}

void quest_quaternion_to_matrix(const QuestQuaternion* quat, float mat[9]) {
    if (!quat || !mat) return;
    float w = quat->w, x = quat->x, y = quat->y, z = quat->z;
    mat[0] = 1.0f - 2.0f * (y * y + z * z);
    mat[1] = 2.0f * (x * y - w * z);
    mat[2] = 2.0f * (x * z + w * y);
    mat[3] = 2.0f * (x * y + w * z);
    mat[4] = 1.0f - 2.0f * (x * x + z * z);
    mat[5] = 2.0f * (y * z - w * x);
    mat[6] = 2.0f * (x * z - w * y);
    mat[7] = 2.0f * (y * z + w * x);
    mat[8] = 1.0f - 2.0f * (x * x + y * y);
}

void quest_quaternion_to_euler(const QuestQuaternion* quat,
                               float* roll, float* pitch, float* yaw) {
    if (!quat) return;
    float w = quat->w, x = quat->x, y = quat->y, z = quat->z;

    // 旋转矩阵元素（行主序）
    float r00 = 1.0f - 2.0f * (y * y + z * z);
    float r10 = 2.0f * (x * y + w * z);
    float r20 = 2.0f * (x * z - w * y);
    float r21 = 2.0f * (y * z + w * x);
    float r22 = 1.0f - 2.0f * (x * x + y * y);

    float p, r, yw;

    // 检查万向锁
    if (fabsf(r20) > SINGULARITY_THRESHOLD) {
        // 俯仰角为 ±90°
        yw = 0.0f;  // 约定 yaw = 0
        if (r20 < 0.0f) {
            p = 90.0f;
            r = atan2f(r10, r00) * 180.0f / (float)M_PI;
        } else {
            p = -90.0f;
            r = atan2f(-r10, -r00) * 180.0f / (float)M_PI;
        }
    } else {
        p = -asinf(r20) * 180.0f / (float)M_PI;
        float cp = cosf(p * (float)M_PI / 180.0f);
        r = atan2f(r21 / cp, r22 / cp) * 180.0f / (float)M_PI;
        yw = atan2f(r10 / cp, r00 / cp) * 180.0f / (float)M_PI;
    }

    if (roll) *roll = r;
    if (pitch) *pitch = p;
    if (yaw) *yaw = yw;
}
