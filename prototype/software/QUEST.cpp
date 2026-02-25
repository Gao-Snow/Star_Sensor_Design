/**
 * @file attitude_estimator.c
 * @brief 基于Davenport q方法的姿态估计器（Wahba问题求解）
 * 
 * 该程序实现了从多对向量观测中估计最优旋转矩阵（四元数）的功能。
 * 核心算法：构造姿态剖面矩阵B -> 构造Davenport矩阵K -> 对K进行特征分解 ->
 * 取最大特征值对应的特征向量作为最优四元数 -> 转换为旋转矩阵和欧拉角。
 * 
 * 注意：函数名虽为quest_solver，但实际使用的是精确的4x4特征分解（Davenport q方法），
 *       而非近似迭代的QUEST算法。
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

/* ========================= 常量定义 ========================= */
#define MAX_STARS 10            // 最大支持的星点数
#define MAX_JACOBI_ITER 100     // Jacobi迭代最大次数
#define EPSILON 1e-6            // 收敛阈值（用于归一化、Jacobi迭代）
#define SINGULARITY_THRESHOLD 0.999999  // 欧拉角万向锁判断阈值
#define VALID_ERROR_THRESHOLD 1.0       // 有效姿态的误差阈值（弧度），实际应用中需调整
#define PI 3.14159265358979323846

/* ========================= 数据结构 ========================= */
// 三维向量
typedef struct {
    double x, y, z;
} Vector3;

// 四元数 [q0, q1, q2, q3] (q0为标量部分)
typedef struct {
    double q0, q1, q2, q3;
} Quaternion;

// 旋转矩阵 (3x3)
typedef struct {
    double m[3][3];
} RotationMatrix;

// 姿态解算结果
typedef struct {
    Quaternion quaternion;          // 最优四元数
    RotationMatrix rotation_matrix; // 对应的旋转矩阵
    double euler_angles[3];         // [roll, pitch, yaw] 单位：度
    double error_estimate;          // 加权均方根误差
    bool valid;                     // 解是否有效
    int num_stars_used;             // 实际使用的星数
} AttitudeResult;

/* ========================= 辅助函数 ========================= */
/**
 * @brief 归一化三维向量（若模长小于EPSILON则保持不变）
 */
void normalize_vector(Vector3* v) {
    double norm = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
    if (norm > EPSILON) {
        v->x /= norm;
        v->y /= norm;
        v->z /= norm;
    }
    // 若norm <= EPSILON，向量无效，但不作处理（由调用者保证输入合理）
}

/**
 * @brief 向量点积
 */
double dot_product(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * @brief 向量叉积
 */
Vector3 cross_product(Vector3 a, Vector3 b) {
    Vector3 result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

/**
 * @brief 3x3矩阵乘法 C = A * B
 */
void matrix_multiply(double A[3][3], double B[3][3], double C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

/**
 * @brief 3x3矩阵的迹（对角线元素之和）
 */
double matrix_trace(double mat[3][3]) {
    return mat[0][0] + mat[1][1] + mat[2][2];
}

/**
 * @brief 3x3矩阵转置
 */
void matrix_transpose(double mat[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = mat[j][i];
        }
    }
}

/* ================== 核心算法：构建B矩阵和K矩阵 ================= */
/**
 * @brief 计算姿态剖面矩阵 B = sum_i weight_i * (b_i * r_i^T)
 * 
 * @param B 输出3x3矩阵
 * @param body_vectors 本体系向量数组
 * @param inertial_vectors 参考系向量数组
 * @param weights 权重数组（若为NULL则使用平均权重 1/n）
 * @param n 向量对数量
 */
void compute_B_matrix(double B[3][3], Vector3 body_vectors[], Vector3 inertial_vectors[],
                      double weights[], int n) {
    // 初始化B为零矩阵
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            B[i][j] = 0.0;
        }
    }

    // 累加每对向量的外积
    for (int idx = 0; idx < n; idx++) {
        double weight = weights ? weights[idx] : 1.0 / n;  // 若权重为空则平均加权
        // 计算外积矩阵 b * r^T
        double outer[3][3] = {
            {body_vectors[idx].x * inertial_vectors[idx].x,
             body_vectors[idx].x * inertial_vectors[idx].y,
             body_vectors[idx].x * inertial_vectors[idx].z},
            {body_vectors[idx].y * inertial_vectors[idx].x,
             body_vectors[idx].y * inertial_vectors[idx].y,
             body_vectors[idx].y * inertial_vectors[idx].z},
            {body_vectors[idx].z * inertial_vectors[idx].x,
             body_vectors[idx].z * inertial_vectors[idx].y,
             body_vectors[idx].z * inertial_vectors[idx].z}
        };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                B[i][j] += weight * outer[i][j];
            }
        }
    }
}

/**
 * @brief 构造Davenport K矩阵 (4x4)
 * 
 * K = [ S - sigma*I ,  z ;
 *          z^T     , sigma ]
 * 其中 S = B + B^T, sigma = trace(B), z = [B(1,2)-B(2,1); B(2,0)-B(0,2); B(0,1)-B(1,0)]
 */
void construct_K_matrix(double K[4][4], double B[3][3]) {
    // 计算 S = B + B^T
    double S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = B[i][j] + B[j][i];
        }
    }

    // 计算 sigma = trace(B)
    double sigma = matrix_trace(B);

    // 计算 Z 向量
    double Z[3] = {
        B[1][2] - B[2][1],
        B[2][0] - B[0][2],
        B[0][1] - B[1][0]
    };

    // 填充K矩阵
    // 左上角 3x3: S - sigma * I
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = S[i][j];
            if (i == j) {
                K[i][j] -= sigma;
            }
        }
    }
    // 右上角 3x1: Z
    for (int i = 0; i < 3; i++) {
        K[i][3] = Z[i];
    }
    // 左下角 1x3: Z^T
    for (int j = 0; j < 3; j++) {
        K[3][j] = Z[j];
    }
    // 右下角 1x1: sigma
    K[3][3] = sigma;
}

/* ================== 特征分解（Jacobi旋转法） ================= */
/**
 * @brief 对4x4对称矩阵进行Jacobi特征分解，得到特征值和特征向量
 * 
 * @param A 输入对称矩阵（会被覆盖）
 * @param eigenvalues 输出特征值数组（长度为4）
 * @param eigenvectors 输出特征向量矩阵（列优先，第j列为第j个特征值对应的特征向量）
 * 
 * @note 使用经典Jacobi方法，迭代直到所有非对角元绝对值小于EPSILON或达到最大迭代次数
 */
void jacobi_eigen4x4(double A[4][4], double eigenvalues[4], double eigenvectors[4][4]) {
    // 初始化特征向量为单位矩阵
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            eigenvectors[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 复制A，因为我们会不断修改它
    double work[4][4];
    memcpy(work, A, sizeof(double) * 16);

    for (int iter = 0; iter < MAX_JACOBI_ITER; iter++) {
        // 寻找当前最大的非对角元素（上三角部分）
        double max_val = 0.0;
        int p = 0, q = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                double abs_val = fabs(work[i][j]);
                if (abs_val > max_val) {
                    max_val = abs_val;
                    p = i;
                    q = j;
                }
            }
        }

        // 若最大非对角元已小于阈值，收敛
        if (max_val < EPSILON) {
            break;
        }

        // 计算Jacobi旋转角，使 work[p][q] 变为0
        double theta = 0.5 * atan2(2.0 * work[p][q], work[q][q] - work[p][p]);
        double c = cos(theta);
        double s = sin(theta);

        // 更新 work 矩阵中对角线元素和 p,q 行/列
        double App = work[p][p];
        double Aqq = work[q][q];
        double Apq = work[p][q];

        work[p][p] = c * c * App + s * s * Aqq - 2.0 * s * c * Apq;
        work[q][q] = s * s * App + c * c * Aqq + 2.0 * s * c * Apq;
        work[p][q] = work[q][p] = 0.0;  // 精确置零

        // 更新其他元素 (r != p, r != q)
        for (int r = 0; r < 4; r++) {
            if (r != p && r != q) {
                double Apr = work[p][r];
                double Aqr = work[q][r];
                work[p][r] = c * Apr - s * Aqr;
                work[r][p] = work[p][r];
                work[q][r] = s * Apr + c * Aqr;
                work[r][q] = work[q][r];
            }
        }

        // 更新特征向量矩阵 V = V * R
        for (int r = 0; r < 4; r++) {
            double vrp = eigenvectors[r][p];
            double vrq = eigenvectors[r][q];
            eigenvectors[r][p] = c * vrp - s * vrq;
            eigenvectors[r][q] = s * vrp + c * vrq;
        }
    }

    // 提取特征值（对角线元素）
    for (int i = 0; i < 4; i++) {
        eigenvalues[i] = work[i][i];
    }
}

/* ================== 姿态表示转换 ================= */
/**
 * @brief 四元数转旋转矩阵
 * 
 * 公式采用标量部分为q0，向量部分为(q1,q2,q3)的约定。
 */
RotationMatrix quaternion_to_rotation_matrix(Quaternion q) {
    RotationMatrix R;
    double q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;

    R.m[0][0] = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    R.m[0][1] = 2.0 * (q1 * q2 - q0 * q3);
    R.m[0][2] = 2.0 * (q1 * q3 + q0 * q2);

    R.m[1][0] = 2.0 * (q1 * q2 + q0 * q3);
    R.m[1][1] = 1.0 - 2.0 * (q1 * q1 + q3 * q3);
    R.m[1][2] = 2.0 * (q2 * q3 - q0 * q1);

    R.m[2][0] = 2.0 * (q1 * q3 - q0 * q2);
    R.m[2][1] = 2.0 * (q2 * q3 + q0 * q1);
    R.m[2][2] = 1.0 - 2.0 * (q1 * q1 + q2 * q2);

    return R;
}

/**
 * @brief 旋转矩阵转欧拉角 (ZYX顺序，即yaw-pitch-roll)
 * 
 * @param R 旋转矩阵
 * @param euler 输出数组 [roll, pitch, yaw] 单位：度
 */
void rotation_matrix_to_euler(RotationMatrix R, double euler[3]) {
    double roll, pitch, yaw;

    // 检查万向锁：当 |R[2][0]| 接近1时，俯仰角为 ±90度
    if (fabs(R.m[2][0]) > SINGULARITY_THRESHOLD) {
        // 万向锁情况：约定 yaw = 0
        yaw = 0.0;
        if (R.m[2][0] < 0.0) {
            pitch = 90.0;
            roll = atan2(R.m[0][1], R.m[0][2]) * 180.0 / PI;
        } else {
            pitch = -90.0;
            roll = atan2(-R.m[0][1], -R.m[0][2]) * 180.0 / PI;
        }
    } else {
        pitch = -asin(R.m[2][0]) * 180.0 / PI;
        double cos_pitch = cos(pitch * PI / 180.0);
        roll = atan2(R.m[2][1] / cos_pitch, R.m[2][2] / cos_pitch) * 180.0 / PI;
        yaw  = atan2(R.m[1][0] / cos_pitch, R.m[0][0] / cos_pitch) * 180.0 / PI;
    }

    euler[0] = roll;
    euler[1] = pitch;
    euler[2] = yaw;
}

/* ================== 误差计算 ================= */
/**
 * @brief 计算加权均方根误差
 * 
 * 误差定义： sqrt( sum_i weight_i * || b_i - A * r_i ||^2 / n )
 */
double calculate_attitude_error(Vector3 body_vectors[], Vector3 inertial_vectors[],
                                RotationMatrix R, double weights[], int n) {
    double error = 0.0;

    for (int i = 0; i < n; i++) {
        double weight = weights ? weights[i] : 1.0 / n;

        // 预测本体系向量：A * r_i
        Vector3 predicted;
        predicted.x = R.m[0][0] * inertial_vectors[i].x +
                      R.m[0][1] * inertial_vectors[i].y +
                      R.m[0][2] * inertial_vectors[i].z;
        predicted.y = R.m[1][0] * inertial_vectors[i].x +
                      R.m[1][1] * inertial_vectors[i].y +
                      R.m[1][2] * inertial_vectors[i].z;
        predicted.z = R.m[2][0] * inertial_vectors[i].x +
                      R.m[2][1] * inertial_vectors[i].y +
                      R.m[2][2] * inertial_vectors[i].z;

        // 差值向量
        Vector3 diff = {
            body_vectors[i].x - predicted.x,
            body_vectors[i].y - predicted.y,
            body_vectors[i].z - predicted.z
        };

        error += weight * (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    }

    return sqrt(error / n);  // 均方根
}

/* ================== 主求解函数 ================= */
/**
 * @brief Davenport q方法求解最优姿态
 * 
 * @param body_vectors 本体系观测向量数组（会被归一化）
 * @param inertial_vectors 参考系向量数组（会被归一化）
 * @param weights 权重数组（可为NULL，此时等权重）
 * @param n 向量对数量，必须在[2, MAX_STARS]之间
 * @return AttitudeResult 包含姿态估计结果
 */
AttitudeResult quest_solver(Vector3 body_vectors[], Vector3 inertial_vectors[],
                            double weights[], int n) {
    AttitudeResult result;
    memset(&result, 0, sizeof(result));

    // 输入有效性检查
    if (n < 2 || n > MAX_STARS) {
        result.valid = false;
        result.error_estimate = -1.0;
        return result;
    }

    // 创建本地副本，避免修改外部数组（归一化操作）
    Vector3 local_body[MAX_STARS];
    Vector3 local_inertial[MAX_STARS];
    for (int i = 0; i < n; i++) {
        local_body[i] = body_vectors[i];
        local_inertial[i] = inertial_vectors[i];
        normalize_vector(&local_body[i]);
        normalize_vector(&local_inertial[i]);
    }

    // 计算姿态剖面矩阵 B
    double B[3][3];
    compute_B_matrix(B, local_body, local_inertial, weights, n);

    // 构造 Davenport K 矩阵
    double K[4][4];
    construct_K_matrix(K, B);

    // 特征分解 K
    double eigenvalues[4];
    double eigenvectors[4][4];
    jacobi_eigen4x4(K, eigenvalues, eigenvectors);

    // 找到最大特征值的索引
    int max_idx = 0;
    for (int i = 1; i < 4; i++) {
        if (eigenvalues[i] > eigenvalues[max_idx]) {
            max_idx = i;
        }
    }

    // 提取对应的特征向量作为四元数（注意：eigenvectors的列是特征向量）
    Quaternion q = {
        .q0 = eigenvectors[0][max_idx],
        .q1 = eigenvectors[1][max_idx],
        .q2 = eigenvectors[2][max_idx],
        .q3 = eigenvectors[3][max_idx]
    };

    // 归一化四元数
    double norm = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    if (norm > EPSILON) {
        q.q0 /= norm;
        q.q1 /= norm;
        q.q2 /= norm;
        q.q3 /= norm;
    }

    // 确保标量部分非负（可选规范）
    if (q.q0 < 0) {
        q.q0 = -q.q0;
        q.q1 = -q.q1;
        q.q2 = -q.q2;
        q.q3 = -q.q3;
    }

    // 存储结果
    result.quaternion = q;
    result.rotation_matrix = quaternion_to_rotation_matrix(q);
    rotation_matrix_to_euler(result.rotation_matrix, result.euler_angles);
    result.error_estimate = calculate_attitude_error(local_body, local_inertial,
                                                     result.rotation_matrix, weights, n);
    result.valid = (result.error_estimate < VALID_ERROR_THRESHOLD);
    result.num_stars_used = n;

    return result;
}

/* ================== 打印结果 ================= */
void print_attitude_result(AttitudeResult result) {
    if (!result.valid) {
        printf("姿态解算无效! 错误估计: %.6f\n", result.error_estimate);
        return;
    }

    printf("姿态解算成功!\n");
    printf("使用星数: %d\n", result.num_stars_used);
    printf("四元数: [%.4f, %.4f, %.4f, %.4f]\n",
           result.quaternion.q0, result.quaternion.q1,
           result.quaternion.q2, result.quaternion.q3);
    printf("旋转矩阵:\n");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%.4f\t", result.rotation_matrix.m[i][j]);
        }
        printf("\n");
    }
    printf("欧拉角(度): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n",
           result.euler_angles[0], result.euler_angles[1], result.euler_angles[2]);
    printf("误差估计: %.6f\n", result.error_estimate);
}

/* ================== 测试代码 ================= */
int main() {
    // 固定随机种子，使结果可重复
    srand(42);

    // ===== 测试1：简单两星测试（绕Z轴旋转30度） =====
    printf("===== 测试1: 简单两星测试 =====\n");

    double theta = 30.0 * PI / 180.0;
    RotationMatrix true_R = {
        .m = {
            {cos(theta), -sin(theta), 0},
            {sin(theta),  cos(theta), 0},
            {0, 0, 1}
        }
    };

    Vector3 star1_inertial = {1, 0, 0};
    Vector3 star2_inertial = {0, 1, 0};

    // 计算理想的本体系坐标
    Vector3 star1_body = {
        true_R.m[0][0] * star1_inertial.x + true_R.m[0][1] * star1_inertial.y,
        true_R.m[1][0] * star1_inertial.x + true_R.m[1][1] * star1_inertial.y,
        0
    };
    Vector3 star2_body = {
        true_R.m[0][0] * star2_inertial.x + true_R.m[0][1] * star2_inertial.y,
        true_R.m[1][0] * star2_inertial.x + true_R.m[1][1] * star2_inertial.y,
        0
    };

    // 添加微小噪声
    double noise_level = 0.001;
    star1_body.x += noise_level * (rand() / (double)RAND_MAX - 0.5);
    star1_body.y += noise_level * (rand() / (double)RAND_MAX - 0.5);
    star1_body.z += noise_level * (rand() / (double)RAND_MAX - 0.5);
    star2_body.x += noise_level * (rand() / (double)RAND_MAX - 0.5);
    star2_body.y += noise_level * (rand() / (double)RAND_MAX - 0.5);
    star2_body.z += noise_level * (rand() / (double)RAND_MAX - 0.5);

    Vector3 body_vectors[] = {star1_body, star2_body};
    Vector3 inertial_vectors[] = {star1_inertial, star2_inertial};
    double weights[] = {0.5, 0.5};
    int n = 2;

    AttitudeResult result = quest_solver(body_vectors, inertial_vectors, weights, n);
    print_attitude_result(result);

    // ===== 测试2：多星随机测试 =====
    printf("\n===== 测试2: 多星测试 =====\n");
    int n_stars = 4;
    Vector3 body_multi[MAX_STARS];
    Vector3 inertial_multi[MAX_STARS];
    double weights_multi[MAX_STARS];

    // 生成参考系中的随机单位向量
    for (int i = 0; i < n_stars; i++) {
        Vector3 v = {
            (rand() / (double)RAND_MAX) * 2 - 1,
            (rand() / (double)RAND_MAX) * 2 - 1,
            (rand() / (double)RAND_MAX) * 2 - 1
        };
        normalize_vector(&v);
        inertial_multi[i] = v;
        weights_multi[i] = 0.5 + rand() / (double)RAND_MAX;  // 随机权重
    }

    // 定义一个随机旋转（欧拉角：roll=10°, pitch=20°, yaw=30°）
    double roll = 10.0, pitch = 20.0, yaw = 30.0;
    double cr = cos(roll * PI / 180.0), sr = sin(roll * PI / 180.0);
    double cp = cos(pitch * PI / 180.0), sp = sin(pitch * PI / 180.0);
    double cy = cos(yaw * PI / 180.0), sy = sin(yaw * PI / 180.0);
    RotationMatrix random_R = {
        .m = {
            {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
            {-sp, cp * sr, cp * cr}
        }
    };

    // 计算本体系观测值并添加噪声
    for (int i = 0; i < n_stars; i++) {
        Vector3 ideal = {
            random_R.m[0][0] * inertial_multi[i].x +
            random_R.m[0][1] * inertial_multi[i].y +
            random_R.m[0][2] * inertial_multi[i].z,
            random_R.m[1][0] * inertial_multi[i].x +
            random_R.m[1][1] * inertial_multi[i].y +
            random_R.m[1][2] * inertial_multi[i].z,
            random_R.m[2][0] * inertial_multi[i].x +
            random_R.m[2][1] * inertial_multi[i].y +
            random_R.m[2][2] * inertial_multi[i].z
        };
        ideal.x += noise_level * (rand() / (double)RAND_MAX - 0.5);
        ideal.y += noise_level * (rand() / (double)RAND_MAX - 0.5);
        ideal.z += noise_level * (rand() / (double)RAND_MAX - 0.5);
        normalize_vector(&ideal);  // 确保单位向量
        body_multi[i] = ideal;
    }

    // 归一化权重（使权重和为1，可选，但非必须）
    double sum_w = 0.0;
    for (int i = 0; i < n_stars; i++) sum_w += weights_multi[i];
    for (int i = 0; i < n_stars; i++) weights_multi[i] /= sum_w;

    AttitudeResult result_multi = quest_solver(body_multi, inertial_multi, weights_multi, n_stars);
    print_attitude_result(result_multi);

    return 0;
}
