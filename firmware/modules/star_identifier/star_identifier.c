/**
 * @file star_identifier.c
 * @brief 星图识别模块实现（三角形匹配）
 */

#include "star_identifier.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 内部常量 */
#define MAX_OBS_STARS 50          ///< 最大观测星数（与 pipeline 保持一致）
#define MAX_TRIANGLES_PER_OBS 200 ///< 最大观测三角形数（组合数限制）
#define VOTE_THRESHOLD 2           ///< 投票阈值（至少获得几票）

/* 角度量化精度：每个角度乘以 1000 并取整，组合成 64 位键 */
#define ANGLE_QUANT_FACTOR 1000.0f

/**
 * @brief 识别器实例结构（隐藏实现）
 */
struct StarIdentifier {
    const StarCatalogEntry* catalog;
    int catalog_size;
    const TriangleFeature* db;
    int db_size;
};

/* ========== 内部辅助函数 ========== */

/**
 * @brief 计算两个单位向量之间的夹角（弧度）
 */
static float angle_between(const float a[3], const float b[3]) {
    float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    return acosf(dot);
}

/**
 * @brief 生成三角形特征键（从三个角度量化）
 */
static uint64_t make_feature_key(float a1, float a2, float a3) {
    // 将角度排序（升序）
    float angles[3] = {a1, a2, a3};
    if (angles[0] > angles[1]) { float t = angles[0]; angles[0] = angles[1]; angles[1] = t; }
    if (angles[1] > angles[2]) { float t = angles[1]; angles[1] = angles[2]; angles[2] = t; }
    if (angles[0] > angles[1]) { float t = angles[0]; angles[0] = angles[1]; angles[1] = t; }

    // 量化
    uint32_t q0 = (uint32_t)(angles[0] * ANGLE_QUANT_FACTOR);
    uint32_t q1 = (uint32_t)(angles[1] * ANGLE_QUANT_FACTOR);
    uint32_t q2 = (uint32_t)(angles[2] * ANGLE_QUANT_FACTOR);

    // 组合成 64 位键（每部分 21 位，共 63 位）
    return ((uint64_t)q0 << 42) | ((uint64_t)q1 << 21) | q2;
}

/**
 * @brief 在数据库中查找匹配的特征（线性扫描，小型数据库适用）
 * 
 * 注意：若数据库很大，应使用哈希表。这里为了简化，采用线性查找。
 * 嵌入式环境中三角形数据库通常不大（几千条），线性扫描可接受。
 */
static int find_matches_in_db(const TriangleFeature* db, int db_size,
                              uint64_t key, uint32_t out_ids[][3], int max_out) {
    int count = 0;
    for (int i = 0; i < db_size && count < max_out; ++i) {
        if (db[i].feature_key == key) {
            out_ids[count][0] = db[i].star_ids[0];
            out_ids[count][1] = db[i].star_ids[1];
            out_ids[count][2] = db[i].star_ids[2];
            count++;
        }
    }
    return count;
}

/* ========== 公开 API ========== */

StarIdentifier* star_identifier_create(
    const StarCatalogEntry* catalog,
    int catalog_size,
    const TriangleFeature* db,
    int db_size)
{
    if (!catalog || catalog_size <= 0 || !db || db_size <= 0) {
        return NULL;
    }
    StarIdentifier* inst = (StarIdentifier*)malloc(sizeof(StarIdentifier));
    if (!inst) return NULL;

    inst->catalog = catalog;
    inst->catalog_size = catalog_size;
    inst->db = db;
    inst->db_size = db_size;

    return inst;
}

void star_identifier_destroy(StarIdentifier* inst) {
    free(inst);
}

IdentifierStatus star_identifier_match(
    const StarIdentifier* inst,
    const float obs_vectors[][3],
    int num_obs,
    uint32_t matches_out[],
    int* match_count_out)
{
    if (!inst || !obs_vectors || !matches_out || !match_count_out) {
        return IDENTIFIER_ERROR_INVALID_PARAM;
    }
    if (num_obs < 3 || num_obs > MAX_OBS_STARS) {
        return IDENTIFIER_ERROR_NOT_ENOUGH_STARS;
    }

    *match_count_out = 0;

    /* 1. 生成所有观测三角形 */
    int tri_count = 0;
    uint64_t tri_keys[MAX_TRIANGLES_PER_OBS];
    int tri_indices[MAX_TRIANGLES_PER_OBS][3];   // 记录三角形的三个观测星索引

    for (int i = 0; i < num_obs - 2; ++i) {
        for (int j = i + 1; j < num_obs - 1; ++j) {
            for (int k = j + 1; k < num_obs; ++k) {
                if (tri_count >= MAX_TRIANGLES_PER_OBS) break;

                // 计算三个边角
                float a = angle_between(obs_vectors[i], obs_vectors[j]);
                float b = angle_between(obs_vectors[i], obs_vectors[k]);
                float c = angle_between(obs_vectors[j], obs_vectors[k]);

                // 可选：过滤太尖或太钝的三角形（使用角度阈值）
                // 这里简化，不设阈值

                // 计算三个内角
                // 使用球面三角形的余弦定理求内角（小角度时平面近似亦可）
                float cosA = (cosf(a) - cosf(b)*cosf(c)) / (sinf(b)*sinf(c));
                float cosB = (cosf(b) - cosf(a)*cosf(c)) / (sinf(a)*sinf(c));
                float cosC = (cosf(c) - cosf(a)*cosf(b)) / (sinf(a)*sinf(b));

                // 处理数值误差
                if (cosA < -1.0f) cosA = -1.0f;
                if (cosA > 1.0f) cosA = 1.0f;
                if (cosB < -1.0f) cosB = -1.0f;
                if (cosB > 1.0f) cosB = 1.0f;
                if (cosC < -1.0f) cosC = -1.0f;
                if (cosC > 1.0f) cosC = 1.0f;

                float A = acosf(cosA);
                float B = acosf(cosB);
                float C = acosf(cosC);

                // 生成特征键
                uint64_t key = make_feature_key(A, B, C);

                tri_keys[tri_count] = key;
                tri_indices[tri_count][0] = i;
                tri_indices[tri_count][1] = j;
                tri_indices[tri_count][2] = k;
                tri_count++;
            }
        }
    }

    if (tri_count == 0) {
        return IDENTIFIER_ERROR_NO_MATCH;
    }

    /* 2. 投票矩阵（观测星索引 -> 星表ID 的投票计数） */
    // 观测星数量 <= MAX_OBS_STARS，星表ID可能很多，但匹配的候选有限。
    // 这里使用动态映射：对每个观测星，记录一个候选ID及其票数。
    // 简单起见，我们使用固定大小的数组记录每个观测星对应的候选，但星表ID可能很大。
    // 更实用的方法是使用哈希表，但为了简化，我们采用临时数组+线性查找候选。
    // 这里实现一种简化：对每个观测三角形，如果找到匹配，就为三颗星对应的星表ID投票。
    // 最后对每个观测星，选取得票最高的ID。

    // 定义候选结构
    typedef struct {
        uint32_t id;
        int votes;
    } Candidate;

    // 为每个观测星维护一个候选列表（最多假设10个候选）
    #define MAX_CANDIDATES 10
    Candidate candidates[MAX_OBS_STARS][MAX_CANDIDATES];
    int cand_count[MAX_OBS_STARS] = {0};

    // 初始化候选列表为空
    for (int i = 0; i < num_obs; ++i) {
        cand_count[i] = 0;
    }

    // 遍历所有观测三角形，查找匹配
    uint32_t matched_ids[3];
    for (int t = 0; t < tri_count; ++t) {
        // 在数据库中查找
        uint32_t matches[100][3];  // 假设最多100个匹配
        int num_matches = find_matches_in_db(inst->db, inst->db_size,
                                             tri_keys[t], matches, 100);

        for (int m = 0; m < num_matches; ++m) {
            // 对于这个匹配的星表三角形 (ids[0], ids[1], ids[2])
            // 假设观测三角形的顺序与星表三角形顺序对应（需要处理排序问题）
            // 实际中，由于我们使用的是排序后的角度，顺序已经固定，
            // 因此可以直接对应：观测星 i 对应星表ID matches[m][0]，等等。
            // 但需要注意，三个角排序后，边的对应关系不一定保持，但通常假设最可能的对应是直接对应。
            // 这里简化处理：直接按顺序对应。
            for (int k = 0; k < 3; ++k) {
                int obs_idx = tri_indices[t][k];
                uint32_t cat_id = matches[m][k];

                // 在候选列表中查找是否已存在该 cat_id
                int found = -1;
                for (int c = 0; c < cand_count[obs_idx]; ++c) {
                    if (candidates[obs_idx][c].id == cat_id) {
                        found = c;
                        break;
                    }
                }
                if (found >= 0) {
                    candidates[obs_idx][found].votes++;
                } else {
                    // 添加新候选
                    if (cand_count[obs_idx] < MAX_CANDIDATES) {
                        candidates[obs_idx][cand_count[obs_idx]].id = cat_id;
                        candidates[obs_idx][cand_count[obs_idx]].votes = 1;
                        cand_count[obs_idx]++;
                    }
                }
            }
        }
    }

    /* 3. 选择最佳匹配 */
    int matched = 0;
    for (int i = 0; i < num_obs; ++i) {
        if (cand_count[i] == 0) {
            matches_out[i] = 0; // 0 表示未匹配
            continue;
        }
        // 找票数最高的候选
        int best_idx = 0;
        int best_votes = candidates[i][0].votes;
        for (int c = 1; c < cand_count[i]; ++c) {
            if (candidates[i][c].votes > best_votes) {
                best_votes = candidates[i][c].votes;
                best_idx = c;
            }
        }
        if (best_votes >= VOTE_THRESHOLD) {
            matches_out[i] = candidates[i][best_idx].id;
            matched++;
        } else {
            matches_out[i] = 0;
        }
    }

    *match_count_out = matched;
    return (matched >= 3) ? IDENTIFIER_SUCCESS : IDENTIFIER_ERROR_NO_MATCH;
}
