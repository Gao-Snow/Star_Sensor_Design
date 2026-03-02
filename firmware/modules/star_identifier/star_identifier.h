/**
 * @file star_identifier.h
 * @brief 星图识别模块接口（三角形匹配算法）
 * 
 * 本模块使用预计算的三角形数据库进行快速星图识别。
 * 星表和三角形数据库需离线生成，并通过头文件提供。
 */

#ifndef STAR_IDENTIFIER_H
#define STAR_IDENTIFIER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 错误码 */
typedef enum {
    IDENTIFIER_SUCCESS = 0,
    IDENTIFIER_ERROR_INVALID_PARAM = -1,
    IDENTIFIER_ERROR_NOT_ENOUGH_STARS = -2,   ///< 观测星数不足
    IDENTIFIER_ERROR_NO_MATCH = -3,           ///< 无匹配结果
    IDENTIFIER_ERROR_DB_NOT_LOADED = -4       ///< 数据库未加载
} IdentifierStatus;

/**
 * @brief 星表条目结构
 */
typedef struct {
    uint32_t id;          ///< 星表 ID
    float vec[3];         ///< 单位向量 (参考系)
    float magnitude;      ///< 星等（可选，可用于排序）
} StarCatalogEntry;

/**
 * @brief 三角形特征结构（用于数据库查找）
 * 
 * 注意：此结构用于快速查找，角度已量化为整数键。
 */
typedef struct {
    uint32_t star_ids[3];     ///< 三颗星的 ID
    uint64_t feature_key;     ///< 特征键（由三个量化角度组合而成）
} TriangleFeature;

/**
 * @brief 识别器实例（不透明指针）
 */
typedef struct StarIdentifier StarIdentifier;

/**
 * @brief 创建识别器实例（需传入星表和三角形数据库）
 * 
 * @param catalog       [in] 星表数组
 * @param catalog_size  [in] 星表大小
 * @param db            [in] 三角形数据库数组
 * @param db_size       [in] 数据库大小
 * @return StarIdentifier* 实例指针，失败返回 NULL
 */
StarIdentifier* star_identifier_create(
    const StarCatalogEntry* catalog,
    int catalog_size,
    const TriangleFeature* db,
    int db_size
);

/**
 * @brief 销毁识别器实例
 */
void star_identifier_destroy(StarIdentifier* inst);

/**
 * @brief 识别观测星点
 * 
 * @param inst            [in]  识别器实例
 * @param obs_vectors     [in]  观测向量数组（本体系，单位向量）
 * @param num_obs         [in]  观测星数量
 * @param matches_out     [out] 输出匹配的星表 ID 数组（长度至少 num_obs）
 * @param match_count_out [out] 实际匹配数量
 * @return IdentifierStatus
 */
IdentifierStatus star_identifier_match(
    const StarIdentifier* inst,
    const float obs_vectors[][3],
    int num_obs,
    uint32_t matches_out[],
    int* match_count_out
);

#ifdef __cplusplus
}
#endif

#endif // STAR_IDENTIFIER_H
