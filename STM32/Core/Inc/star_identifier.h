#ifndef STAR_IDENTIFIER_H
#define STAR_IDENTIFIER_H

#include <stdint.h>
#include "star_catalog_nav.h"   // 新星表结构



#ifdef __cplusplus
extern "C" {
#endif

/* 错误码 */
typedef enum {
    IDENTIFIER_SUCCESS = 0,
    IDENTIFIER_ERROR_INVALID_PARAM = -1,
    IDENTIFIER_ERROR_NOT_ENOUGH_STARS = -2,
    IDENTIFIER_ERROR_NO_MATCH = -3,
    IDENTIFIER_ERROR_DB_NOT_LOADED = -4
} IdentifierStatus;

/* 识别器实例（不透明指针） */
typedef struct StarIdentifier StarIdentifier;

/* 创建识别器实例 */
StarIdentifier* star_identifier_create(
    const NavStarEntry* catalog,   // 新星表类型
    int catalog_size,
    const TriangleFeature* db,     // 三角形数据库（已在 star_catalog_nav.h 中定义）
    int db_size
);

/* 销毁识别器 */
void star_identifier_destroy(StarIdentifier* inst);

/* 匹配观测星点 */
IdentifierStatus star_identifier_match(
    const StarIdentifier* inst,
    const float obs_vectors[][3],
    int num_obs,
    uint32_t matches_out[],      // 输出星表索引（0 ~ NAV_STAR_COUNT-1）
    int* match_count_out
);

#ifdef __cplusplus
}
#endif

#endif
