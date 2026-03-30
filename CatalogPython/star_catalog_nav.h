#ifndef STAR_CATALOG_NAV_H
#define STAR_CATALOG_NAV_H

#include <stdint.h>

typedef enum { CATALOG_SUCCESS = 0, CATALOG_ERROR_NOT_FOUND = -1, CATALOG_ERROR_INVALID_ID = -2 } CatalogStatus;

CatalogStatus catalog_get_vector_by_index(uint16_t index, float vec_out[3]);

#define NAV_STAR_COUNT 116
#define TRIANGLE_COUNT 1449

typedef struct {
    uint64_t tyc1_2;
    uint8_t  tyc3;
    int32_t  ra_mdeg;
    int32_t  dec_mdeg;
    uint16_t vt_mmag;
    int16_t  x, y, z;
} NavStarEntry;

typedef struct {
    uint64_t feature_key;
    uint16_t star_ids[3];
} TriangleFeature;

extern const NavStarEntry g_nav_star_catalog[NAV_STAR_COUNT];
extern const TriangleFeature g_triangle_db[TRIANGLE_COUNT];

#endif
