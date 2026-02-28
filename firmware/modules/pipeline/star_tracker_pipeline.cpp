/**
 * @file star_tracker.c
 * @brief 星敏感器主流水线实现
 */

#include "star_tracker.h"
#include "catalog.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 假设的质心提取和星图识别模块接口（需要实际实现）
// 此处仅为声明，实际使用时需包含对应头文件
StarTrackerStatus centroid_extract(
    const uint16_t* image, int img_w, int img_h,
    int star_x, int star_y,
    float* cx, float* cy
);

StarTrackerStatus star_identify(
    const float centroids[][2],
    int star_count,
    int star_ids[]
);

// 流水线实例结构（隐藏内部细节）
struct StarTrackerInstance {
    StarTrackerConfig config;
    QuestConfig quest_cfg;
    // 可添加其他状态
};

StarTrackerInstance* star_tracker_create(const StarTrackerConfig* config) {
    if (!config || config->max_detected_stars <= 0 ||
        config->image_width <= 0 || config->image_height <= 0) {
        return NULL;
    }

    StarTrackerInstance* inst = (StarTrackerInstance*)malloc(sizeof(StarTrackerInstance));
    if (!inst) return NULL;

    memcpy(&inst->config, config, sizeof(StarTrackerConfig));

    // 初始化 QUEST 配置（可根据需要从 config 读取）
    inst->quest_cfg.max_iterations = 100;
    inst->quest_cfg.convergence_thresh = 1e-6f;

    // 初始化星表（若需要）
    catalog_init();

    return inst;
}

void star_tracker_destroy(StarTrackerInstance* inst) {
    if (inst) free(inst);
}

StarTrackerStatus star_tracker_process_frame(
    StarTrackerInstance* inst,
    const uint16_t* image,
    const int star_pixels[][2],
    int star_count,
    float quat_out[4])
{
    if (!inst || !image || !star_pixels || !quat_out) {
        return STAR_TRACKER_ERROR_INVALID_PARAM;
    }
    if (star_count > inst->config.max_detected_stars) {
        return STAR_TRACKER_ERROR_STAR_COUNT_EXCEEDED;
    }
    if (star_count == 0) {
        return STAR_TRACKER_ERROR_NO_STARS;
    }

    // 临时缓冲区（使用VLA，但确保最大星数不太大）
    float centroids[star_count][2];
    int star_ids[star_count];
    QuestVector3 body_vec[star_count];
    QuestVector3 ref_vec[star_count];

    // 1. 质心提取
    for (int i = 0; i < star_count; ++i) {
        float cx, cy;
        StarTrackerStatus status = centroid_extract(
            image,
            inst->config.image_width, inst->config.image_height,
            star_pixels[i][0], star_pixels[i][1],
            &cx, &cy
        );
        if (status != STAR_TRACKER_SUCCESS) {
            return status;  // 直接传递错误码
        }
        centroids[i][0] = cx;
        centroids[i][1] = cy;
    }

    // 2. 星图识别
    StarTrackerStatus status = star_identify(centroids, star_count, star_ids);
    if (status != STAR_TRACKER_SUCCESS) {
        return status;
    }

    // 3. 将质心坐标转换为本体系单位向量
    for (int i = 0; i < star_count; ++i) {
        float vec[3];
        if (!camera_pixel_to_vector(&inst->config.camera,
                                    centroids[i][0], centroids[i][1],
                                    vec)) {
            return STAR_TRACKER_ERROR_CENTROID_FAILED;  // 转换失败
        }
        body_vec[i].x = vec[0];
        body_vec[i].y = vec[1];
        body_vec[i].z = vec[2];
    }

    // 4. 查询星表获得参考系向量
    for (int i = 0; i < star_count; ++i) {
        float vec[3];
        CatalogStatus cs = catalog_get_vector(star_ids[i], vec);
        if (cs != CATALOG_SUCCESS) {
            return STAR_TRACKER_ERROR_CATALOG_FAILED;
        }
        ref_vec[i].x = vec[0];
        ref_vec[i].y = vec[1];
        ref_vec[i].z = vec[2];
    }

    // 5. 调用 QUEST 解算姿态
    QuestResult qres;
    QuestStatus qs = quest_solve(
        &inst->quest_cfg,
        body_vec, ref_vec,
        NULL,  // 权重为NULL表示等权重
        star_count,
        &qres
    );
    if (qs != QUEST_SUCCESS) {
        return STAR_TRACKER_ERROR_SOLVER_FAILED;
    }

    // 输出四元数
    quat_out[0] = qres.quat.w;
    quat_out[1] = qres.quat.x;
    quat_out[2] = qres.quat.y;
    quat_out[3] = qres.quat.z;

    return STAR_TRACKER_SUCCESS;
}
