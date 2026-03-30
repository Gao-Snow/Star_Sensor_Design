#include "stm32h7xx_hal.h"
#include "Star_Tracker_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <star_identifier.h>

struct StarTrackerInstance {
    StarTrackerConfig config;
    CentroidConfig centroid_cfg;
    DavenportConfig davenport_cfg;
    SystemParams params;
    StarIdentifier* identifier;
    void (*cmd_cb)(uint8_t cmd, uint8_t* data, uint16_t len);
    int last_star_count;
};

StarTrackerInstance* star_tracker_create(const StarTrackerConfig* config) {
    if (config == NULL || config->max_detected_stars <= 0) {
        return NULL;
    }


    StarTrackerInstance* inst = (StarTrackerInstance*)malloc(sizeof(StarTrackerInstance));
    if (inst == NULL) return NULL;

    memcpy(&inst->config, config, sizeof(StarTrackerConfig));
    centroid_config_default(&inst->centroid_cfg);
    inst->davenport_cfg.max_iterations = 100;
    inst->davenport_cfg.convergence_thresh = 1e-6f;

    // 手动设置默认参数
    inst->params.magic = 0x5A5A5A5A;
    inst->params.exposure_us = 20000;
    inst->params.gain = 16;
    inst->params.output_rate_hz = 5;
    inst->params.crc = 0;

    // 创建星图识别器
    inst->identifier = star_identifier_create(
        g_nav_star_catalog,
        NAV_STAR_COUNT,
        g_triangle_db,
        TRIANGLE_COUNT
    );
    if (inst->identifier == NULL) {
        // 可选打印调试信息，但这里无法打印，可先忽略
    }

    return inst;
}

void star_tracker_destroy(StarTrackerInstance* inst) {
    if (inst) {
        free(inst);
    }
}

StarTrackerStatus star_tracker_process_frame(
    StarTrackerInstance* inst,
    const uint16_t* image,
    int image_width,
    int image_height,
    float quat_out[4],
	int* out_detected,   // 【新增】输出检测到的星数
	int* out_matched)   // 【新增】输出匹配成功的星数)
{
	*out_detected = 0;  // 默认0
    *out_matched = 0;   // 默认0
    // 1. 星点检测 (找出图里的星星在哪里)
    static int star_pixels[STAR_TRACKER_MAX_STARS][2];
    int star_count = 0;
    StarDetectorConfig det_cfg;
    star_detector_config_default(&det_cfg);

    // 如果没检测到星星，或者星星少于3颗，直接返回失败
    if (!star_detector_detect(&det_cfg, image, image_width, image_height, star_pixels, STAR_TRACKER_MAX_STARS, &star_count)) {
        return STAR_TRACKER_ERROR_SOLVER_FAILED;
    }
    if (star_count < 3) {
        return STAR_TRACKER_ERROR_SOLVER_FAILED;
    }
    *out_detected = star_count; // 【新增】记录检测到的数量
    if (star_count < 3) return STAR_TRACKER_ERROR_SOLVER_FAILED;
    // 2. 将像素坐标转为 3D 观测向量 (使用小孔相机模型)
    static float obs_vectors[STAR_TRACKER_MAX_STARS][3];
    for(int i = 0; i < star_count; i++) {
        // 这里暂时使用检测到的粗像素坐标，后续你可以再串联 centroid_compute 进行亚像素级优化
        camera_pixel_to_vector(&inst->config.camera, (float)star_pixels[i][0], (float)star_pixels[i][1], obs_vectors[i]);
    }

    // 3. 星图识别 (去数据库里找这些星星到底是谁)
    	static uint32_t matched_ids[STAR_TRACKER_MAX_STARS];
        int match_count = 0;

        // 调用匹配函数
        IdentifierStatus id_status = star_identifier_match(inst->identifier, obs_vectors, star_count, matched_ids, &match_count);

        *out_matched = match_count; // 【新增】记录匹配上的数量

        // 判断是否匹配成功且数量足够
        if (id_status != IDENTIFIER_SUCCESS || match_count < 3) {
            return STAR_TRACKER_ERROR_SOLVER_FAILED;
        }
    // 4. 构建姿态解算需要的数据结构
    static DavenportVector3 body_vec[STAR_TRACKER_MAX_STARS];
    static DavenportVector3 ref_vec[STAR_TRACKER_MAX_STARS];
    int valid_pairs = 0;

    for(int i = 0; i < star_count; i++) {
        if (matched_ids[i] != 0xFFFFFFFF) { // 只处理成功匹配的星点 (对应前面的修改)
            float ref_v[3];
            // 从星表取出它的绝对天球坐标
            if (catalog_get_vector_by_index((uint16_t)matched_ids[i], ref_v) == CATALOG_SUCCESS) {
                body_vec[valid_pairs].x = obs_vectors[i][0];
                body_vec[valid_pairs].y = obs_vectors[i][1];
                body_vec[valid_pairs].z = obs_vectors[i][2];

                ref_vec[valid_pairs].x = ref_v[0];
                ref_vec[valid_pairs].y = ref_v[1];
                ref_vec[valid_pairs].z = ref_v[2];
                valid_pairs++;
            }
        }
    }

    if (valid_pairs < 2) {
        return STAR_TRACKER_ERROR_SOLVER_FAILED;
    }

    // 5. 进行 Davenport 姿态解算
    DavenportResult qres;
    DavenportStatus ds = davenport_solve(&inst->davenport_cfg, body_vec, ref_vec, NULL, valid_pairs, &qres);

    if (ds != DAVENPORT_SUCCESS) {
        return STAR_TRACKER_ERROR_SOLVER_FAILED;
    }

    // 输出正确映射的四元数
    quat_out[0] = qres.quat.w;
    quat_out[1] = -qres.quat.x;
    quat_out[2] = -qres.quat.y;
    quat_out[3] = -qres.quat.z;

    return STAR_TRACKER_SUCCESS;
}
