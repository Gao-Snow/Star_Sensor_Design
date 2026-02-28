/**
 * @file star_tracker.h
 * @brief 星敏感器主流水线接口
 * 
 * 整合质心提取、星图识别、星表查询、姿态解算等模块。
 * 假设质心提取和星图识别已实现，本模块负责调用。
 */

#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "quest.h"
#include "pinhole_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 错误码（细化） */
typedef enum {
    STAR_TRACKER_SUCCESS = 0,
    STAR_TRACKER_ERROR_INVALID_PARAM = -1,
    STAR_TRACKER_ERROR_NOT_INITIALIZED = -2,
    STAR_TRACKER_ERROR_IMAGE_SIZE_MISMATCH = -3,
    STAR_TRACKER_ERROR_STAR_COUNT_EXCEEDED = -4,
    STAR_TRACKER_ERROR_NO_STARS = -5,
    STAR_TRACKER_ERROR_CENTROID_FAILED = -6,
    STAR_TRACKER_ERROR_IDENTIFY_FAILED = -7,
    STAR_TRACKER_ERROR_CATALOG_FAILED = -8,
    STAR_TRACKER_ERROR_SOLVER_FAILED = -9
} StarTrackerStatus;

/* 配置结构 */
typedef struct {
    int image_width;
    int image_height;
    int max_detected_stars;      // 最大星数
    PinholeCamera camera;         // 相机内参
    // 可添加其他模块的配置
} StarTrackerConfig;

/* 主流水线实例（不透明指针） */
typedef struct StarTrackerInstance StarTrackerInstance;

/**
 * @brief 创建并初始化流水线实例
 * @param config 配置参数
 * @return 实例指针，失败返回NULL
 */
StarTrackerInstance* star_tracker_create(const StarTrackerConfig* config);

/**
 * @brief 销毁流水线实例
 */
void star_tracker_destroy(StarTrackerInstance* inst);

/**
 * @brief 处理一帧图像
 * @param inst         实例
 * @param image        图像数据（灰度，16位）
 * @param star_pixels  星点粗坐标数组（每个元素[x,y]）
 * @param star_count   星点数量
 * @param quat_out     输出四元数 [w,x,y,z]（内存由调用者分配）
 * @return StarTrackerStatus
 */
StarTrackerStatus star_tracker_process_frame(
    StarTrackerInstance* inst,
    const uint16_t* image,
    const int star_pixels[][2],
    int star_count,
    float quat_out[4]
);

#ifdef __cplusplus
}
#endif

#endif // STAR_TRACKER_H
