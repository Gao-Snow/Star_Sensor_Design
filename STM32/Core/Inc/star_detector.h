#ifndef STAR_DETECTOR_H
#define STAR_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 星点检测配置 */
typedef struct {
    float threshold_sigma;      /**< 阈值系数: 背景 + sigma * 标准差 */
    int min_star_pixels;        /**< 最小星点像素数（用于过滤噪声） */
    int max_stars;              /**< 最大检测星点数 */
} StarDetectorConfig;

/**
 * @brief 设置默认配置
 */
void star_detector_config_default(StarDetectorConfig* config);

/**
 * @brief 从图像中检测星点，输出粗坐标（像素级）
 * 
 * @param config       [in]  配置参数
 * @param image        [in]  图像数据（16位灰度，行优先）
 * @param width        [in]  图像宽度
 * @param height       [in]  图像高度
 * @param star_pixels  [out] 检测到的星点粗坐标数组，每个元素 [x, y]
 * @param max_stars    [in]  star_pixels 数组最大容量
 * @param star_count   [out] 实际检测到的星点数
 * @return true 成功，false 失败（参数错误或内存不足）
 */
bool star_detector_detect(const StarDetectorConfig* config,
                          const uint16_t* image,
                          int width, int height,
                          int star_pixels[][2],
                          int max_stars,
                          int* star_count);

#ifdef __cplusplus
}
#endif

#endif // STAR_DETECTOR_H
