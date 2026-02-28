/**
 * @file centroid.h
 * @brief 质心提取模块接口（平方加权质心算法）
 * 
 * 本模块实现亚像素级质心定位，采用带背景扣除的平方加权质心，
 * 适用于星敏感器中弥散星点的精确定位。
 */

#ifndef CENTROID_H
#define CENTROID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 错误码 */
typedef enum {
    CENTROID_SUCCESS = 0,
    CENTROID_ERROR_INVALID_PARAM = -1,   ///< 参数无效（空指针、窗口尺寸错误）
    CENTROID_ERROR_NO_VALID_PIXELS = -2, ///< 没有超过阈值的像素
    CENTROID_ERROR_BAD_ROI = -3          ///< ROI 提取失败（尺寸不匹配）
} CentroidStatus;

/**
 * @brief 质心算法类型枚举（预留扩展）
 */
typedef enum {
    CENTROID_SQUARE_WEIGHTED = 0,  ///< 平方加权质心（默认推荐）
    // 后续可扩展其他算法，如普通加权质心
} CentroidMethod;

/**
 * @brief 质心提取配置结构
 */
typedef struct {
    CentroidMethod method;   ///< 算法类型（目前仅平方加权）
    float threshold_sigma;   ///< 阈值系数：背景 + threshold_sigma * 噪声标准差
    int window_size;         ///< ROI 窗口大小（必须为奇数，如 5, 7, 9）
} CentroidConfig;

/**
 * @brief 设置默认配置
 * @param config 输出配置结构体
 */
void centroid_config_default(CentroidConfig* config);

/**
 * @brief 从图像窗口（ROI）计算平方加权质心
 * 
 * @param config     [in]  配置参数
 * @param roi        [in]  图像窗口数据（一维数组，行优先，大小为 window_size*window_size）
 * @param roi_width  [in]  窗口宽度（应与 config->window_size 一致）
 * @param roi_height [in]  窗口高度（应与 config->window_size 一致）
 * @param cx         [out] 质心x坐标（相对于窗口左上角，亚像素）
 * @param cy         [out] 质心y坐标
 * @return CentroidStatus
 */
CentroidStatus centroid_compute(
    const CentroidConfig* config,
    const uint16_t* roi,
    int roi_width, int roi_height,
    float* cx, float* cy
);

#ifdef __cplusplus
}
#endif

#endif // CENTROID_H
