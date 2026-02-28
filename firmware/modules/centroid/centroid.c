/**
 * @file centroid.c
 * @brief 质心提取模块实现（平方加权质心算法）
 * 
 * 算法步骤：
 * 1. 估计背景和噪声标准差（取窗口边缘像素）
 * 2. 计算阈值 = 背景 + threshold_sigma * 噪声
 * 3. 对超过阈值的像素计算平方加权质心
 * 4. 输出亚像素坐标
 */

#include "centroid.h"
#include <math.h>
#include <string.h>
#include <float.h>

// 内部函数：估计背景和噪声（使用窗口四周的像素）
static void estimate_background_and_noise(const uint16_t* roi,
                                          int width, int height,
                                          float* background,
                                          float* noise_std) {
    int count = 0;
    double sum = 0.0, sum_sq = 0.0;

    // 取四周像素：第一行、最后一行、第一列、最后一列（避免重复计算角点）
    // 第一行 (y=0)
    for (int x = 0; x < width; ++x) {
        float val = (float)roi[x];
        sum += val;
        sum_sq += val * val;
        count++;
    }
    // 最后一行 (y=height-1)
    if (height > 1) {
        int row_start = (height - 1) * width;
        for (int x = 0; x < width; ++x) {
            float val = (float)roi[row_start + x];
            sum += val;
            sum_sq += val * val;
            count++;
        }
    }
    // 第一列 (x=0) 和最后一列 (x=width-1)，跳过已计入的角点
    for (int y = 1; y < height - 1; ++y) {
        int row = y * width;
        // 第一列
        float val_left = (float)roi[row];
        sum += val_left;
        sum_sq += val_left * val_left;
        count++;
        // 最后一列
        float val_right = (float)roi[row + width - 1];
        sum += val_right;
        sum_sq += val_right * val_right;
        count++;
    }

    if (count > 0) {
        *background = (float)(sum / count);
        // 计算标准差： sqrt( (sum(x^2) - n*mean^2) / (n-1) )
        double mean = sum / count;
        double variance = (sum_sq - count * mean * mean) / (count - 1);
        *noise_std = (float)(sqrt(variance > 0 ? variance : 0));
    } else {
        // 降级处理
        *background = 0.0f;
        *noise_std = 0.0f;
    }
}

// 平方加权质心计算（内部，不检查参数）
static CentroidStatus square_weighted_centroid(const uint16_t* roi,
                                               int width, int height,
                                               float threshold,
                                               float* cx, float* cy) {
    double total_weight = 0.0;   // 平方和
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float val = (float)roi[y * width + x];
            if (val > threshold) {
                // 平方加权
                float w = val * val;
                total_weight += w;
                sum_x += x * w;
                sum_y += y * w;
                count++;
            }
        }
    }

    if (count == 0 || total_weight < 1e-6f) {
        return CENTROID_ERROR_NO_VALID_PIXELS;
    }

    *cx = (float)(sum_x / total_weight);
    *cy = (float)(sum_y / total_weight);
    return CENTROID_SUCCESS;
}

// 公开 API
void centroid_config_default(CentroidConfig* config) {
    if (config) {
        config->method = CENTROID_SQUARE_WEIGHTED;
        config->threshold_sigma = 3.0f;   // 3倍噪声标准差
        config->window_size = 7;           // 7x7 窗口
    }
}

CentroidStatus centroid_compute(
    const CentroidConfig* config,
    const uint16_t* roi,
    int roi_width, int roi_height,
    float* cx, float* cy)
{
    if (!config || !roi || !cx || !cy ||
        roi_width <= 0 || roi_height <= 0 ||
        roi_width != config->window_size || roi_height != config->window_size) {
        return CENTROID_ERROR_INVALID_PARAM;
    }

    // 目前仅支持平方加权算法
    if (config->method != CENTROID_SQUARE_WEIGHTED) {
        return CENTROID_ERROR_INVALID_PARAM;
    }

    // 估计背景和噪声
    float background, noise_std;
    estimate_background_and_noise(roi, roi_width, roi_height,
                                  &background, &noise_std);

    // 计算阈值
    float threshold = background + config->threshold_sigma * noise_std;
    if (threshold < 1.0f) threshold = 1.0f;  // 至少为1，避免无像素

    // 调用平方加权质心
    CentroidStatus status = square_weighted_centroid(roi,
                                                     roi_width, roi_height,
                                                     threshold,
                                                     cx, cy);
    return status;
}
