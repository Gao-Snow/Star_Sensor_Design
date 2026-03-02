/**
 * @file param.h
 * @brief 参数管理模块接口
 * 负责关键参数的存储、加载和更新，支持掉电保存。
 * 存储系统运行参数（曝光、增益、输出频率等）到非易失存储器，
 * 支持三重冗余和CRC校验。
 */

#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 参数结构体（可根据需求扩展） */
typedef struct {
    uint32_t magic;              /**< 魔数，用于标识有效性 */
    uint16_t exposure_us;        /**< 曝光时间（微秒） */
    uint8_t  gain;               /**< 增益（0-255） */
    uint8_t  output_rate_hz;     /**< 输出频率（Hz） */
    uint16_t crc;                /**< CRC16校验值 */
} SystemParams;

/* 错误码 */
typedef enum {
    PARAM_OK = 0,
    PARAM_ERROR_INVALID_PARAM,
    PARAM_ERROR_READ_FAILED,
    PARAM_ERROR_WRITE_FAILED,
    PARAM_ERROR_CRC_MISMATCH,
    PARAM_ERROR_MAGIC_MISMATCH
} ParamStatus;

/**
 * @brief 初始化参数模块（从Flash加载参数）
 * @param params 输出参数结构体
 * @return ParamStatus
 */
ParamStatus param_init(SystemParams* params);

/**
 * @brief 保存参数到Flash（带三重冗余）
 * @param params 要保存的参数
 * @return ParamStatus
 */
ParamStatus param_save(const SystemParams* params);

/**
 * @brief 更新单个参数（不立即写入Flash）
 * @param params 当前参数结构体（将被修改）
 * @param id      参数ID（可自定义枚举）
 * @param value   新值（以uint32_t传递）
 * @return ParamStatus
 */
ParamStatus param_update(SystemParams* params, uint8_t id, uint32_t value);

/**
 * @brief 恢复默认参数
 * @param params 输出默认参数
 */
void param_set_default(SystemParams* params);

#ifdef __cplusplus
}
#endif

#endif // PARAM_H
