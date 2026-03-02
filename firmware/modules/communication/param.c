/**
 * @file param.c
 * @brief 参数管理模块实现
 * 
 * 使用STM32内部Flash模拟EEPROM，将参数存储在指定扇区，
 * 写入时同时写入三个备份，读取时多数表决。
 */

#include "param.h"
#include "config.h"
#include <string.h>
#include <stddef.h>

// 假设使用HAL库
#include "stm32h7xx_hal.h"

// Flash操作相关
#define PARAM_FLASH_ADDR      0x08100000  // 示例地址，需根据实际MCU调整
#define PARAM_BACKUP_COUNT     3
#define PARAM_SECTOR_SIZE      0x20000     // 128KB（H7系列）

// 内部函数声明
static uint16_t crc16(const uint8_t* data, uint32_t len);
static ParamStatus read_params(SystemParams* params);
static ParamStatus write_params(const SystemParams* params);

// 默认参数值
static const SystemParams DEFAULT_PARAMS = {
    .magic = PARAM_MAGIC,
    .exposure_us = 20000,    // 20ms
    .gain = 16,
    .output_rate_hz = 5,
    .crc = 0  // 将在初始化时计算
};

ParamStatus param_init(SystemParams* params) {
    if (!params) return PARAM_ERROR_INVALID_PARAM;

    // 尝试从Flash读取
    ParamStatus status = read_params(params);
    if (status == PARAM_OK) {
        // 验证魔数和CRC
        if (params->magic != PARAM_MAGIC) {
            status = PARAM_ERROR_MAGIC_MISMATCH;
        } else {
            // 计算CRC（排除crc字段自身）
            uint16_t crc_calc = crc16((uint8_t*)params, offsetof(SystemParams, crc));
            if (crc_calc != params->crc) {
                status = PARAM_ERROR_CRC_MISMATCH;
            }
        }
    }

    // 如果读取失败或校验失败，使用默认参数
    if (status != PARAM_OK) {
        param_set_default(params);
        // 尝试将默认参数写入Flash
        param_save(params);
    }

    return PARAM_OK;
}

ParamStatus param_save(const SystemParams* params) {
    if (!params) return PARAM_ERROR_INVALID_PARAM;
    return write_params(params);
}

ParamStatus param_update(SystemParams* params, uint8_t id, uint32_t value) {
    if (!params) return PARAM_ERROR_INVALID_PARAM;

    // 根据ID更新对应字段（示例，可扩展）
    switch (id) {
        case 0: // 曝光时间
            if (value >= 1000 && value <= 500000) // 范围检查
                params->exposure_us = (uint16_t)value;
            else
                return PARAM_ERROR_INVALID_PARAM;
            break;
        case 1: // 增益
            if (value <= 255)
                params->gain = (uint8_t)value;
            else
                return PARAM_ERROR_INVALID_PARAM;
            break;
        case 2: // 输出频率
            if (value == 1 || value == 2 || value == 5 || value == 10)
                params->output_rate_hz = (uint8_t)value;
            else
                return PARAM_ERROR_INVALID_PARAM;
            break;
        default:
            return PARAM_ERROR_INVALID_PARAM;
    }

    // 更新CRC
    params->crc = crc16((uint8_t*)params, offsetof(SystemParams, crc));
    return PARAM_OK;
}

void param_set_default(SystemParams* params) {
    if (!params) return;
    memcpy(params, &DEFAULT_PARAMS, sizeof(SystemParams));
    params->crc = crc16((uint8_t*)params, offsetof(SystemParams, crc));
}

/* ==================== 内部实现 ==================== */

// 简单CRC16 (CCITT)
static uint16_t crc16(const uint8_t* data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// 从Flash读取参数（三份备份，多数表决）
static ParamStatus read_params(SystemParams* params) {
    uint32_t base = PARAM_FLASH_ADDR;
    SystemParams copies[PARAM_BACKUP_COUNT];
    uint32_t errors[PARAM_BACKUP_COUNT] = {0};

    // 读取三个备份
    for (int i = 0; i < PARAM_BACKUP_COUNT; ++i) {
        memcpy(&copies[i], (const void*)(base + i * PARAM_SECTOR_SIZE), sizeof(SystemParams));
        // 校验每个备份的CRC
        uint16_t crc_calc = crc16((uint8_t*)&copies[i], offsetof(SystemParams, crc));
        if (copies[i].magic != PARAM_MAGIC || crc_calc != copies[i].crc) {
            errors[i] = 1;
        }
    }

    // 找没有错误的备份
    int good_idx = -1;
    for (int i = 0; i < PARAM_BACKUP_COUNT; ++i) {
        if (!errors[i]) {
            good_idx = i;
            break;
        }
    }
    if (good_idx >= 0) {
        *params = copies[good_idx];
        return PARAM_OK;
    }

    // 所有备份都损坏
    return PARAM_ERROR_READ_FAILED;
}

// 写入参数到Flash（三个备份）
static ParamStatus write_params(const SystemParams* params) {
    // 解锁Flash
    HAL_FLASH_Unlock();

    // 擦除三个扇区
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = PARAM_FLASH_SECTOR;
    erase.NbSectors = PARAM_BACKUP_COUNT; // 连续扇区
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;
    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return PARAM_ERROR_WRITE_FAILED;
    }

    // 编程三个备份
    uint32_t base = PARAM_FLASH_ADDR;
    for (int i = 0; i < PARAM_BACKUP_COUNT; ++i) {
        uint32_t addr = base + i * PARAM_SECTOR_SIZE;
        // 以32位字为单位写入
        const uint32_t* src = (const uint32_t*)params;
        for (uint32_t j = 0; j < sizeof(SystemParams) / 4; ++j) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + j*4, src[j]) != HAL_OK) {
                HAL_FLASH_Lock();
                return PARAM_ERROR_WRITE_FAILED;
            }
        }
    }

    HAL_FLASH_Lock();
    return PARAM_OK;
}
