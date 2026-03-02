/**
 * @file comm.h
 * @brief 通信模块接口（RS422）
 * 
 * 自定义二进制帧协议：
 * | 帧头 (2B) | 长度 (1B) | 类型 (1B) | 数据域 (N) | CRC16 (2B) |
 * 
 * 支持上行指令和下行遥测。
 */

#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 帧类型定义 */
#define FRAME_TYPE_CMD_RESET        0x01    /**< 重置指令 */
#define FRAME_TYPE_CMD_MODE         0x02    /**< 模式切换指令 */
#define FRAME_TYPE_CMD_SET_PARAM    0x03    /**< 参数设置指令 */
#define FRAME_TYPE_CMD_UPGRADE      0x04    /**< 固件升级启动指令 */
#define FRAME_TYPE_CMD_DATA         0x05    /**< 固件数据包 */

#define FRAME_TYPE_ATTITUDE         0x81    /**< 姿态数据下行 */
#define FRAME_TYPE_TELEMETRY        0x82    /**< 遥测数据下行 */
#define FRAME_TYPE_ACK              0x83    /**< 应答 */

/* 通信状态 */
typedef enum {
    COMM_OK = 0,
    COMM_ERROR_INVALID_PARAM,
    COMM_ERROR_TX_BUSY,
    COMM_ERROR_RX_OVERRUN,
    COMM_ERROR_FRAME_INVALID,
    COMM_ERROR_CRC_MISMATCH
} CommStatus;

/* 回调函数类型，用于处理接收到的指令 */
typedef void (*CommandCallback)(uint8_t cmd, uint8_t* data, uint16_t len);

/**
 * @brief 初始化通信模块
 * @param baudrate 波特率（0表示使用默认）
 */
void comm_init(uint32_t baudrate);

/**
 * @brief 注册指令处理回调
 * @param cb 回调函数
 */
void comm_register_callback(CommandCallback cb);

/**
 * @brief 发送姿态数据（自动打包成帧）
 * @param quat     四元数 [w, x, y, z]
 * @param timestamp 时间戳（毫秒）
 * @param status   状态字
 * @return CommStatus
 */
CommStatus comm_send_attitude(const float quat[4], uint32_t timestamp, uint8_t status);

/**
 * @brief 发送遥测数据（温度、电压、星数、模式、错误码）
 */
CommStatus comm_send_telemetry(int16_t temp_c, uint16_t voltage_mv,
                               uint8_t star_count, uint8_t mode, uint8_t error);

/**
 * @brief 发送简单应答
 * @param ack_code 应答码（0成功，非0错误）
 */
CommStatus comm_send_ack(uint8_t ack_code);

/**
 * @brief 在主循环中调用，处理接收到的数据（非阻塞）
 */
void comm_process(void);

#ifdef __cplusplus
}
#endif

#endif // COMM_H
