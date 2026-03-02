/**
 * @file config.h
 * @brief 系统全局配置参数，包括通信相关的波特率、帧格式等 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ========================= 通信配置 ========================= */
#define COMM_UART_INSTANCE          &huart1      /**< 使用的UART句柄（RS422） */
#define COMM_DEFAULT_BAUDRATE       115200       /**< 默认波特率 */
#define COMM_FRAME_HEADER            0xAA55       /**< 帧头 */
#define COMM_MAX_DATA_LEN            256          /**< 最大数据域长度 */
#define COMM_TIMEOUT_MS              100          /**< 接收超时（毫秒） */

/* ========================= 系统参数 ========================= */
#define PARAM_FLASH_SECTOR           11           /**< 存储参数的Flash扇区（根据MCU调整） */
#define PARAM_MAGIC                   0x5A5A5A5A   /**< 参数区魔数，用于校验有效性 */

#endif // CONFIG_H
