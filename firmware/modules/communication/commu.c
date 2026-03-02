/**
 * @file comm.c
 * @brief 通信模块实现（基于UART，使用DMA + 环形缓冲区）
 */

#include "comm.h"
#include <string.h>
#include <stdio.h>

// 假设使用HAL库的UART
extern UART_HandleTypeDef huart1;  // 在main.c中定义

#define RX_BUF_SIZE     512
#define TX_BUF_SIZE     256

static uint8_t rx_buffer[RX_BUF_SIZE];
static uint8_t tx_buffer[TX_BUF_SIZE];

// 环形缓冲区读写索引
static volatile uint16_t rx_rd_idx = 0;
static volatile uint16_t rx_wr_idx = 0;

static CommandCallback user_callback = NULL;

// 内部函数：计算CRC16
static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
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

// 内部函数：发送一帧（使用DMA或轮询）
static CommStatus send_frame(uint8_t type, const uint8_t* data, uint16_t len) {
    if (len > COMM_MAX_DATA_LEN) return COMM_ERROR_INVALID_PARAM;

    uint8_t frame[COMM_MAX_DATA_LEN + 6];
    uint16_t idx = 0;
    frame[idx++] = (COMM_FRAME_HEADER >> 8) & 0xFF;
    frame[idx++] = COMM_FRAME_HEADER & 0xFF;
    frame[idx++] = len;
    frame[idx++] = type;
    memcpy(&frame[idx], data, len);
    idx += len;
    uint16_t crc = crc16(frame, idx);  // 从帧头到数据域结束
    frame[idx++] = crc >> 8;
    frame[idx++] = crc & 0xFF;

    // 使用HAL_UART_Transmit（阻塞）或DMA
    if (HAL_UART_Transmit(&huart1, frame, idx, COMM_TIMEOUT_MS) != HAL_OK) {
        return COMM_ERROR_TX_BUSY;
    }
    return COMM_OK;
}

// UART接收完成回调（应在中断中调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == huart1.Instance) {
        // 将接收到的字节存入环形缓冲区
        rx_buffer[rx_wr_idx] = (uint8_t)(huart1.Instance->DR & 0xFF);
        rx_wr_idx = (rx_wr_idx + 1) % RX_BUF_SIZE;
        // 重新启动接收（单字节模式）
        HAL_UART_Receive_IT(huart, (uint8_t*)&huart1.Instance->DR, 1);
    }
}

// 初始化
void comm_init(uint32_t baudrate) {
    // 假设UART已由HAL_Init初始化，只需配置接收中断
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&huart1.Instance->DR, 1);
    if (baudrate != 0 && baudrate != COMM_DEFAULT_BAUDRATE) {
        // 修改波特率（需重新初始化UART）
        huart1.Init.BaudRate = baudrate;
        HAL_UART_Init(&huart1);
    }
}

void comm_register_callback(CommandCallback cb) {
    user_callback = cb;
}

CommStatus comm_send_attitude(const float quat[4], uint32_t timestamp, uint8_t status) {
    uint8_t data[4*4 + 4 + 1] = {0};
    memcpy(data, quat, 4*4);
    memcpy(data + 16, &timestamp, 4);
    data[20] = status;
    return send_frame(FRAME_TYPE_ATTITUDE, data, 21);
}

CommStatus comm_send_telemetry(int16_t temp_c, uint16_t voltage_mv,
                               uint8_t star_count, uint8_t mode, uint8_t error) {
    uint8_t data[8];
    memcpy(data, &temp_c, 2);
    memcpy(data+2, &voltage_mv, 2);
    data[4] = star_count;
    data[5] = mode;
    data[6] = error;
    return send_frame(FRAME_TYPE_TELEMETRY, data, 7);
}

CommStatus comm_send_ack(uint8_t ack_code) {
    return send_frame(FRAME_TYPE_ACK, &ack_code, 1);
}

// 处理接收数据（在主循环调用）
void comm_process(void) {
    while (rx_rd_idx != rx_wr_idx) {
        uint8_t byte = rx_buffer[rx_rd_idx];
        rx_rd_idx = (rx_rd_idx + 1) % RX_BUF_SIZE;

        // 简单的帧解析状态机
        static enum {
            WAIT_HEAD1, WAIT_HEAD2, WAIT_LEN, WAIT_TYPE, WAIT_DATA, WAIT_CRC1, WAIT_CRC2
        } state = WAIT_HEAD1;
        static uint8_t frame_len, frame_type;
        static uint8_t frame_data[COMM_MAX_DATA_LEN];
        static uint16_t frame_idx, frame_crc;

        switch (state) {
            case WAIT_HEAD1:
                if (byte == (COMM_FRAME_HEADER >> 8)) state = WAIT_HEAD2;
                break;
            case WAIT_HEAD2:
                if (byte == (COMM_FRAME_HEADER & 0xFF)) state = WAIT_LEN;
                else state = WAIT_HEAD1;
                break;
            case WAIT_LEN:
                frame_len = byte;
                if (frame_len > COMM_MAX_DATA_LEN) {
                    state = WAIT_HEAD1;  // 长度无效，丢弃
                } else {
                    state = WAIT_TYPE;
                }
                break;
            case WAIT_TYPE:
                frame_type = byte;
                frame_idx = 0;
                if (frame_len == 0) {
                    state = WAIT_CRC1;  // 无数据域
                } else {
                    state = WAIT_DATA;
                }
                break;
            case WAIT_DATA:
                frame_data[frame_idx++] = byte;
                if (frame_idx == frame_len) {
                    state = WAIT_CRC1;
                }
                break;
            case WAIT_CRC1:
                frame_crc = byte << 8;
                state = WAIT_CRC2;
                break;
            case WAIT_CRC2:
                frame_crc |= byte;
                // 验证CRC（需重新计算从帧头到数据域）
                uint8_t check_buf[COMM_MAX_DATA_LEN + 4];
                check_buf[0] = COMM_FRAME_HEADER >> 8;
                check_buf[1] = COMM_FRAME_HEADER & 0xFF;
                check_buf[2] = frame_len;
                check_buf[3] = frame_type;
                memcpy(check_buf+4, frame_data, frame_len);
                uint16_t calc_crc = crc16(check_buf, 4 + frame_len);
                if (calc_crc == frame_crc) {
                    // 有效帧，调用回调
                    if (user_callback) {
                        user_callback(frame_type, frame_data, frame_len);
                    }
                }
                // 无论成功与否，回到初始状态
                state = WAIT_HEAD1;
                break;
        }
    }
}
