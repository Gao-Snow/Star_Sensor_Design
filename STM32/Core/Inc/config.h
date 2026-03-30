/**
 * @file config.h
 * @brief ϵͳȫ�����ò���������ͨ����صĲ����ʡ�֡��ʽ�� 
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include <stm32h7xx_hal_flash.h>  // �ṩ FLASH_SECTOR_12 ����

/* ========================= ͨ������ ========================= */
#define COMM_UART_INSTANCE          &huart1      /**< ʹ�õ�UART�����RS422�� */
#define COMM_DEFAULT_BAUDRATE       115200       /**< Ĭ�ϲ����� */
#define COMM_FRAME_HEADER            0xAA55       /**< ֡ͷ */
#define COMM_MAX_DATA_LEN            256          /**< ��������򳤶� */
#define COMM_TIMEOUT_MS              100          /**< ���ճ�ʱ�����룩 */

/* ========================= ϵͳ���� ========================= */
// 使用Bank1的扇区7（地址范围：0x080E0000 - 0x080FFFFF）
#define PARAM_MAGIC         0x5A5A5A5A
#define PARAM_FLASH_ADDR    0x080E0000
#define PARAM_FLASH_SECTOR  FLASH_SECTOR_7
#define PARAM_SECTOR_SIZE   0x20000    // 128KB
/*
FLASH_SECTOR_12�� STM32 HAL ���ж����һ�� ö�ٳ�������꣩������ STM32 Flash �ĵ� 12 ����
ͨ���� stm32h7xx_hal_flash.h�����������µĶ��壺
#define FLASH_SECTOR_0     ((uint32_t)0x08000000U)
#define FLASH_SECTOR_1     ((uint32_t)0x08008000U)
...
#define FLASH_SECTOR_12    ((uint32_t)0x08100000U)
�����ǵ�ַ��Ҳ������������ţ���Ҫ����оƬ��ȷ��������֮ #define PARAM_FLASH_SECTOR  FLASH_SECTOR_12�У� 
FLASH_SECTOR_12�ᱻ������ʶ������STM32�ĺ꣬Ȼ����ݿ�ͷ��#include <stm32h7xx_hal_flash.h>���õ�12������Ӧ�ĵ�ַ 
*/

/* ========================= ������������ ========================= */
#define STAR_TRACKER_MAX_STARS       50      // ��ദ���ǵ���
#define STAR_TRACKER_ROI_SIZE        7       // ROI ���ڴ�С��������

#endif // CONFIG_H

