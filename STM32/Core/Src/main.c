/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "Star_Tracker_pipeline.h"
#include "param.h"
#include "centroid.h"
#include "Davenport_q.h"
#include "pinhole_camera.h"
#include "config.h"
#include "star_identifier.h"
#include "star_catalog_nav.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static StarTrackerInstance* g_star_tracker = NULL;
uint8_t rx_buffer[160 * 1024];   // 可根据需要调整大小
uint32_t rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();

    HAL_Delay(1000);

    CDC_Transmit_FS((uint8_t*)"Start init\r\n", 12);

    StarTrackerConfig st_config = {
        .image_width = 0,
        .image_height = 0,
        .max_detected_stars = STAR_TRACKER_MAX_STARS,
        .camera = { .fx = 500.0f, .fy = 500.0f, .cx = 160.0f, .cy = 120.0f }
    };

    CDC_Transmit_FS((uint8_t*)"Calling create...\r\n", 19);
    g_star_tracker = star_tracker_create(&st_config);
    CDC_Transmit_FS((uint8_t*)"Create returned\r\n", 17);

    if (g_star_tracker == NULL) {
        CDC_Transmit_FS((uint8_t*)"Tracker init FAILED\r\n", 21);
    } else {
        CDC_Transmit_FS((uint8_t*)"Tracker init OK\r\n", 17);
    }

    // 主循环：完整帧解析 + 姿态输出
    while (1)
    {
        // 如果收到至少 4 字节，尝试解析帧
        if (rx_index >= 4) {
            uint8_t* p = rx_buffer;        // 缓冲区指针
            uint32_t idx = 0;              // 当前搜索位置

            // 1. 查找帧头 0xAA 0x55
            while (idx + 2 <= rx_index) {
                if (p[idx] == 0xAA && p[idx + 1] == 0x55) {
                    break;
                }
                idx++;
            }
            if (idx + 2 > rx_index) {
                // 未找到帧头，清空缓冲区（保留最后一个字节防止跨帧）
                if (rx_index > 0) {
                    memmove(rx_buffer, rx_buffer + rx_index - 1, 1);
                    rx_index = 1;
                }
                continue;
            }

            // 2. 确保能读取宽高（2字节宽+2字节高）
            if (idx + 6 > rx_index) {
                HAL_Delay(10);
                continue;
            }
            uint16_t img_width  = p[idx + 2] | (p[idx + 3] << 8);
            uint16_t img_height = p[idx + 4] | (p[idx + 5] << 8);

            // 3. 计算图像数据长度和总帧长
            uint32_t img_data_len = (uint32_t)img_width * img_height * 2;
            uint32_t frame_len = 6 + img_data_len;   // 帧头2 + 宽2 + 高2 + 图像数据

            if (idx + frame_len > rx_index) {
                // 数据尚未收完，等待下一轮
                HAL_Delay(10);
                continue;
            }

            // 4. 提取图像数据指针
            uint16_t* image = (uint16_t*)(p + idx + 6);

            // 5. 调用星跟踪器处理
            float quat[4];
            int detected = 0, matched = 0; // 【新增】接收状态变量
            StarTrackerStatus status = star_tracker_process_frame(
            g_star_tracker, image, img_width, img_height, quat, &detected, &matched);
            // 【新增】：定义一个静态的发送缓冲区，确保 USB 后台发送时内存不被销毁
            static char tx_buf[128];


            // 6. 根据结果发送姿态或调试错误
            if (status == STAR_TRACKER_SUCCESS) {
            	int len = snprintf(tx_buf, sizeof(tx_buf), "ATT: %f %f %f %f (Det:%d Mat:%d)\r\n",
            			quat[0], quat[1], quat[2], quat[3], detected, matched);
                            if (len > 0) {
                                CDC_Transmit_FS((uint8_t*)tx_buf, len);
                            }
                        } else {
                            int len = snprintf(tx_buf, sizeof(tx_buf), "ERR: Det=%d, Mat=%d\r\n", detected, matched);
                            if (len > 0) {
                                CDC_Transmit_FS((uint8_t*)tx_buf, len);
                            }
                        }

            // 7. 移除已处理的帧
            uint32_t remaining = rx_index - (idx + frame_len);
            if (remaining > 0) {
                memmove(rx_buffer, rx_buffer + idx + frame_len, remaining);
            }
            rx_index = remaining;
        }

        // 每秒打印一次调试信息（可选）
        // 如果不想频繁打印，可以注释掉下面两行
        //char dbg[32];
        //sprintf(dbg, "Hello, rx_index=%d\r\n", rx_index);
        //CDC_Transmit_FS((uint8_t*)dbg, strlen(dbg));

        //HAL_Delay(1000);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* MPU Configuration */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
