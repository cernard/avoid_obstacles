/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "atk_ms53l1m.h"
#include "stdio.h"
#include "oled.h"
#include "bmp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L1X_ADDR 0x52
#define VL53L1X_WRITE 0xEF
#define VL53L1X_READ 0xEE
#define ADDR_24LCxx_WRITE 0xA0
#define ADDR_24LCxx_READ 0xA1
#define BufferSize 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// overwrite _write printf function with USART for clion
#ifdef __GNUC__

int _write(int file, char *ptr, int len) {
    // Send UART DATA with DMA
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) ptr, len + 2);
//    HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 100);
    return len;

}

#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    printf("Program Start!\r\n");

/* ATK_MS53L1M初始化

    uint8_t ret;

    ret = atk_ms53l1m_init(115200, &id);
    if (ret != 0) {
        printf("MS53L1M init failed!\r\n");
        return 0;
    }
    printf("MS53L1M init success!\r\n");

    // Show id
    char buf[23];
    sprintf(buf, "ATK-MS53L1M ID: 0x%04x", id);
    printf("%s\r\n", buf);

    */
/* 设置ATK-MS53L1M的工作模式为Modbus模式 *//*

    ret = atk_ms53l1m_write_data(id, ATK_MS53L1M_FUNCODE_WORKMODE, ATK_MS53L1M_WORKMODE_MODBUS);
    if (ret == 0) {
        */
/* 设置ATK-MS53L1M的测量模式为长距离测量模式 *//*

        atk_ms53l1m_write_data(id, ATK_MS53L1M_FUNCODE_MEAUMODE, ATK_MS53L1M_MEAUMODE_LONG);
        printf("Set to Modbus mode.\r\n");
    } else {
        printf("Set Modbus mode failed!\r\n");
    }

    OLED_Init();
    OLED_ColorTurn(0);//0ʾ1 ɫʾ
    OLED_DisplayTurn(0);//0ʾ 1 Ļתʾ
    OLED_Refresh();

    uint16_t pwmVal = 0;   //PWM占空比
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

    uint32_t capture_Buf[3] = {0};   //存放计数值
    uint8_t capture_Cnt = 0;    //状态标志位
    uint32_t high_time;   //高电平时间
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*
        // 获取ATK-MS53L1M测量值
        uint16_t dat;
        ret = atk_ms53l1m_modbus_get_data(id, &dat);
        if (ret == 0) {
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, dat / 8);

            char str[23];
            // divide by 1000 to get meter
            sprintf(str, "Distance: %1.3fm\r\n", dat / 1000.0);
            printf(str);
            OLED_ShowString(0, 0, str, 16, 1);
            OLED_Refresh();
        } else {
            printf("Modbus mode get data failed!\r\n");
        }
*/
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        if (HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
