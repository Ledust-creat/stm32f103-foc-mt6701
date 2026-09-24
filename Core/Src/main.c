/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "foc_config.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "mt6701.h"
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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
*********************************************************************************************************
*   函 数 名: Main_PrintVofaIfDue
*   功能说明: 在主循环按20 ms节拍发送ADC原始值和校零后的两相电流；不得在中断中调用
*   返 回 值: 无；printf由USART2的fputc发送，发送超时由重定向层限制
*********************************************************************************************************
*/
static void Main_PrintVofaIfDue(void)
{
  static uint32_t last_print_tick;
  float current_u_a = 0.0f, current_v_a = 0.0f;
  uint32_t raw_u, raw_v;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - last_print_tick) < MOTOR_CURRENT_TEST_PRINT_PERIOD_MS)
  {
    return;
  }
  last_print_tick = now;

  raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  raw_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
  (void)MotorControl_GetCurrentU(&current_u_a);
  (void)MotorControl_GetCurrentV(&current_v_a);
  /* FireWater列顺序：ADC1原始值、ADC2原始值、U电流(A)、V电流(A)、校零完成、故障。 */
  printf("%lu,%lu,%.4f,%.4f,%u,%u\r\n",
         (unsigned long)raw_u, (unsigned long)raw_v,
         current_u_a, current_v_a,
         (unsigned int)MotorControl_IsCurrentReady(),
         (unsigned int)MotorControl_IsCurrentLoopFaulted());
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
#if MOTOR_CURRENT_TEST_ENABLE
  float angle_rad = 0.0f;
  uint32_t start_tick;
  float id_a, iq_a;
#endif
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MT6701_Init();
  MotorControl_Init(); /* 仅初始化软件状态和PI参数；此调用本身不启动ADC、定时器或闭环。 */
  (void)MotorControl_SetIqTarget(0.0f);
  /* 成功后SPI DMA、TIM3速度节拍、TIM1触发的ADC采样会由硬件自动持续触发。 */
  if (!MotorHw_Init())
  {
    Error_Handler();
  }

#if MOTOR_CURRENT_TEST_ENABLE
  /* 闭环测试才等待校零；仅观察时即使校零失败，也继续打印ADC原始值。 */
  start_tick = HAL_GetTick();
  while (!MotorControl_IsCurrentReady())
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= 1000U)
    {
      Error_Handler();
    }
  }

  /* 仅启用测试时通电对齐；ADC过流仍由中断立即关PWM。 */
  if (!MotorHw_StartPwm())
  {
    Error_Handler();
  }
  MotorHw_SetPhaseDuty(0.5f, 0.0f, 0.0f);
  HAL_Delay(400U);
  MotorHw_StopPwm(); // 对齐结束后关闭通道，不能用0占空比代替。
  if (!MT6701_GetMechanicalAngleRad(&angle_rad) ||
      !MotorControl_SetElectricalZeroAngle(angle_rad) ||
      MotorControl_IsCurrentLoopFaulted())
  {
    Error_Handler();
  }

  /* 零位设置会使旧dq反馈失效；等下一次ADC和TIM3反馈后再启动电流环。 */
  start_tick = HAL_GetTick();
	MotorControl_GetCurrentDQ(&id_a, &iq_a);

  if (!MotorHw_StartPwm())
  {
    Error_Handler();
  }
  if (!MotorControl_EnableCurrentLoop())
  {
    MotorHw_StopPwm();
    Error_Handler();
  }

  /* 一次性测试：零目标基线 → 小Iq阶跃 → 无论是否故障都停PWM。 */
  start_tick = HAL_GetTick();
  while (((uint32_t)(HAL_GetTick() - start_tick) < MOTOR_CURRENT_TEST_BASELINE_MS) &&
         !MotorControl_IsCurrentLoopFaulted())
  {
    Main_PrintVofaIfDue();
  }
  if (!MotorControl_IsCurrentLoopFaulted())
  {
    (void)MotorControl_SetIqTarget(MOTOR_CURRENT_TEST_IQ_A);
    start_tick = HAL_GetTick();
    while (((uint32_t)(HAL_GetTick() - start_tick) < MOTOR_CURRENT_TEST_DURATION_MS) &&
           !MotorControl_IsCurrentLoopFaulted())
    {
      Main_PrintVofaIfDue();
    }
  }
  (void)MotorControl_SetIqTarget(0.0f);
  MotorControl_DisableCurrentLoop();
  MotorHw_StopPwm();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    Main_PrintVofaIfDue();
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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
