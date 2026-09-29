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
*   功能说明: 每20 ms发送Iq目标、U/V相电流、dq反馈、电压、速度及故障；不得在中断中调用
*   返 回 值: 无；printf由USART2的fputc发送，发送超时由重定向层限制
*********************************************************************************************************
*/
static void Main_PrintVofaIfDue(void)
{
  static uint32_t last_print_tick;
  float id_ref_a = 0.0f, iq_ref_a = 0.0f;
  float iu_a = 0.0f, iv_a = 0.0f;
  float id_a = 0.0f, iq_a = 0.0f, uq_norm = 0.0f;
  float speed_rad_s = 0.0f;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - last_print_tick) < 20U)
  {
    return;
  }
  last_print_tick = now;

  (void)MotorControl_GetCurrentTargets(&id_ref_a, &iq_ref_a);
  (void)MotorControl_GetPhaseCurrentUV(&iu_a, &iv_a);
  (void)MotorControl_GetCurrentDQ(&id_a, &iq_a);
  (void)MotorControl_GetVoltageQ(&uq_norm);
  (void)MotorControl_GetMechanicalSpeed(&speed_rad_s);
  // FireWater列顺序：Iq目标(A)、U/V相电流(A)、Id/Iq反馈(A)、Uq归一化、速度(rad/s)、故障。
  printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u\r\n",
         id_ref_a, iu_a, iv_a, id_a, iq_a, uq_norm, speed_rad_s,
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
  float angle_rad = 0.0f;
  uint32_t start_tick;
  FocPhaseDuty_t duty;
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
  /* 成功后SPI DMA、TIM3速度节拍、TIM1触发的ADC采样会由硬件自动持续触发。 */
  if (!MotorHw_Init())
  {
    Error_Handler();
  }
  // 上电只测试一次；失败、过流或到时都进入同一个停机出口。
  do
  {
    start_tick = HAL_GetTick();
    while (!MotorControl_IsCurrentReady() &&
           !MotorControl_IsCurrentLoopFaulted() &&
           (uint32_t)(HAL_GetTick() - start_tick) < 1000U)
    {
      Main_PrintVofaIfDue();
    }
    if (!MotorControl_IsCurrentReady() || MotorControl_IsCurrentLoopFaulted())
    {
      break;
    }

    // 固定d轴电压对齐；幅值沿用当前0.5，须限流供电且确认转子实际对齐。
    if (!FocSvpwm_ComputeDuty(0.0f, 0.5f, 0.0f, &duty) || !MotorHw_StartPwm())
    {
      break;
    }
	  MotorHw_SetPhaseDuty(duty.duty_u, duty.duty_v, duty.duty_w);

    HAL_Delay(400U); // 仅用于上电对齐，不得在ISR中调用。

    if (MotorControl_IsCurrentLoopFaulted() || !MT6701_GetMechanicalAngleRad(&angle_rad))
    {
      break;
    }
	 MotorHw_StopPwm();
    (void)MotorControl_SetElectricalZeroAngle(angle_rad);


    // 先以零电流目标建立基线，再对Iq施加0.1 A阶跃。
    (void)MotorControl_SetIqTarget(0.0f);
    (void)MotorControl_SetIdTarget(0.0f);
    if (!MotorHw_StartPwm() || !MotorControl_EnableCurrentLoop())
    {
      break;
    }

    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) < 100U &&
           !MotorControl_IsCurrentLoopFaulted())
    {
      Main_PrintVofaIfDue();
    }
    if (MotorControl_IsCurrentLoopFaulted())
    {
      break;
    }

    (void)MotorControl_SetIdTarget(0.5f);
    start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) < 2000U &&
           !MotorControl_IsCurrentLoopFaulted())
    {
      Main_PrintVofaIfDue();
    }
  } while (0);

  (void)MotorControl_SetIqTarget(0.0f);
  (void)MotorControl_SetIdTarget(0.0f);
  MotorControl_DisableCurrentLoop();
  MotorHw_StopPwm();
  HAL_Delay(20U);
  Main_PrintVofaIfDue(); // 最后一帧显示停机后的目标、电压和故障；dq可能保留停机前值。
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

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
