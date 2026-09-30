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
#include "foc_config.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "motor_tune_link.h"
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
typedef enum
{
  MAIN_STATE_BOOT = 0, /* 外设启动、校零和转子对齐阶段 */
  MAIN_STATE_READY,    /* PWM关闭，等待有效控制命令 */
  MAIN_STATE_RUNNING,  /* 速度阶跃执行中 */
  MAIN_STATE_STOPPING, /* 速度目标归零，等待转子降至静止 */
  MAIN_STATE_FAULT     /* 本次启动发生故障，禁止再次启动 */
} MainState_t;

/* 应用层只管理一次速度试验的运行阶段；电流/速度 PI 状态由MotorControl持有。 */
static MainState_t s_main_state = MAIN_STATE_BOOT;
static uint32_t s_trial_start_ms;     /* 本次试验起始tick，单位ms */
static uint32_t s_trial_duration_ms;  /* 本次试验时长，单位ms */
static uint32_t s_stop_start_ms;      /* 进入受控停机时的tick，单位ms */
static uint32_t s_still_start_ms;     /* 连续满足静止条件的起始tick，0表示尚未满足 */
static uint32_t s_last_data_ms;       /* 上一帧遥测发送tick，单位ms */

/*
*********************************************************************************************************
*   函 数 名: Main_Stop
*   功能说明: 统一停止当前控制并关闭PWM，供STOP、故障和试验结束路径复用
*   返 回 值: 无
*********************************************************************************************************
*/
static void Main_Stop(void)
{
  MotorControl_Disable();
  MotorHw_StopPwm();
}

/*
*********************************************************************************************************
*   函 数 名: Main_ProcessCommand
*   功能说明: 执行已解析的串口命令；负责应用状态、限值检查及控制/硬件启停顺序
*   形    参: command - motor_tune_link解析出的命令，不包含HAL或FOC内部状态
*   返 回 值: 无；命令被接受时发送OK，不接受的命令不回复
*********************************************************************************************************
*/
static void Main_ProcessCommand(const MotorTuneCommand_t *command)
{
  float speed_rad_s;

  if (command->type == MOTOR_TUNE_PING)
  {
    MotorTuneLink_SendOk();
    return;
  }
  if (command->type == MOTOR_TUNE_STOP)
  {
    Main_Stop();
    s_main_state = MotorControl_IsFaulted() ? MAIN_STATE_FAULT : MAIN_STATE_READY;
    MotorTuneLink_SendOk();
    return;
  }
  if (s_main_state != MAIN_STATE_READY || !MOTOR_SPEED_LIMITS_READY) return;

  if (command->type == MOTOR_TUNE_SET_SPEED_PID)
  {
    if (MotorControl_SetSpeedPid(command->value_a, command->value_b))
      MotorTuneLink_SendOk();
    return;
  }
  if (command->type != MOTOR_TUNE_RUN_SPEED_STEP) return;
  if (!(command->value_a > 0.0f && command->value_a <= MOTOR_SPEED_MAX_RAD_S) ||
      (command->duration_ms == 0U) ||
      (command->duration_ms > MOTOR_SPEED_TRIAL_MAX_MS) ||
      !MotorControl_GetMechanicalSpeed(&speed_rad_s) ||
      !(speed_rad_s >= -MOTOR_SPEED_STOP_THRESHOLD_RAD_S &&
        speed_rad_s <= MOTOR_SPEED_STOP_THRESHOLD_RAD_S))
  {
    return;
  }

  if (!MotorControl_SetSpeedTargetRadS(command->value_a) || !MotorHw_StartPwm())
  {
    Main_Stop();
    return;
  }
  if (!MotorControl_Enable())
  {
    Main_Stop();
    if (MotorControl_IsFaulted()) s_main_state = MAIN_STATE_FAULT;
    return;
  }
  s_trial_duration_ms = command->duration_ms;
  s_trial_start_ms = HAL_GetTick();
  s_last_data_ms = s_trial_start_ms;
  s_main_state = MAIN_STATE_RUNNING;
  MotorTuneLink_SendOk();
}

/*
*********************************************************************************************************
*   函 数 名: Main_PollCommand
*   功能说明: 在主循环中取出一条完整串口命令并交给应用状态机
*   返 回 值: 无
*********************************************************************************************************
*/
static void Main_PollCommand(void)
{
  MotorTuneCommand_t command;
  if (MotorTuneLink_ReadCommand(&command)) Main_ProcessCommand(&command);
}

/*
*********************************************************************************************************
*   函 数 名: Main_UpdateTrial
*   功能说明: 推进限时试验和受控停机状态；超时/故障时统一进入安全停机
*   返 回 值: 无
*********************************************************************************************************
*/
static void Main_UpdateTrial(void)
{
  uint32_t now = HAL_GetTick();
  float speed_rad_s;

  if (MotorControl_IsFaulted())
  {
    Main_Stop();
    s_main_state = MAIN_STATE_FAULT;
    return;
  }
  if (s_main_state == MAIN_STATE_RUNNING &&
      (uint32_t)(now - s_trial_start_ms) >= s_trial_duration_ms)
  {
    (void)MotorControl_SetSpeedTargetRadS(0.0f);
    s_stop_start_ms = now;
    s_still_start_ms = 0U;
    s_main_state = MAIN_STATE_STOPPING;
  }
  if (s_main_state != MAIN_STATE_STOPPING) return;

  if (MotorControl_GetMechanicalSpeed(&speed_rad_s) &&
      (speed_rad_s >= -MOTOR_SPEED_STOP_THRESHOLD_RAD_S) &&
      (speed_rad_s <= MOTOR_SPEED_STOP_THRESHOLD_RAD_S))
  {
    if (s_still_start_ms == 0U) s_still_start_ms = now;
    if ((uint32_t)(now - s_still_start_ms) >= 50U)
    {
      Main_Stop();
      s_main_state = MAIN_STATE_READY;
      return;
    }
  }
  else
  {
    s_still_start_ms = 0U;
  }
  if ((uint32_t)(now - s_stop_start_ms) >= MOTOR_SPEED_STOP_TIMEOUT_MS)
  {
    Main_Stop();
    s_main_state = MAIN_STATE_FAULT;
  }
}

/*
*********************************************************************************************************
*   函 数 名: Main_SendDataIfDue
*   功能说明: 试验运行期间每10ms发送速度与Iq遥测；不在任何ISR中格式化或发送
*   返 回 值: 无
*********************************************************************************************************
*/
static void Main_SendDataIfDue(void)
{
  uint32_t now = HAL_GetTick();
  float speed_ref;
  float speed;
  float id_ref;
  float iq_ref;
  float id;
  float iq;

  if ((s_main_state != MAIN_STATE_RUNNING) && (s_main_state != MAIN_STATE_STOPPING)) return;
  if ((uint32_t)(now - s_last_data_ms) < 10U) return;
  s_last_data_ms = now;
  if (!MotorControl_GetMechanicalSpeed(&speed) ||
      !MotorControl_GetSpeedTargetRadS(&speed_ref) ||
      !MotorControl_GetCurrentTargets(&id_ref, &iq_ref) ||
      !MotorControl_GetCurrentDQ(&id, &iq)) return;
  (void)id_ref;
  (void)id;
  MotorTuneLink_SendData(now - s_trial_start_ms, speed_ref, speed, iq_ref, iq);
}

/*
*********************************************************************************************************
*   函 数 名: Main_InitMotor
*   功能说明: 启动采样、等待电流零偏校准并施加一次固定电压对齐
*   返 回 值: 无；结束时进入READY或FAULT，PWM保持关闭
*********************************************************************************************************
*/
static void Main_InitMotor(void)
{
  uint32_t start_ms;
  float angle_rad;
  FocPhaseDuty_t duty;

  if (!MotorHw_Init())
  {
    s_main_state = MAIN_STATE_FAULT;
    return;
  }
  start_ms = HAL_GetTick();
  while (!MotorControl_IsCurrentReady() &&
         !MotorControl_IsFaulted() &&
         (uint32_t)(HAL_GetTick() - start_ms) < 1000U)
  {
  }
  if (!MotorControl_IsCurrentReady() || MotorControl_IsFaulted() ||
      !FocSvpwm_ComputeDuty(0.0f, 0.5f, 0.0f, &duty) || !MotorHw_StartPwm())
  {
    MotorHw_StopPwm();
    s_main_state = MAIN_STATE_FAULT;
    return;
  }
  MotorHw_SetPhaseDuty(duty.duty_u, duty.duty_v, duty.duty_w);
  HAL_Delay(400U); /* 仅上电对齐使用；绝不放入中断。 */
  MotorHw_StopPwm();
  if (!MT6701_GetMechanicalAngleRad(&angle_rad) ||
      !MotorControl_SetElectricalZeroAngle(angle_rad))
  {
    s_main_state = MAIN_STATE_FAULT;
    return;
  }
  s_main_state = MAIN_STATE_READY;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  if (!MotorControl_Init(MOTOR_CONTROL_MODE_SPEED) || !MotorTuneLink_Init())
  {
    Error_Handler();
  }
  Main_InitMotor(); /* 校零、转子对齐一次，成功后保持PWM关闭并等待命令。 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Main_PollCommand();
    Main_UpdateTrial();
    Main_SendDataIfDue();
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
