#include "spi.h"
#include "adc.h"
#include "tim.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "foc_config.h"
#include "mt6701.h"

/*
*********************************************************************************************************
*   模块变量
*********************************************************************************************************
*/

static uint8_t s_spi1_tx_dummy[3];             /* MT6701读取时发送的占位字节；TX/RX禁止复用同一DMA缓冲 */
static uint8_t s_spi1_rx_data[3];              /* 仅由SPI1 DMA写入、完成回调读取的固定帧缓冲 */
static volatile uint32_t s_spi1_error_count;   /* SPI1错误ISR累计；后续可接入故障状态与遥测 */

/* 外部传入的占空比只允许映射到TIM1的有效比较范围。 */
static float MotorHw_ClampDuty(float duty)
{
    if (duty < 0.0f)
    {
        return 0.0f;
    }

    if (duty > 1.0f)
    {
        return 1.0f;
    }

    return duty;
}

/* 按ARR+1个计数槽换算CCR；实际输出极性由TIM1通道配置决定。 */
static uint32_t MotorHw_DutyToCompare(float duty)
{
    const uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    return (uint32_t)(MotorHw_ClampDuty(duty) * (float)period_counts);
}

/*
*********************************************************************************************************
*   函 数 名: MotorHw_StartEncoderDma
*   功能说明: 拉低MT6701片选并启动一帧3字节SPI全双工DMA传输
*   返 回 值: HAL_OK表示DMA已接管本帧；失败时恢复CSN高电平，避免编码器长期被选中
*********************************************************************************************************
*/
static HAL_StatusTypeDef MotorHw_StartEncoderDma(void)
{
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive_DMA(&hspi1, s_spi1_tx_dummy, s_spi1_rx_data, sizeof(s_spi1_rx_data));
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
    }

    return status;
}

/*
*********************************************************************************************************
*   函 数 名: MotorHw_Init
 *   功能说明: 校准ADC并启动编码器DMA、同步注入采样与两个定时器计数；不启动PWM通道
*   形    参: 无
*   返 回 值: true表示全部启动成功；false时调用者须进入安全错误处理
*********************************************************************************************************
*/
bool MotorHw_Init(void)
{

    /* 此处只启动外设；启动成功后SPI DMA、ADC注入完成和TIM3周期回调自动发生。 */
    if ((HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) ||
        (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) ||
        (MotorHw_StartEncoderDma() != HAL_OK) ||
        (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) ||
        (HAL_TIM_Base_Start(&htim1) != HAL_OK))
    {
        return false;
    }

    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorHw_StartPwm
*   功能说明: 将三相比较值清零后启动TIM1的CH1、CH2、CH3
*   形    参: 无
*   返 回 值: true表示启动成功；任一通道失败时关闭全部通道并返回false
*********************************************************************************************************
*/
bool MotorHw_StartPwm(void)
{
    MotorHw_SetPhaseDuty(0.0f, 0.0f, 0.0f);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        MotorHw_StopPwm();
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        MotorHw_StopPwm();
        return false;
    }

    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorHw_StopPwm
*   功能说明: 禁止三相输出并清零下一次启动所使用的比较寄存器
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void MotorHw_StopPwm(void)
{
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    MotorHw_SetPhaseDuty(0.0f, 0.0f, 0.0f);

}

/*
*********************************************************************************************************
*   函 数 名: MotorHw_SetPhaseDuty
*   功能说明: 将归一化三相占空比转换为TIM1 CCR1、CCR2、CCR3比较值
*   形    参: duty_u/duty_v/duty_w - 三相归一化占空比，范围[0.0, 1.0]
*   返 回 值: 无；比较寄存器采用预装载，通常在下一个更新事件同步生效
*********************************************************************************************************
*/
void MotorHw_SetPhaseDuty(float duty_u, float duty_v, float duty_w)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorHw_DutyToCompare(duty_u));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MotorHw_DutyToCompare(duty_v));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MotorHw_DutyToCompare(duty_w));
}

/*
*********************************************************************************************************
*   函 数 名: HAL_SPI_TxRxCpltCallback
*   功能说明: SPI1 DMA完成后释放CSN、发布编码器角度并提交下一帧
*   形    参: hspi - 触发回调的SPI句柄；非SPI1事件不处理
*   返 回 值: 无；该中断中不执行FOC计算和日志输出
*********************************************************************************************************
*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        /* Init启动首帧；每次DMA完成后再发下一帧，形成持续的编码器角度采样。 */
        HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
        MT6701_OnSpiFrameISR(s_spi1_rx_data);
        (void)MotorHw_StartEncoderDma();
    }
}

/*
*********************************************************************************************************
*   函 数 名: HAL_SPI_ErrorCallback
*   功能说明: SPI1传输错误时累计故障并释放CSN；本轮连续读取停止
*   形    参: hspi - 触发回调的SPI句柄；非SPI1事件不处理
*   返 回 值: 无；恢复读取需另行触发
*********************************************************************************************************
*/
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        s_spi1_error_count++;
        HAL_GPIO_WritePin(MT6701_CSN_GPIO_Port, MT6701_CSN_Pin, GPIO_PIN_SET);
    }
}

/*
*********************************************************************************************************
*   函 数 名: HAL_ADCEx_InjectedConvCpltCallback
*   功能说明: ADC1双ADC同步完成后换算两相电流输入，并按motor_control结果更新或关闭PWM
*   形    参: hadc - 仅处理主ADC1的完成事件，避免重复发布
*   返 回 值: 无；不打印，控制算法由motor_control执行
*********************************************************************************************************
*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    const float volt_per_count = (float)ADC_REFERENCE_VOLT /
        (float)((1UL << ADC_BITS) - 1UL);
    const float midpoint_v = (float)ADC_REFERENCE_VOLT * 0.5f;
    float u_centered_v;
    float v_centered_v;
    FocPhaseDuty_t duty;
    MotorControlAdcResult_t result;

    /* 由TIM1触发的ADC1/ADC2同步采样自动进入；每次都更新反馈，是否输出由控制状态决定。 */
    if (hadc != &hadc1)
    {
        return;
    }

    u_centered_v = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) *
                   volt_per_count - midpoint_v;
    v_centered_v = (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) *
                   volt_per_count - midpoint_v;
    result = MotorControl_OnAdcVoltageISR(u_centered_v, v_centered_v, &duty);
    if (result == MOTOR_CONTROL_ADC_DUTY_READY)
    {
        MotorHw_SetPhaseDuty(duty.duty_u, duty.duty_v, duty.duty_w);
    }
    else if (result == MOTOR_CONTROL_ADC_FAULT)
    {
        MotorHw_StopPwm();
    }
}

/*
*********************************************************************************************************
*   函 数 名: HAL_TIM_PeriodElapsedCallback
*   功能说明: TIM3周期事件只转交速度估算；TIM1不参与此回调
*********************************************************************************************************
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* TIM3每个更新周期自动调用；这里仅转交速度估算，不启停电流环。 */
    if (htim == &htim3)
    {
        MotorControl_OnSpeedTickISR();
    }
}
