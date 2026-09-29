#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "foc_math.h"

typedef enum
{
    MOTOR_CONTROL_ADC_NO_UPDATE = 0, /* 仅更新反馈/等待校准，不写PWM */
    MOTOR_CONTROL_ADC_DUTY_READY,     /* 已计算本周期SVPWM占空比 */
    MOTOR_CONTROL_ADC_FAULT            /* 故障锁存，硬件层应立即关闭PWM */
} MotorControlAdcResult_t;

/* 清除软件反馈/校准/PI状态；必须在ADC/TIM3中断启动前调用，不会启动硬件或电流环。 */
void MotorControl_Init(void);
/* 256次无电流采样完成时返回true；不代表相序、极性或电角度零位已验证。 */
bool MotorControl_IsCurrentReady(void);
/* 保存机械角作为电角度零位；调用者须保证输入有效，闭环前应先做转子对齐。 */
bool MotorControl_SetElectricalZeroAngle(float mechanical_angle_rad);
/* 设置d轴电流目标(A)，超出±1.55A时钳位。 */
bool MotorControl_SetIdTarget(float id_ref_a);
/* 设置q轴电流目标(A)，超出±1.55A时钳位。 */
bool MotorControl_SetIqTarget(float iq_ref_a);
/* PWM启动后显式使能；要求校零完成、零位及编码器角度有效，不依赖速度反馈。 */
bool MotorControl_EnableCurrentLoop(void);
/* 禁用电流环并清除PI状态；不清除已锁存的保护故障。 */
void MotorControl_DisableCurrentLoop(void);
bool MotorControl_IsCurrentLoopFaulted(void);

/* 由motor_hw的HAL ADC完成回调自动调用；未显式使能时只更新反馈，不输出PWM占空比。 */
MotorControlAdcResult_t MotorControl_OnAdcVoltageISR(float u_centered_v,
                                                      float v_centered_v,
                                                      FocPhaseDuty_t *duty_out);
/* 由motor_hw的TIM3 HAL回调自动调用；依据MT6701新帧更新机械速度。 */
void MotorControl_OnSpeedTickISR(void);

/* 读取最新d/q电流，单位A；低频读取时两值可能跨一个ADC周期。 */
bool MotorControl_GetCurrentDQ(float *id_a, float *iq_a);
/* 临时串口诊断：读取最新U/V相电流，单位A；低频读取时两值可能跨一个ADC周期。 */
bool MotorControl_GetPhaseCurrentUV(float *iu_a, float *iv_a);
/* 读取d/q电流目标，单位A；返回实际保存的限幅后目标。 */
bool MotorControl_GetCurrentTargets(float *id_ref_a, float *iq_ref_a);
/* 最近一次q轴归一化电压指令；停机或故障后为0，供VOFA低频观察。 */
bool MotorControl_GetVoltageQ(float *uq_norm);
/* 机械速度(rad/s)；编码器停更时返回false且不写输出。 */
bool MotorControl_GetMechanicalSpeed(float *speed_rad_s);

#endif /* MOTOR_CONTROL_H */
