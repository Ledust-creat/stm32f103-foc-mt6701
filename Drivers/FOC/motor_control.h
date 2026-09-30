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

typedef enum
{
    MOTOR_CONTROL_MODE_CURRENT = 0, /* 上层直接给Id/Iq目标 */
    MOTOR_CONTROL_MODE_SPEED,       /* 速度PI给Iq目标，内含电流环 */
    MOTOR_CONTROL_MODE_POSITION     /* 预留位置→速度→电流串级；当前未实现 */
} MotorControlMode_t;

bool MotorControl_Init(MotorControlMode_t mode); /* 选择模式并初始化控制器；不启动外设或PWM。 */
bool MotorControl_Enable(void);                  /* 使能所选模式；需完成校零、对齐，SPEED还需速度PI就绪。 */
void MotorControl_Disable(void);                 /* 退出运行并复位PI；硬件PWM由应用层关闭。 */
bool MotorControl_IsCurrentReady(void);           /* 返回256次电流零偏校准是否完成。 */
bool MotorControl_SetElectricalZeroAngle(float mechanical_angle_rad); /* 设置对齐零位，输入范围[0,2π)。 */
bool MotorControl_SetIdTarget(float id_ref_a);    /* 设置d轴电流目标，单位A，范围±1.55A。 */
bool MotorControl_SetIqTarget(float iq_ref_a);    /* CURRENT模式设置q轴目标，单位A，范围±1.55A。 */
bool MotorControl_IsFaulted(void);                /* 查询锁存故障；需重新初始化才能清除。 */

bool MotorControl_SetSpeedPid(float kp, float ki_per_s); /* 配置速度PI；仅SPEED模式停机时有效。 */
bool MotorControl_SetSpeedTargetRadS(float target_rad_s); /* 设置机械速度目标，单位rad/s。 */
bool MotorControl_GetSpeedTargetRadS(float *target_rad_s); /* 读取机械速度目标，单位rad/s。 */

bool MotorControl_SetPositionTargetRad(float target_rad); /* 保存位置目标，单位rad，范围[-2π,2π)。 */
bool MotorControl_GetPositionTargetRad(float *target_rad); /* 读取已保存的位置目标，单位rad。 */
bool MotorControl_GetMechanicalPositionRad(float *position_rad); /* 读取两圈折返机械位置，单位rad。 */

MotorControlAdcResult_t MotorControl_OnAdcVoltageISR(float u_centered_v,
                                                      float v_centered_v,
                                                      FocPhaseDuty_t *duty_out); /* ADC同步输入V，输出本周期PWM占空比。 */
bool MotorControl_OnSpeedTickISR(void);           /* TIM3速度更新；返回true表示需由硬件层关闭PWM。 */

bool MotorControl_GetCurrentDQ(float *id_a, float *iq_a); /* 读取d/q电流反馈，单位A。 */
bool MotorControl_GetPhaseCurrentUV(float *iu_a, float *iv_a); /* 读取U/V相电流，单位A。 */
bool MotorControl_GetCurrentTargets(float *id_ref_a, float *iq_ref_a); /* 读取d/q电流目标，单位A。 */
bool MotorControl_GetVoltageQ(float *uq_norm);    /* 读取q轴归一化电压指令；停机时为0。 */
bool MotorControl_GetMechanicalSpeed(float *speed_rad_s); /* 读取机械速度反馈，单位rad/s。 */

#endif /* MOTOR_CONTROL_H */
