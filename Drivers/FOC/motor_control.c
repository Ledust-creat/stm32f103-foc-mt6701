#include "motor_control.h"
#include "foc_config.h"
#include "foc_filter.h"
#include "foc_math.h"
#include "foc_pid.h"
#include "mt6701.h"
#include "cmsis_compiler.h"
#include <math.h>
#include <stdint.h>


/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/

#define MOTOR_CONTROL_ZERO_SAMPLES        (256U)                 /* 电流零偏平均采样帧数 */
#define MOTOR_CONTROL_TWO_PI_F            (2.0f * FOC_PI_F)      /* 单圈机械角周期，rad */
#define MOTOR_CONTROL_LOGIC_PERIOD_RAD    (4.0f * FOC_PI_F)      /* 逻辑角度两圈周期，折返范围[-2π, 2π) */
#define MOTOR_CONTROL_SPEED_ALPHA         (0.1f)                 /* 速度低通滤波输入权重 */
#define MOTOR_CONTROL_CURRENT_ALPHA       (0.3f)                 /* dq电流低通滤波输入权重 */

typedef enum
{
    MOTOR_CONTROL_STATE_DISABLED = 0, /* 仅采样和发布反馈 */
    MOTOR_CONTROL_STATE_RUNNING,      /* ADC中断执行电流环并更新PWM */
    MOTOR_CONTROL_STATE_FAULT         /* 故障锁存，需重新初始化清除 */
} MotorControlState_t;

/*
*********************************************************************************************************
*   模块变量
*********************************************************************************************************
*/

/* ADC ISR写入零点与电流；main仅低频观察，dq两值可能跨一个采样周期。 */
static volatile float s_u_zero_v;                 /* U相零电流偏置电压，单位V */
static volatile float s_v_zero_v;                 /* V相零电流偏置电压，单位V */
static float s_u_zero_sum_v;                      /* U相校准期间的电压累加值 */
static float s_v_zero_sum_v;                      /* V相校准期间的电压累加值 */
static volatile uint16_t s_zero_sample_count;     /* 已完成的零点校准帧数 */
static volatile float s_current_u_a;              /* 最新U相电流，单位A */
static volatile float s_current_v_a;              /* 最新V相电流，单位A */
static volatile float s_id_a;                     /* 滤波后的d轴电流，单位A */
static volatile float s_iq_a;                     /* 滤波后的q轴电流，单位A */
static volatile float s_uq_norm;                  /* 最近一次q轴归一化电压指令；停机时为0 */

/* 控制配置与运行状态；电流PI由ADC更新，位置/速度PI由TIM3更新。 */
static volatile float s_id_target_a;               /* d轴电流目标，单位A */
static volatile float s_iq_target_a;               /* q轴电流目标，单位A */
static volatile float s_electrical_zero_mechanical_angle_rad; /* 对齐时的机械角零位，单位rad */
static volatile bool s_electrical_zero_valid;      /* 电角度零位是否已设置 */
static volatile MotorControlState_t s_current_control_state; /* 电流控制运行状态 */
static volatile MotorControlMode_t s_control_mode;  /* 初始化时选择的控制模式 */
static FocPid_t s_id_pi;                           /* d轴电流PI状态 */
static FocPid_t s_iq_pi;                           /* q轴电流PI状态 */
static FocPid_t s_speed_pi;                        /* 速度PI状态；POSITION时作为位置环内环 */
static FocPid_t s_position_pi;                     /* 位置PI状态；位置模式输出速度目标 */
static volatile float s_speed_target_rad_s;        /* 机械速度目标，单位rad/s */
static volatile float s_position_target_rad;       /* 位置模式目标，单位rad */

/* TIM3速度估算状态。 */
static volatile float s_speed_rad_s;               /* 滤波后的机械速度，单位rad/s */
static volatile bool s_speed_valid;                /* 已取得速度反馈，且连续停更未达到配置阈值 */
static float s_last_angle_rad;                     /* 上次速度更新时的单圈机械角，单位rad */
static volatile uint32_t s_last_speed_encoder_frame; /* 上次用于速度估算的MT6701帧号；主循环用于判断位置反馈有效 */
static volatile float s_motor_logic_angle_rad;     /* 相对上电首个有效样本的机械逻辑角度，单位rad，范围[-2π, 2π) */
static uint16_t s_speed_elapsed_ticks;             /* 两个有效编码器帧之间经过的TIM3节拍数 */



/*
*********************************************************************************************************
*   函 数 名: MotorControl_ResetCurrentPid
*   功能说明: 清除d/q电流PI的历史状态，避免旧积分影响重新使能后的输出
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void MotorControl_ResetCurrentPid(void)
{
    FocPid_Reset(&s_id_pi);
    FocPid_Reset(&s_iq_pi);
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_LatchFault
*   功能说明: 锁存控制故障、禁止后续PWM更新并复位电流PI；硬件回调负责关PWM
*   形    参: 无
*   返 回 值: MOTOR_CONTROL_ADC_FAULT
*********************************************************************************************************
*/
static MotorControlAdcResult_t MotorControl_LatchFault(void)
{
    s_current_control_state = MOTOR_CONTROL_STATE_FAULT;
    s_speed_target_rad_s = 0.0f;
    s_iq_target_a = 0.0f;
    s_uq_norm = 0.0f;
    MotorControl_ResetCurrentPid();
    FocPid_Reset(&s_speed_pi);
    FocPid_Reset(&s_position_pi);
    return MOTOR_CONTROL_ADC_FAULT;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetElectricalZeroAngle
*   功能说明: 保存转子对齐后的机械角，作为机械角到电角度的零位
*   形    参: mechanical_angle_rad - 对齐时的单圈机械角，单位rad
*   返 回 值: true表示[0,2π)内的零位已保存；超出范围时返回false
*********************************************************************************************************
*/
bool MotorControl_SetElectricalZeroAngle(float mechanical_angle_rad)
{
    if (!(mechanical_angle_rad >= 0.0f && mechanical_angle_rad < MOTOR_CONTROL_TWO_PI_F))
    {
        return false;
    }
    s_electrical_zero_mechanical_angle_rad = mechanical_angle_rad;
    s_electrical_zero_valid = true;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_ElectricalAngle
*   功能说明: 将MT6701单圈机械角减去对齐零位后乘极对数，并折返到单个电周期
*   形    参: mechanical_angle_rad - 当前单圈机械角，单位rad
*   返 回 值: [-π, π)范围内的转子电角度，单位rad
*********************************************************************************************************
*/
static float MotorControl_ElectricalAngle(float mechanical_angle_rad)
{
    float electrical_angle_rad =
        (mechanical_angle_rad - s_electrical_zero_mechanical_angle_rad) * (float)POLE_PAIRS;

    /* CMSIS sin/cos的角度输入折返到一圈内。 */
    return FOC_CyclicDiff(electrical_angle_rad, 0.0f, MOTOR_CONTROL_TWO_PI_F);
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_Init
*   功能说明: 选择控制模式并初始化反馈状态及PI；POSITION模式组成位置/速度/电流串级
*   形    参: mode - CURRENT直接控电流，SPEED控速度，POSITION执行位置到速度到电流串级
*   返 回 值: true表示初始化成功；不支持的模式或PI初始化失败返回false
*********************************************************************************************************
*/
bool MotorControl_Init(MotorControlMode_t mode)
{
    if ((mode != MOTOR_CONTROL_MODE_CURRENT) &&
        (mode != MOTOR_CONTROL_MODE_SPEED) &&
        (mode != MOTOR_CONTROL_MODE_POSITION))
    {
        return false;
    }
    s_u_zero_v = 0.0f;
    s_v_zero_v = 0.0f;
    s_u_zero_sum_v = 0.0f;
    s_v_zero_sum_v = 0.0f;
    s_zero_sample_count = 0U;
    s_current_u_a = 0.0f;
    s_current_v_a = 0.0f;
    s_id_a = 0.0f;
    s_iq_a = 0.0f;
    s_uq_norm = 0.0f;
    s_id_target_a = 0.0f;
    s_iq_target_a = 0.0f;
    s_electrical_zero_mechanical_angle_rad = 0.0f;
    s_electrical_zero_valid = false;
    s_current_control_state = MOTOR_CONTROL_STATE_DISABLED;
    s_control_mode = mode;
    s_speed_target_rad_s = 0.0f;
    s_position_target_rad = 0.0f;
    s_speed_rad_s = 0.0f;
    s_speed_valid = false;
    s_last_angle_rad = 0.0f;
    s_last_speed_encoder_frame = 0U;
    s_motor_logic_angle_rad = 0.0f;
    s_speed_elapsed_ticks = 0U;
    /* 速度PI按TIM3标称930 Hz离散化；实际约929.37 Hz，约0.07%误差。 */
    if (!FocPid_Init(&s_id_pi, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI_PER_S, 0.0f,
                    MOTOR_CURRENT_LOOP_PERIOD_S, -MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM,
                    MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM) ||
        !FocPid_Init(&s_iq_pi, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI_PER_S, 0.0f,
                    MOTOR_CURRENT_LOOP_PERIOD_S, -MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM,
                    MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM) ||
        !FocPid_Init(&s_position_pi, MOTOR_POSITION_KP, MOTOR_POSITION_KI_PER_S, 0.0f,
                     1.0f / (float)motor_speed_calc_freq,
                     -MOTOR_SPEED_MAX_RAD_S, MOTOR_SPEED_MAX_RAD_S) ||
        (((mode == MOTOR_CONTROL_MODE_SPEED) || (mode == MOTOR_CONTROL_MODE_POSITION)) &&
         !FocPid_Init(&s_speed_pi, MOTOR_SPEED_KP, MOTOR_SPEED_KI_PER_S, 0.0f,
                      1.0f / (float)motor_speed_calc_freq,
                      -MOTOR_CURRENT_LIMIT_A, MOTOR_CURRENT_LIMIT_A)))
    {
        s_current_control_state = MOTOR_CONTROL_STATE_FAULT;
        return false;
    }
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_IsCurrentReady
*   功能说明: 判断256次无电流采样的零点平均是否完成；main据此决定能否开始对齐
*   形    参: 无
*   返 回 值: true表示零点已计算，不代表相序与电流方向已验证
*********************************************************************************************************
*/
bool MotorControl_IsCurrentReady(void)
{
    return s_zero_sample_count == MOTOR_CONTROL_ZERO_SAMPLES;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetIdTarget
*   功能说明: 设置d轴电流目标；超出软件电流范围时拒绝，不静默钳位
*   形    参: id_ref_a - d轴目标电流，单位A
*   返 回 值: true表示目标已保存；超出±MOTOR_CURRENT_LIMIT_A时返回false
*********************************************************************************************************
*/
bool MotorControl_SetIdTarget(float id_ref_a)
{
    if (!(id_ref_a >= -MOTOR_CURRENT_LIMIT_A && id_ref_a <= MOTOR_CURRENT_LIMIT_A))
    {
        return false;
    }
    s_id_target_a = id_ref_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetIqTarget
*   功能说明: 设置q轴电流目标；超出软件电流范围时拒绝，不静默钳位
*   形    参: iq_ref_a - q轴目标电流，单位A
*   返 回 值: true表示目标已保存；模式不允许或超出±MOTOR_CURRENT_LIMIT_A时返回false
*********************************************************************************************************
*/
bool MotorControl_SetIqTarget(float iq_ref_a)
{
    if (s_control_mode != MOTOR_CONTROL_MODE_CURRENT)
    {
        return false; /* SPEED模式的Iq目标始终由速度PI拥有。 */
    }
    if (!(iq_ref_a >= -MOTOR_CURRENT_LIMIT_A && iq_ref_a <= MOTOR_CURRENT_LIMIT_A))
    {
        return false;
    }
    s_iq_target_a = iq_ref_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetPid
*   功能说明: 停机时按环路选择PI并重置其历史状态；位置和速度PI在POSITION模式串级运行
*   形    参: loop - d/q电流环、速度环或位置环
*             kp/ki_per_s/kd_s - 连续时间PID参数；本轮Kd须为0
*   返 回 值: true表示指定PI已更新；状态、环路或参数不符时返回false
*********************************************************************************************************
*/
bool MotorControl_SetPid(MotorControlPidLoop_t loop, float kp, float ki_per_s,
                         float kd_s)
{
    if ((s_current_control_state != MOTOR_CONTROL_STATE_DISABLED) || (kd_s != 0.0f))
    {
        return false;
    }

    if ((loop == MOTOR_CONTROL_PID_ID) || (loop == MOTOR_CONTROL_PID_IQ))
    {
        /* 电流环使用ADC周期；上限取当前已使用的d/q默认增益。 */
        if (!(kp >= 0.0f && kp <= MOTOR_CURRENT_KP) ||
            !(ki_per_s >= 0.0f && ki_per_s <= MOTOR_CURRENT_KI_PER_S))
        {
            return false;
        }
        return FocPid_Init((loop == MOTOR_CONTROL_PID_ID) ? &s_id_pi : &s_iq_pi,
                           kp, ki_per_s, kd_s, MOTOR_CURRENT_LOOP_PERIOD_S,
                           -MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM,
                           MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM);
    }

    if (loop == MOTOR_CONTROL_PID_SPEED)
    {
        /* POSITION模式也要使用速度PI作为位置PI的内环。 */
        if (((s_control_mode != MOTOR_CONTROL_MODE_SPEED) &&
             (s_control_mode != MOTOR_CONTROL_MODE_POSITION)) ||
            !(kp >= 0.0f && kp <= MOTOR_SPEED_KP_MAX) ||
            !(ki_per_s >= 0.0f && ki_per_s <= MOTOR_SPEED_KI_PER_S_MAX))
        {
            return false;
        }
        return FocPid_Init(&s_speed_pi, kp, ki_per_s, kd_s,
                           1.0f / (float)motor_speed_calc_freq,
                           -MOTOR_CURRENT_LIMIT_A, MOTOR_CURRENT_LIMIT_A);
    }

    if (loop == MOTOR_CONTROL_PID_POSITION)
    {
        /* 位置误差(rad)对应速度输出(rad/s)，PI周期与TIM3速度环一致。 */
        if (!(kp >= 0.0f && kp <= MOTOR_POSITION_KP_MAX) ||
            !(ki_per_s >= 0.0f && ki_per_s <= MOTOR_POSITION_KI_PER_S_MAX))
        {
            return false;
        }
        return FocPid_Init(&s_position_pi, kp, ki_per_s, kd_s,
                           1.0f / (float)motor_speed_calc_freq,
                           -MOTOR_SPEED_MAX_RAD_S, MOTOR_SPEED_MAX_RAD_S);
    }

    return false;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetSpeedTargetRadS
*   功能说明: 设置速度外环目标；目标是机械速度，正值对应当前定义的正转方向
*   形    参: target_rad_s - 速度目标，单位rad/s，受板级配置范围限制
*   返 回 值: true表示目标已保存；模式、范围或故障状态不允许时返回false
*********************************************************************************************************
*/
bool MotorControl_SetSpeedTargetRadS(float target_rad_s)
{
    if ((s_control_mode != MOTOR_CONTROL_MODE_SPEED) ||
        !(target_rad_s >= 0.0f && target_rad_s <= MOTOR_SPEED_MAX_RAD_S) ||
        (s_current_control_state == MOTOR_CONTROL_STATE_FAULT))
    {
        return false;
    }
    s_speed_target_rad_s = target_rad_s;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetSpeedTargetRadS
*   功能说明: 读取当前机械速度目标
*   形    参: target_rad_s - 调用者提供的输出存储，单位rad/s
*   返 回 值: 输出指针有效时返回true
*********************************************************************************************************
*/
bool MotorControl_GetSpeedTargetRadS(float *target_rad_s)
{
    if (target_rad_s == 0) return false;
    *target_rad_s = s_speed_target_rad_s;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetPositionTargetRad
*   功能说明: 设置POSITION模式的位置目标；运行中更新目标可触发位置到速度串级响应
*   形    参: target_rad - 上电逻辑零点下的机械位置，单位rad，范围[-2π,2π)
*   返 回 值: true表示目标已保存；范围外输入返回false
*********************************************************************************************************
*/
bool MotorControl_SetPositionTargetRad(float target_rad)
{
    if (!(target_rad >= -MOTOR_CONTROL_LOGIC_PERIOD_RAD * 0.5f &&
          target_rad < MOTOR_CONTROL_LOGIC_PERIOD_RAD * 0.5f))
    {
        return false;
    }
    s_position_target_rad = target_rad;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetPositionTargetRad
*   功能说明: 读取POSITION模式当前使用的位置目标
*   形    参: target_rad - 调用者提供的输出存储，单位rad
*   返 回 值: 输出指针有效时返回true
*********************************************************************************************************
*/
bool MotorControl_GetPositionTargetRad(float *target_rad)
{
    if (target_rad == 0) return false;
    *target_rad = s_position_target_rad;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetMechanicalPositionRad
*   功能说明: 读取TIM3按新编码器帧累计的机械逻辑位置，按4π周期折返
*   形    参: position_rad - 调用者提供的输出存储，范围[-2π,2π)
*   返 回 值: 首个有效编码器帧到达后返回true，否则返回false
*********************************************************************************************************
*/
bool MotorControl_GetMechanicalPositionRad(float *position_rad)
{
    if ((position_rad == 0) || (s_last_speed_encoder_frame == 0U)) return false;
    *position_rad = s_motor_logic_angle_rad;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_Enable
*   功能说明: 使能初始化时选择的控制模式；POSITION模式先保持当前位置再运行位置/速度/电流串级
*   形    参: 无；调用者须先显式启动MotorHw PWM
*   返 回 值: true表示所选控制模式已使能；前置条件不满足或故障锁存时返回false
*********************************************************************************************************
*/
bool MotorControl_Enable(void)
{
    float angle_rad;
    float position_rad;
    uint32_t primask;

    if ((s_current_control_state != MOTOR_CONTROL_STATE_DISABLED) ||
        !MotorControl_IsCurrentReady() || !s_electrical_zero_valid ||
        !MT6701_GetMechanicalAngleRad(&angle_rad))
    {
        return false;
    }
    if (((s_control_mode == MOTOR_CONTROL_MODE_SPEED) ||
         (s_control_mode == MOTOR_CONTROL_MODE_POSITION)) && !s_speed_valid)
    {
        return false;
    }
    if ((s_control_mode == MOTOR_CONTROL_MODE_POSITION) &&
        !MotorControl_GetMechanicalPositionRad(&position_rad))
    {
        return false;
    }

    /* 与ADC故障锁存同步切换状态，避免主循环覆盖刚发生的FAULT。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_current_control_state == MOTOR_CONTROL_STATE_FAULT)
    {
        __set_PRIMASK(primask);
        return false;
    }
    MotorControl_ResetCurrentPid();
    FocPid_Reset(&s_speed_pi);
    FocPid_Reset(&s_position_pi);
    if (s_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        s_iq_target_a = 0.0f;
    }
    else if (s_control_mode == MOTOR_CONTROL_MODE_POSITION)
    {
        /* 上电/使能先保持当前轴位置，避免默认目标造成突然转动。 */
        s_position_target_rad = position_rad;
        s_speed_target_rad_s = 0.0f;
        s_iq_target_a = 0.0f;
    }
    s_uq_norm = 0.0f;
    s_current_control_state = MOTOR_CONTROL_STATE_RUNNING;
    __set_PRIMASK(primask);
    return true;
}

/*
*********************************************************************************************************
 *   函 数 名: MotorControl_Disable
 *   功能说明: 禁止所选控制模式输出并清除PI历史状态；硬件PWM由应用层关闭
*   形    参: 无
*   返 回 值: 无；此函数不负责关闭硬件PWM通道
*********************************************************************************************************
*/
void MotorControl_Disable(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING)
    {
        s_current_control_state = MOTOR_CONTROL_STATE_DISABLED;
        s_speed_target_rad_s = 0.0f;
        s_iq_target_a = 0.0f;
        s_uq_norm = 0.0f;
        MotorControl_ResetCurrentPid();
        FocPid_Reset(&s_speed_pi);
        FocPid_Reset(&s_position_pi);
    }
    __set_PRIMASK(primask);
}

/*
*********************************************************************************************************
 *   函 数 名: MotorControl_IsFaulted
*   功能说明: 查询是否存在锁存的PI初始化、过流或无效控制输入故障
*   形    参: 无
*   返 回 值: true表示故障已锁存
*********************************************************************************************************
*/
bool MotorControl_IsFaulted(void)
{
    return s_current_control_state == MOTOR_CONTROL_STATE_FAULT;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_OnAdcVoltageISR
*   功能说明: ADC逐次更新零偏、电流和dq；校零后检查过流，RUNNING时执行PI/SVPWM
*   形    参: u_centered_v/v_centered_v - U/V同步采样电压减理论中点，单位V
*             duty_out                 - 输出占空比存储；只在返回DUTY_READY时有效
*   返 回 值: NO_UPDATE表示仅更新反馈；DUTY_READY表示占空比有效；FAULT表示故障锁存
*********************************************************************************************************
*/
MotorControlAdcResult_t MotorControl_OnAdcVoltageISR(float u_centered_v,
                                                      float v_centered_v,
                                                      FocPhaseDuty_t *duty_out)
{
    float current_u_a;
    float current_v_a;
    float current_w_a;
    float angle_rad;
    float electrical_angle_rad;
    float sin_theta;
    float cos_theta;
    float id_a;
    float iq_a;
    float ud_norm;
    float uq_norm;
    if (duty_out == 0)
    {
        return MotorControl_LatchFault();
    }
    /* FAULT为锁存态；不能因后续采样正常而自行恢复。 */
    if (s_current_control_state == MOTOR_CONTROL_STATE_FAULT)
    {
        return MOTOR_CONTROL_ADC_FAULT;
    }
    /* ADC启动后自动完成256帧零偏平均；校零未结束时不换算电流。 */
    if (s_zero_sample_count < MOTOR_CONTROL_ZERO_SAMPLES)
    {
        /* 校零须在PWM关闭且无相电流时进行；每路分别估计模拟零偏。 */
        s_u_zero_sum_v += u_centered_v;
        s_v_zero_sum_v += v_centered_v;
        s_zero_sample_count++;
        if (s_zero_sample_count == MOTOR_CONTROL_ZERO_SAMPLES)
        {
            s_u_zero_v = s_u_zero_sum_v / (float)MOTOR_CONTROL_ZERO_SAMPLES;
            s_v_zero_v = s_v_zero_sum_v / (float)MOTOR_CONTROL_ZERO_SAMPLES;
        }
        return MOTOR_CONTROL_ADC_NO_UPDATE;
    }


    /* 校零后每个ADC周期都更新相电流和保护；这一段在闭环禁用时也照常运行。 */
    current_u_a = (u_centered_v - s_u_zero_v) / (R_SHUNT * OP_GAIN);
    current_v_a = (v_centered_v - s_v_zero_v) / (R_SHUNT * OP_GAIN);
    current_w_a = -(current_u_a + current_v_a);
    s_current_u_a = current_u_a;
    s_current_v_a = current_v_a;
    /* 对齐时也检查三相电流；目标限幅不能替代实测过流保护。 */
    if ((fabsf(current_u_a) > MOTOR_CURRENT_LIMIT_A) ||
        (fabsf(current_v_a) > MOTOR_CURRENT_LIMIT_A) ||
        (fabsf(current_w_a) > MOTOR_CURRENT_LIMIT_A))
    {
        return MotorControl_LatchFault();
    }
    /* 对齐前只更新相电流，不用尚未设置的电角度零位做Park变换。 */
    if (!s_electrical_zero_valid)
    {
        return (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) ?
            MotorControl_LatchFault() : MOTOR_CONTROL_ADC_NO_UPDATE;
    }
    if (!MT6701_GetMechanicalAngleRad(&angle_rad))
    {
        return (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) ?
            MotorControl_LatchFault() : MOTOR_CONTROL_ADC_NO_UPDATE;
    }
    /* 同次采样的两相电流经Clarke/Park变为A单位的dq反馈。 */
    electrical_angle_rad = MotorControl_ElectricalAngle(angle_rad);
    /* 本周期只计算一次正余弦；Park与SVPWM反Park复用，不缓存到下一周期。 */
    FOC_SinCos(electrical_angle_rad, &sin_theta, &cos_theta);
    FOC_CurrentToDQWithSinCos(current_u_a, current_v_a, sin_theta, cos_theta, &id_a, &iq_a);

    /* 仅对dq反馈低通；前面的U/V/W过流判断仍使用未经滤波的实测电流。 */
    id_a = FocFilter_LowPass(id_a, s_id_a, MOTOR_CONTROL_CURRENT_ALPHA);
    iq_a = FocFilter_LowPass(iq_a, s_iq_a, MOTOR_CONTROL_CURRENT_ALPHA);

    /* 保存滤波结果，供本周期PI及main低频观察。 */
    s_id_a = id_a;
    s_iq_a = iq_a;

    /* 零位和角度可用后，dq反馈也会自动更新；只有显式Enable后才计算控制输出。 */
    if (s_current_control_state != MOTOR_CONTROL_STATE_RUNNING)
    {
        return MOTOR_CONTROL_ADC_NO_UPDATE;
    }

    /* 电流环控制。 */
    ud_norm = FocPid_Update(&s_id_pi, s_id_target_a - id_a);
    uq_norm = FocPid_Update(&s_iq_pi, s_iq_target_a - iq_a);
    /* 计算SVPWM。 */
    FocSvpwm_ComputeDutyWithSinCos(ud_norm, uq_norm, sin_theta, cos_theta, duty_out);
    s_uq_norm = uq_norm;
    return MOTOR_CONTROL_ADC_DUTY_READY;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_OnSpeedTickISR
*   功能说明: 新编码器帧到达时估算机械位置/速度；运行时每拍更新速度外环及位置串级
*   形    参: 无
 *   返 回 值: true表示编码器停更触发故障，motor_hw应立即关闭PWM
*********************************************************************************************************
*/
bool MotorControl_OnSpeedTickISR(void)
{
    float angle_rad;
    uint32_t frame_count;
    float delta_rad;
    float speed_raw_rad_s;

    if (s_speed_elapsed_ticks < UINT16_MAX) s_speed_elapsed_ticks++;
    if (MT6701_GetSample(&angle_rad, &frame_count))
    {
        /* 首帧只建立角度基准；没有角度差时不发布虚假的零速度。 */
        if (s_last_speed_encoder_frame == 0U)
        {
            s_last_angle_rad = angle_rad;
            s_last_speed_encoder_frame = frame_count;
            s_speed_elapsed_ticks = 0U;
        }
        else if (frame_count != s_last_speed_encoder_frame)
        {
            delta_rad = FOC_CyclicDiff(angle_rad, s_last_angle_rad, MOTOR_CONTROL_TWO_PI_F);
            /* 逻辑位置以上电首个角度为零，并按4π折返到[-2π,2π)。 */
            s_motor_logic_angle_rad = FOC_CyclicDiff(s_motor_logic_angle_rad + delta_rad, 0.0f, MOTOR_CONTROL_LOGIC_PERIOD_RAD);
            /* 仅新编码器帧更新速度；重复帧不被误当成零速样本。 */
            if (s_speed_elapsed_ticks == 0U) s_speed_elapsed_ticks = 1U;
            speed_raw_rad_s = delta_rad * (float)motor_speed_calc_freq /
                              (float)s_speed_elapsed_ticks;
            s_speed_rad_s = FocFilter_LowPass(speed_raw_rad_s,
                                              s_speed_rad_s, MOTOR_CONTROL_SPEED_ALPHA);
            s_speed_valid = true;
            s_last_angle_rad = angle_rad;
            s_last_speed_encoder_frame = frame_count;
            s_speed_elapsed_ticks = 0U;
        }
    }

    /* 编码器停更时速度反馈失效；任何运行模式都需要新角度，故锁故障并请求停PWM。 */
    if (s_speed_elapsed_ticks >= MOTOR_SPEED_STALE_TICKS)
    {
        s_speed_valid = false;
        if (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING)
        {
            (void)MotorControl_LatchFault();
            return true;
        }
    }

    /* 外环每个TIM3节拍运行；位置模式按位置→速度→Iq串级生成目标。 */
    if ((s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) && s_speed_valid)
    {
        if (s_control_mode == MOTOR_CONTROL_MODE_POSITION)
        {
            float position_error_rad = FOC_CyclicDiff(
                s_position_target_rad, s_motor_logic_angle_rad, MOTOR_CONTROL_LOGIC_PERIOD_RAD);
            s_speed_target_rad_s = FocPid_Update(&s_position_pi, position_error_rad);
        }
        if ((s_control_mode == MOTOR_CONTROL_MODE_SPEED) ||
            (s_control_mode == MOTOR_CONTROL_MODE_POSITION))
        {
            s_iq_target_a = FocPid_Update(&s_speed_pi,
                                          s_speed_target_rad_s - s_speed_rad_s);
        }
    }
    return false;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetCurrentDQ
*   功能说明: 读取最新发布的滤波后d/q电流反馈，供应用层低频观察
*   形    参: id_a/iq_a - d/q轴电流输出存储，单位A，须为不同的有效指针
*   返 回 值: 输出指针正确时返回true；首个dq更新前数值为0，故障后保留最后反馈
*********************************************************************************************************
*/
bool MotorControl_GetCurrentDQ(float *id_a, float *iq_a)
{
    if ((id_a == 0) || (iq_a == 0) || (id_a == iq_a) )
    {
        return false;
    }

    *id_a = s_id_a;
    *iq_a = s_iq_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetPhaseCurrentUV
*   功能说明: 读取ADC换算后的最新U/V相电流，供短时串口诊断
*   形    参: iu_a/iv_a - U/V相电流输出存储，单位A，须为不同的有效指针
*   返 回 值: 参数有效时返回true；低频读取允许两值来自相邻ADC周期
*********************************************************************************************************
*/
bool MotorControl_GetPhaseCurrentUV(float *iu_a, float *iv_a)
{
    if ((iu_a == 0) || (iv_a == 0) || (iu_a == iv_a))
    {
        return false;
    }

    *iu_a = s_current_u_a;
    *iv_a = s_current_v_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetCurrentTargets
*   功能说明: 读取最近保存的d/q电流目标，单位A
*   形    参: id_ref_a/iq_ref_a - d/q目标输出存储，单位A，须为不同的有效指针
*   返 回 值: 参数有效时返回true
*********************************************************************************************************
*/
bool MotorControl_GetCurrentTargets(float *id_ref_a, float *iq_ref_a)
{
    if ((id_ref_a == 0) || (iq_ref_a == 0) || (id_ref_a == iq_ref_a))
    {
        return false;
    }
    *id_ref_a = s_id_target_a;
    *iq_ref_a = s_iq_target_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetVoltageQ
*   功能说明: 读取最近一次电流PI计算的q轴归一化电压指令，供低频遥测观察限幅
*   形    参: uq_norm - 输出存储；停机或故障后读到0
*   返 回 值: true表示输出指针有效
*********************************************************************************************************
*/
bool MotorControl_GetVoltageQ(float *uq_norm)
{
    if (uq_norm == 0)
    {
        return false;
    }

    *uq_norm = s_uq_norm;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetMechanicalSpeed
*   功能说明: 读取TIM3速度节拍计算并低通后的机械速度
*   形    参: speed_rad_s - 输出存储，单位rad/s
*   返 回 值: 速度有效且指针正确时返回true；否则不写输出
*********************************************************************************************************
*/
bool MotorControl_GetMechanicalSpeed(float *speed_rad_s)
{
    if ((speed_rad_s == 0) || !s_speed_valid)
    {
        return false;
    }
    *speed_rad_s = s_speed_rad_s;
    return true;
}
