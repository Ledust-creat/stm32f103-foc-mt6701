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
#define MOTOR_CONTROL_SPEED_ALPHA         (0.1f)                 /* 速度低通滤波输入权重 */

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

/* ADC ISR写入的零点、电流与dq反馈状态；main通过读取接口取得快照。 */
static volatile float s_u_zero_v;                 /* U相零电流偏置电压，单位V */
static volatile float s_v_zero_v;                 /* V相零电流偏置电压，单位V */
static float s_u_zero_sum_v;                      /* U相校准期间的电压累加值 */
static float s_v_zero_sum_v;                      /* V相校准期间的电压累加值 */
static volatile uint16_t s_zero_sample_count;     /* 已完成的零点校准帧数 */
static volatile float s_current_u_a;              /* 最新U相电流，单位A */
static volatile float s_current_v_a;              /* 最新V相电流，单位A */
static volatile float s_id_a;                     /* 最新d轴电流，单位A */
static volatile float s_iq_a;                     /* 最新q轴电流，单位A */
static volatile float s_uq_norm;                  /* 最近一次q轴归一化电压指令；停机时为0 */
static volatile bool s_dq_valid;                   /* 已有角度零位和有效电流组成dq反馈 */

/* 控制配置与运行状态；PI实例仅由ADC控制中断更新。 */
static volatile float s_iq_target_a;               /* q轴电流目标，单位A；d轴目标固定为0 */
static volatile float s_electrical_zero_mechanical_angle_rad; /* 对齐时的机械角零位，单位rad */
static volatile bool s_electrical_zero_valid;      /* 电角度零位是否已设置 */
static volatile MotorControlState_t s_current_control_state; /* 电流控制运行状态 */
static bool s_current_pi_ready;                    /* 两个PI是否成功初始化 */
static FocPid_t s_id_pi;                           /* d轴电流PI状态 */
static FocPid_t s_iq_pi;                           /* q轴电流PI状态 */

/* TIM3速度估算状态。 */
static volatile float s_speed_rad_s;               /* 滤波后的机械速度，单位rad/s */
static volatile bool s_speed_valid;                /* 最近速度节拍是否收到新的编码器帧 */
static float s_last_angle_rad;                     /* 上次速度更新时的单圈机械角，单位rad */
static uint32_t s_last_speed_encoder_frame;        /* 上次用于速度估算的MT6701帧号 */

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
    s_uq_norm = 0.0f;
    s_dq_valid = false;
    MotorControl_ResetCurrentPid();
    return MOTOR_CONTROL_ADC_FAULT;
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
 *   功能说明: 清零采样/速度状态并初始化d/q电流PI；不启动硬件，也不使能电流环
*   形    参: 无
*   返 回 值: 无；初始化失败时电流环保持不可使能
*********************************************************************************************************
*/
void MotorControl_Init(void)
{
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
    s_dq_valid = false;
    s_iq_target_a = 0.0f;
    s_electrical_zero_mechanical_angle_rad = 0.0f;
    s_electrical_zero_valid = false;
    s_current_control_state = MOTOR_CONTROL_STATE_DISABLED;
    s_speed_rad_s = 0.0f;
    s_speed_valid = false;
    s_last_angle_rad = 0.0f;
    s_last_speed_encoder_frame = 0U;

    s_current_pi_ready =
        FocPid_Init(&s_id_pi, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI_PER_S,
                    0.0f, MOTOR_CURRENT_LOOP_PERIOD_S,
                    -MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM, MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM) &&
        FocPid_Init(&s_iq_pi, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI_PER_S,
                    0.0f, MOTOR_CURRENT_LOOP_PERIOD_S,
                    -MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM, MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM);
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetElectricalZeroAngle
*   功能说明: 保存转子对齐后的机械角，作为机械角到电角度的零位
*   形    参: mechanical_angle_rad - 对齐时的单圈机械角，单位rad
*   返 回 值: true表示零位已保存；调用者须保证输入为有效单圈机械角
*********************************************************************************************************
*/
bool MotorControl_SetElectricalZeroAngle(float mechanical_angle_rad)
{


    s_electrical_zero_mechanical_angle_rad = mechanical_angle_rad;
    s_electrical_zero_valid = true;
    s_dq_valid = false;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_SetIqTarget
*   功能说明: 设置q轴电流目标并按软件电流限值钳位
*   形    参: iq_ref_a - q轴目标电流，单位A
*   返 回 值: true表示已保存限幅后的目标；调用者须保证输入为有限值
*********************************************************************************************************
*/
bool MotorControl_SetIqTarget(float iq_ref_a)
{

    
    s_iq_target_a = FOC_ClampF32(iq_ref_a, -MOTOR_CURRENT_LIMIT_A, MOTOR_CURRENT_LIMIT_A);
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_EnableCurrentLoop
 *   功能说明: 显式打开电流环闸门；就绪后ADC中断才运行PI/SVPWM并更新PWM
*   形    参: 无；调用者须先显式启动MotorHw PWM
*   返 回 值: true表示电流环已使能；任一前置条件不满足或故障锁存时返回false
*********************************************************************************************************
*/
bool MotorControl_EnableCurrentLoop(void)
{
    float angle_rad;
    uint32_t primask;

    if (!s_current_pi_ready || (s_current_control_state == MOTOR_CONTROL_STATE_FAULT) ||
        !MotorControl_IsCurrentReady() ||
        !s_electrical_zero_valid || !s_dq_valid || !s_speed_valid ||
        !MT6701_GetMechanicalAngleRad(&angle_rad))
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
    s_uq_norm = 0.0f;
    s_current_control_state = MOTOR_CONTROL_STATE_RUNNING;
    __set_PRIMASK(primask);
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_DisableCurrentLoop
*   功能说明: 禁止后续ADC中断输出占空比并清除PI历史状态
*   形    参: 无
*   返 回 值: 无；此函数不负责关闭硬件PWM通道
*********************************************************************************************************
*/
void MotorControl_DisableCurrentLoop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING)
    {
        s_current_control_state = MOTOR_CONTROL_STATE_DISABLED;
        s_uq_norm = 0.0f;
        MotorControl_ResetCurrentPid();
    }
    __set_PRIMASK(primask);
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_IsCurrentLoopFaulted
*   功能说明: 查询是否存在锁存的过流、编码器或控制计算故障
*   形    参: 无
*   返 回 值: true表示故障已锁存
*********************************************************************************************************
*/
bool MotorControl_IsCurrentLoopFaulted(void)
{
    return s_current_control_state == MOTOR_CONTROL_STATE_FAULT;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_OnAdcVoltageISR
 *   功能说明: 由motor_hw的ADC完成回调逐次自动调用；累计零偏、更新电流/dq反馈，RUNNING时才运行PI/SVPWM
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
    float id_a;
    float iq_a;
    float ud_norm;
    float uq_norm;

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
    }
    if (!MotorControl_IsCurrentReady())
    {
        return MOTOR_CONTROL_ADC_NO_UPDATE;
    }

    /* 校零后每个ADC周期都更新相电流和保护；这一段在闭环禁用时也照常运行。 */
    current_u_a = (u_centered_v - s_u_zero_v) / (R_SHUNT * OP_GAIN);
    current_v_a = (v_centered_v - s_v_zero_v) / (R_SHUNT * OP_GAIN);
    current_w_a = -(current_u_a + current_v_a);
    s_current_u_a = current_u_a;
    s_current_v_a = current_v_a;

    if ((fabsf(current_u_a) > MOTOR_CURRENT_LIMIT_A) ||
        (fabsf(current_v_a) > MOTOR_CURRENT_LIMIT_A) ||
        (fabsf(current_w_a) > MOTOR_CURRENT_LIMIT_A))
    {
        return MotorControl_LatchFault();
    }

    if (!MT6701_GetMechanicalAngleRad(&angle_rad))
    {
        return (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) ?
            MotorControl_LatchFault() : MOTOR_CONTROL_ADC_NO_UPDATE;
    }
    if (!s_electrical_zero_valid)
    {
        return (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) ?
            MotorControl_LatchFault() : MOTOR_CONTROL_ADC_NO_UPDATE;
    }

    electrical_angle_rad = MotorControl_ElectricalAngle(angle_rad);
    if (!FOC_CurrentToDQ(current_u_a, current_v_a, electrical_angle_rad, &id_a, &iq_a))
    {
        return (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) ?
            MotorControl_LatchFault() : MOTOR_CONTROL_ADC_NO_UPDATE;
    }

    /* 保存本次ADC周期的dq反馈，供main低频观察。 */
    s_id_a = id_a;
    s_iq_a = iq_a;
    s_dq_valid = true;

    /* 零位和角度可用后，dq反馈也会自动更新；只有显式Enable后才计算控制输出。 */
    if (s_current_control_state != MOTOR_CONTROL_STATE_RUNNING)
    {
        return MOTOR_CONTROL_ADC_NO_UPDATE;
    }
    if (duty_out == 0)
    {
        return MotorControl_LatchFault();
    }

    ud_norm = FocPid_Update(&s_id_pi, -id_a);
    uq_norm = FocPid_Update(&s_iq_pi, s_iq_target_a - iq_a);
    if (!FocSvpwm_ComputeDuty(electrical_angle_rad, ud_norm, uq_norm, duty_out))
    {
        return MotorControl_LatchFault();
    }

    s_uq_norm = uq_norm;
    return MOTOR_CONTROL_ADC_DUTY_READY;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_OnSpeedTickISR
*   功能说明: 在TIM3节拍估算机械速度；运行中编码器停更或超速时锁存故障
*   形    参: 无
*   返 回 值: 无；硬件层在本回调后查询故障并关闭PWM
*********************************************************************************************************
*/
void MotorControl_OnSpeedTickISR(void)
{
    float angle_rad;
    uint32_t frame_count;
    float delta_rad;
    float speed_raw_rad_s;

    if (!MT6701_GetSample(&angle_rad, &frame_count))
    {
        s_speed_valid = false;
        if (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING)
        {
            (void)MotorControl_LatchFault(); // 运行中失去编码器反馈，禁止继续沿用旧角度。
        }
        return;
    }

    if (s_last_speed_encoder_frame == 0U)
    {
        s_last_angle_rad = angle_rad;
        s_last_speed_encoder_frame = frame_count;
        return;
    }

    if (frame_count == s_last_speed_encoder_frame)
    {
        s_speed_rad_s = 0.0f;
        s_speed_valid = false;
        s_last_speed_encoder_frame = 0U;
        if (s_current_control_state == MOTOR_CONTROL_STATE_RUNNING)
        {
            (void)MotorControl_LatchFault(); // 本次速度节拍没有新帧，视为编码器停更。
        }
        return;
    }

    delta_rad = FOC_CyclicDiff(angle_rad, s_last_angle_rad, MOTOR_CONTROL_TWO_PI_F);
    speed_raw_rad_s = delta_rad * (float)motor_speed_calc_freq;
    if ((s_current_control_state == MOTOR_CONTROL_STATE_RUNNING) &&
        (fabsf(speed_raw_rad_s) > MOTOR_CURRENT_TEST_SPEED_LIMIT_RAD_S))
    {
        s_speed_rad_s = speed_raw_rad_s; // 故障帧保留未滤波峰值，避免低通滤波掩盖超速。
        s_speed_valid = true;
        (void)MotorControl_LatchFault();
        return;
    }

    s_speed_rad_s = FocFilter_LowPass(speed_raw_rad_s,
                                      s_speed_rad_s, MOTOR_CONTROL_SPEED_ALPHA);
    s_speed_valid = true;
    s_last_angle_rad = angle_rad;
    s_last_speed_encoder_frame = frame_count;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_IsCurrentReady
*   功能说明: 判断U/V两路电流零点平均是否已完成256帧采样
*   形    参: 无
*   返 回 值: true表示零点已就绪；不代表电流方向或相序已验证
*********************************************************************************************************
*/
bool MotorControl_IsCurrentReady(void)
{
    return s_zero_sample_count == MOTOR_CONTROL_ZERO_SAMPLES;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetCurrentU
*   功能说明: 读取最新U相电流
*   形    参: current_u_a - 输出存储，单位A
*   返 回 值: 零点就绪且指针有效时返回true
*********************************************************************************************************
*/
bool MotorControl_GetCurrentU(float *current_u_a)
{
    if ((current_u_a == 0) || !MotorControl_IsCurrentReady())
    {
        return false;
    }
    *current_u_a = s_current_u_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetCurrentV
*   功能说明: 读取最新V相电流
*   形    参: current_v_a - 输出存储，单位A
*   返 回 值: 零点就绪且指针有效时返回true
*********************************************************************************************************
*/
bool MotorControl_GetCurrentV(float *current_v_a)
{
    if ((current_v_a == 0) || !MotorControl_IsCurrentReady())
    {
        return false;
    }
    *current_v_a = s_current_v_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetCurrentDQ
*   功能说明: 读取最新发布的d/q电流反馈，供应用层低频观察
*   形    参: id_a/iq_a - d/q轴电流输出存储，单位A，须为不同的有效指针
*   返 回 值: 零点与dq反馈均有效且参数正确时返回true
*********************************************************************************************************
*/
bool MotorControl_GetCurrentDQ(float *id_a, float *iq_a)
{
    if ((id_a == 0) || (iq_a == 0) || (id_a == iq_a) ||
        !MotorControl_IsCurrentReady() || !s_dq_valid)
    {
        return false;
    }

    *id_a = s_id_a;
    *iq_a = s_iq_a;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: MotorControl_GetCurrentTargets
*   功能说明: 读取d/q电流目标快照；当前d轴目标为0，q轴由设置接口更新
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
    *id_ref_a = 0.0f;
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
