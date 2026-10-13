#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H
/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/

/* TIM1中心对齐的上下边界均计重复次数：20 kHz、RCR=9产生约4 kHz ADC节拍。 */
#define motor_pwm_freq 20000U              /* 三相PWM目标频率，单位Hz */
#define motor_speed_calc_freq 930U         /* TIM3标称速度估算频率，单位Hz，实际约929 Hz */
#define MOTOR_PWM_REPETITION_COUNTER 9U    /* TIM1重复计数器值；不是速度环分频 */
#define MOTOR_CURRENT_LOOP_FREQ_HZ \
    (((float)motor_pwm_freq * 2.0f) / ((float)MOTOR_PWM_REPETITION_COUNTER + 1.0f))
#define MOTOR_CURRENT_LOOP_PERIOD_S (1.0f / MOTOR_CURRENT_LOOP_FREQ_HZ) /* PI固定周期，单位s */

#define POLE_PAIRS 7U                      /* 电机极对数；机械角乘此值得到电角度 */
#define R_SHUNT 0.02f                      /* 相电流采样电阻，单位Ω */
#define OP_GAIN 50.0f                      /* 采样放大器电压增益，无量纲 */
#define ADC_REFERENCE_VOLT 3.3f            /* ADC参考电压，单位V；理论零电流中点为1.65 V */
#define ADC_BITS 12U                       /* ADC分辨率，原始计数范围0～4095 */

/* Iq目标及实测U/V/W过流限值；1.55 A低于理想±1.65 A量程，仍须满足硬件额定范围。 */
#define MOTOR_CURRENT_LIMIT_A 1.55f

/* d/q轴共用PI：A单位电流误差转换为按母线电压归一化的电压指令；手动改值后烧录。 */
#define MOTOR_CURRENT_KP 0.65f             /* 比例系数，单位1/A */
#define MOTOR_CURRENT_KI_PER_S 80.0f       /* 积分系数，单位1/(A·s) */

/* 每轴限幅1/√6，保证最坏dq矢量幅值≤1/√3；这是电压限幅，不是电流限值。 */
#define MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM 0.40824829f

/* 速度目标及位置PI输出限幅；位置模式允许正、反向，测试仍须限流供电。 */
#define MOTOR_SPEED_MAX_RAD_S 150.0f        /* 机械速度目标幅值上限，单位rad/s */
#define MOTOR_SPEED_KP 0.04538f             /* 位置调参期间固定的速度PI Kp，单位A/(rad/s) */
#define MOTOR_SPEED_KI_PER_S 0.54213f       /* 位置调参期间固定的速度PI Ki，单位A/rad；Kd=0 */
#define MOTOR_SPEED_KP_MAX 0.5f           /* 串口可设Kp上限，单位A/(rad/s) */
#define MOTOR_SPEED_KI_PER_S_MAX 5.0f     /* 串口可设Ki上限，单位A/rad */
#define MOTOR_SPEED_TRIAL_MAX_MS 5000U    /* 串口单次阶跃最长时长，单位ms；速度/位置试验共用 */
#define MOTOR_SPEED_STALE_TICKS 5U        /* 连续5个TIM3节拍无新编码器帧则运行中停机 */

/* 位置模式PI默认增益；位置误差(rad)经PI生成机械速度目标(rad/s)。 */
#define MOTOR_POSITION_KP 15.0f            /* 默认比例增益，单位s^-1 */
#define MOTOR_POSITION_KI_PER_S 0.0f      /* 默认积分增益，单位s^-2 */
#define MOTOR_POSITION_KP_MAX 25.0f        /* 配置上限，单位s^-1 */
#define MOTOR_POSITION_KI_PER_S_MAX 1.0f  /* 配置上限，单位s^-2 */
#endif
