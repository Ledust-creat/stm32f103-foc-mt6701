#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H
/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/

/* Board timing; TIM1 center-aligned with RCR=4 gives an approximately 8 kHz ADC loop. */
#define motor_pwm_freq 20000U
#define motor_speed_calc_freq 930U
#define MOTOR_PWM_REPETITION_COUNTER 4U
#define MOTOR_CURRENT_LOOP_FREQ_HZ \
    (((float)motor_pwm_freq * 2.0f) / ((float)MOTOR_PWM_REPETITION_COUNTER + 1.0f))
#define MOTOR_CURRENT_LOOP_PERIOD_S (1.0f / MOTOR_CURRENT_LOOP_FREQ_HZ)

#define POLE_PAIRS 7U
#define R_SHUNT 0.02f
#define OP_GAIN 50.0f
#define ADC_REFERENCE_VOLT 3.3f
#define ADC_BITS 12U

/* 1.5 A软件限值低于理论约±1.65 A的ADC满量程；仍须满足硬件额定范围。 */
#define MOTOR_CURRENT_LIMIT_A 1.55f

/* d/q轴共用；电流误差(A)到归一化电压，初值为0，试验前手动调节并重新烧录。 */
#define MOTOR_CURRENT_KP 0.05f             /* 比例系数，单位1/A */
#define MOTOR_CURRENT_KI_PER_S 0.0f       /* 积分系数，单位1/(A·s) */

/* 临时台架试验：默认关闭，关闭时上电不执行PWM转子对齐或电流闭环。 */
#define MOTOR_CURRENT_TEST_ENABLE 1U               /* 先仅观察ADC；确认采样正确后再开启电流环测试 */
#define MOTOR_CURRENT_TEST_IQ_A 0.05f               /* q轴阶跃目标，单位A */
#define MOTOR_CURRENT_TEST_BASELINE_MS 200U         /* 零目标基线时间，单位ms */
#define MOTOR_CURRENT_TEST_DURATION_MS 1000U        /* 阶跃保持时间，单位ms */
#define MOTOR_CURRENT_TEST_SPEED_LIMIT_RAD_S 10.0f  /* 临时机械速度截止值，单位rad/s */
#define MOTOR_CURRENT_TEST_PRINT_PERIOD_MS 20U      /* VOFA输出周期，单位ms */

/* 每轴限幅sqrt(1/6)，确保dq矢量在线性调制范围内。 */
#define MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM 0.40824829f

#endif
