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

/* 电流误差(A)到SVPWM归一化电压的PI初值；0表示不会主动产生电压，调参后再填写。 */
#define MOTOR_CURRENT_ID_KP 0.0f
#define MOTOR_CURRENT_ID_KI_PER_S 0.0f
#define MOTOR_CURRENT_IQ_KP 0.0f
#define MOTOR_CURRENT_IQ_KI_PER_S 0.0f

/* 每轴限幅sqrt(1/6)，确保dq矢量在线性调制范围内。 */
#define MOTOR_DQ_VOLTAGE_AXIS_LIMIT_NORM 0.40824829f

#endif
