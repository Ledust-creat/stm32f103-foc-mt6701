#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*   宏定义
*********************************************************************************************************
*/

#define FOC_PI_F                       (3.14159265358979323846f)
#define FOC_RAD_PER_DEG_F              (FOC_PI_F / 180.0f)
#define FOC_DEG_PER_RAD_F              (180.0f / FOC_PI_F)

/* 角度单位换算；输入输出均为float。 */
static inline float FOC_DegToRad(float degree)
{
    return degree * FOC_RAD_PER_DEG_F;
}

static inline float FOC_RadToDeg(float radian)
{
    return radian * FOC_DEG_PER_RAD_F;
}

/* 将数值限制到[lower_limit, upper_limit]；调用者须保证下限不大于上限。 */
static inline float FOC_ClampF32(float value, float lower_limit, float upper_limit)
{
    return (value < lower_limit) ? lower_limit : ((value > upper_limit) ? upper_limit : value);
}

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/

typedef struct
{
    float duty_u; /* U相占空比，范围[0,1] */
    float duty_v; /* V相占空比，范围[0,1] */
    float duty_w; /* W相占空比，范围[0,1] */
} FocPhaseDuty_t;

/* 同一电角度的正弦和余弦；任一输出指针可为空。 */
void FOC_SinCos(float electrical_angle_rad, float *sin_value, float *cos_value);
/* 两相输入满足三相和为零；输入、输出保持相同物理单位。 */
void FOC_Clarke(float Ia, float Ib, float *pIalpha,float *pIbeta);
/* sinVal/cosVal对应同一转子电角度；不得传机械角。 */
void FOC_Park(float Ialpha, float Ibeta, float *pId, float *pIq, float sinVal, float cosVal);
void FOC_InvPark(float Id, float Iq, float *pIalpha, float *pIbeta, float sinVal, float cosVal);
/* 两相电流(A)经Clarke、Park转换到dq；输入须为有限值，角度须为已校准电角度(rad)。 */
bool FOC_CurrentToDQ(float iu_a, float iv_a, float electrical_angle_rad,
                     float *id_a, float *iq_a);
/* 电流单位A；sin/cos须对应同一电角度，供实时控制复用，不在内部计算三角函数。 */
bool FOC_CurrentToDQWithSinCos(float iu_a, float iv_a, float sin_theta,
                              float cos_theta, float *id_a, float *iq_a);

/* dq电压按母线电压归一化；输入须为有限值，输出三相占空比。 */
bool FocSvpwm_ComputeDuty(float electrical_angle_rad, float ud_norm, float uq_norm,
                          FocPhaseDuty_t *duty_out);
/* dq电压按母线电压归一化；复用与电流Park相同的sin/cos，输出三相占空比。 */
bool FocSvpwm_ComputeDutyWithSinCos(float ud_norm, float uq_norm, float sin_theta,
                                   float cos_theta, FocPhaseDuty_t *duty_out);

/*
*********************************************************************************************************
*   函 数 名: FOC_CyclicDiff
*   功能说明: 计算循环量从上一值到当前值的最短有符号变化量
*   形    参: current_value  - 当前循环量，单位自定义
*             previous_value - 上一次循环量，单位须与当前值相同
*             cycle_period   - 一个完整循环的范围，必须大于0；如角度为2π、编码器为16384
*   返 回 值: 范围[-cycle_period/2, cycle_period/2)内的差值；周期无效时返回0
*********************************************************************************************************
*/
float FOC_CyclicDiff(float current_value, float previous_value, float cycle_period);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
