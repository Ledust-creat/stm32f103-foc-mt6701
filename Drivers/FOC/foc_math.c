#include "foc_math.h"
#include "arm_math.h"
#include <math.h>
#include <stddef.h>

#define FOC_HALF_F           (0.5f)
#define FOC_SQRT3_BY_2_F     (0.86602540378443864676f)


/*
*********************************************************************************************************
*   函 数 名: FOC_SinCos
*   功能说明: 对同一电角度只计算一次正弦和余弦，供Park及反Park共用
*   形    参: electrical_angle_rad - 电角度，单位rad
*             sin_value            - 正弦输出，可为NULL
*             cos_value            - 余弦输出，可为NULL
*   返 回 值: 无；两个输出均为空时安全返回
*********************************************************************************************************
*/
void FOC_SinCos(float electrical_angle_rad, float *sin_value, float *cos_value)
{
    float sin_theta;
    float cos_theta;

    if ((sin_value == NULL) && (cos_value == NULL))
    {
        return;
    }

    arm_sin_cos_f32(FOC_RadToDeg(electrical_angle_rad), &sin_theta, &cos_theta);
    if (sin_value != NULL)
    {
        *sin_value = sin_theta;
    }
    if (cos_value != NULL)
    {
        *cos_value = cos_theta;
    }
}
/*
*********************************************************************************************************
*   函 数 名: FOC_Clarke
*   功能说明: 将U/V两相电流或电压投影到静止αβ平面；假设Iw=-(Iu+Iv)
*   形    参: Ia - U相物理量，单位A或V
*             Ib - V相物理量，单位A或V
*             pIalpha - α轴输出，单位A或V
*             pIbeta - β轴输出，单位A或V
*   返 回 值: 无；空指针时安全返回
*********************************************************************************************************
*/
void FOC_Clarke(float Ia, float Ib, float *pIalpha,float *pIbeta)
{
    if (pIalpha == NULL || pIbeta == NULL)
    {
        return;
    }

    arm_clarke_f32(Ia, Ib, pIalpha, pIbeta);

}

/*
*********************************************************************************************************
*   函 数 名: FOC_Park
*   功能说明: 以电角度旋转αβ坐标，使d轴与转子磁链对齐；不得传入机械角
*   形    参: Ialpha - α轴输入，单位A或V
*             Ibeta  - β轴输入，单位A或V
*             pId    - d轴输出，单位与输入相同
*             pIq    - q轴输出，单位与输入相同
*             sinVal - sin(θe)，θe为转子电角度，单位rad
*             cosVal - cos(θe)，正方向须与相序一致
*   返 回 值: 无；空指针时安全返回
*********************************************************************************************************
*/
void FOC_Park(float Ialpha, float Ibeta, float *pId, float *pIq, float sinVal, float cosVal)
{
    if (pId == NULL || pIq == NULL)
    {
        return;
    }
    arm_park_f32(Ialpha, Ibeta, pId, pIq, sinVal, cosVal);
}



/*
*********************************************************************************************************
*   函 数 名: FOC_InvPark
*   功能说明: 将dq电压指令旋转回静止αβ平面，供后续SVPWM调制使用
*   形    参: Id       - d轴输入，单位通常为V或归一化电压
*             Iq       - q轴输入，单位通常为V或归一化电压
*             pIalpha  - α轴输出，单位与输入相同
*             pIbeta   - β轴输出，单位与输入相同
*             sinVal   - sin(θe)，与FOC_Park使用同一电角度
*             cosVal   - cos(θe)，与FOC_Park使用同一电角度
*   返 回 值: 无；空指针时安全返回
*********************************************************************************************************
*/
void FOC_InvPark(float Id, float Iq, float *pIalpha, float *pIbeta, float sinVal, float cosVal)
{
    if (pIalpha == NULL || pIbeta == NULL)
    {
        return;
    }
    
    arm_inv_park_f32(Id, Iq, pIalpha, pIbeta, sinVal, cosVal);
}

/*
*********************************************************************************************************
*   函 数 名: FOC_CurrentToDQ
*   功能说明: 将满足三相电流和为零的U/V相电流变换到转子dq坐标
*   形    参: iu_a/iv_a            - 两相电流，单位A，相序和正方向由调用者确认
*             electrical_angle_rad - 已校准的转子电角度，单位rad
*             id_a/iq_a           - dq轴电流输出，单位A
*             所有浮点输入须由调用者保证为有限值
*   返 回 值: 输出指针有效时为true；指针错误时返回false且不写输出
*********************************************************************************************************
*/
bool FOC_CurrentToDQ(float iu_a, float iv_a, float electrical_angle_rad,
                     float *id_a, float *iq_a)
{
    float alpha_a;
    float beta_a;
    float sin_theta;
    float cos_theta;

    if ((id_a == NULL) || (iq_a == NULL) || (id_a == iq_a))
    {
        return false;
    }

    FOC_Clarke(iu_a, iv_a, &alpha_a, &beta_a);
    FOC_SinCos(electrical_angle_rad, &sin_theta, &cos_theta);
    FOC_Park(alpha_a, beta_a, id_a, iq_a, sin_theta, cos_theta);
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: FocSvpwm_ComputeDuty
*   功能说明: 反Park后还原三相电压，以零序注入生成居中的三相占空比
*   形    参: electrical_angle_rad - 电角度，单位rad
*             ud_norm/uq_norm     - dq电压，按母线电压归一化
*             duty_out            - 调用者提供的输出存储
*             所有浮点输入须由调用者保证为有限值
*   返 回 值: 输出指针有效时为true；指针错误时返回false且输出不变
*********************************************************************************************************
*/
bool FocSvpwm_ComputeDuty(float electrical_angle_rad, float ud_norm, float uq_norm,
                          FocPhaseDuty_t *duty_out)
{
    float sin_theta, cos_theta, alpha, beta;
    float u, v, w, max_u, min_u, span, common;

    if (duty_out == NULL)
    {
        return false;
    }

    FOC_SinCos(electrical_angle_rad, &sin_theta, &cos_theta);
    FOC_InvPark(ud_norm, uq_norm, &alpha, &beta, sin_theta, cos_theta);

    // 逆Clarke的三相相电压加入同一个零序偏置，不改变线电压。
    u = alpha;
    v = -FOC_HALF_F * alpha + FOC_SQRT3_BY_2_F * beta;
    w = -FOC_HALF_F * alpha - FOC_SQRT3_BY_2_F * beta;
    max_u = fmaxf(u, fmaxf(v, w));
    min_u = fminf(u, fminf(v, w));
    span = max_u - min_u;

    if (span > 1.0f)
    {
        // 电压矢量整体缩放，避免逐轴钳位造成方向偏移。
        u /= span;
        v /= span;
        w /= span;
        max_u /= span;
        min_u /= span;
    }

    common = FOC_HALF_F * (max_u + min_u);
    duty_out->duty_u = FOC_ClampF32(FOC_HALF_F + u - common, 0.0f, 1.0f);
    duty_out->duty_v = FOC_ClampF32(FOC_HALF_F + v - common, 0.0f, 1.0f);
    duty_out->duty_w = FOC_ClampF32(FOC_HALF_F + w - common, 0.0f, 1.0f);
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: FOC_CyclicDiff
*   功能说明: 将相邻循环量之差折返到[-半周期, 半周期)，供角度累计或位置误差计算使用
*             当两次采样真实变化超过半周期时无法判定实际旋转方向
*   形    参: current_value  - 当前值，单位自定义
*             previous_value - 上一次值，单位与当前值相同
*             cycle_period   - 周期范围，必须大于0
*   返 回 值: 最短有符号差值；周期无效时返回0
*********************************************************************************************************
*/
float FOC_CyclicDiff(float current_value, float previous_value, float cycle_period)
{
    float difference;
    float half_period;

    if (cycle_period <= 0.0f)
    {
        return 0.0f;
    }

    difference = current_value - previous_value;
    half_period = cycle_period * 0.5f;

    while (difference >= half_period)
    {
        difference -= cycle_period;
    }

    while (difference < -half_period)
    {
        difference += cycle_period;
    }

    return difference;
}
