#include "foc_math.h"
#include "arm_math.h"

/*
*********************************************************************************************************
*   函 数 名: FOC_sin_f32
*   功能说明: 计算正弦值
*   形    参: x - 输入角度，单位rad
*   返 回 值: sin(x)
*********************************************************************************************************
*/
float FOC_sin_f32(float x)
{
    return arm_sin_f32(x);
}
/*
*********************************************************************************************************
*   函 数 名: FOC_cos_f32   
*   功能说明: 计算余弦值
*   形    参: x - 输入角度，单位rad
*   返 回 值: cos(x)
*********************************************************************************************************
*/
float FOC_cos_f32(float x)
{
    return arm_cos_f32(x);
}

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

    arm_sin_cos_f32(electrical_angle_rad, &sin_theta, &cos_theta);
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
*   函 数 名: FOC_InvClarke
*   功能说明: 将αβ平面物理量还原为U/V相量；W相由三相和为零的约束确定
*   形    参: Ialpha - α轴输入，单位A或V
*             Ibeta  - β轴输入，单位A或V
*             pIa    - U相输出，可为NULL
*             pIb    - V相输出，可为NULL
*   返 回 值: 无
*********************************************************************************************************
*/
void FOC_InvClarke(float Ialpha, float Ibeta, float *pIa,float *pIb)
{
    if (pIa == NULL || pIb == NULL)
    {
        return;
    }

    arm_inv_clarke_f32(Ialpha, Ibeta, pIa, pIb);

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
