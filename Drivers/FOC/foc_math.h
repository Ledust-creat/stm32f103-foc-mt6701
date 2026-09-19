#ifndef FOC_MATH_H
#define FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif
#define FOC_PI_F                       (3.14159265358979323846f)
#define FOC_RAD_PER_DEG_F              (FOC_PI_F / 180.0f)
#define FOC_DEG_PER_RAD_F              (180.0f / FOC_PI_F)

static inline float FOC_DegToRad(float degree)
{
    return degree * FOC_RAD_PER_DEG_F;
}

static inline float FOC_RadToDeg(float radian)
{
    return radian * FOC_DEG_PER_RAD_F;
}

static inline float FOC_ClampF32(float value, float lower_limit, float upper_limit)
{
    return (value < lower_limit) ? lower_limit : ((value > upper_limit) ? upper_limit : value);
}

/*
*********************************************************************************************************
*   函数声明
*********************************************************************************************************
*/

float FOC_sin_f32(float x);
float FOC_cos_f32(float x);
void FOC_SinCos(float electrical_angle_rad, float *sin_value, float *cos_value);
void FOC_Clarke(float Ia, float Ib, float *pIalpha,float *pIbeta);
void FOC_InvClarke(float Ialpha, float Ibeta, float *pIa,float *pIb);
void FOC_Park(float Ialpha, float Ibeta, float *pId, float *pIq, float sinVal, float cosVal);
void FOC_InvPark(float Id, float Iq, float *pIalpha, float *pIbeta, float sinVal, float cosVal);

#ifdef __cplusplus
}
#endif

#endif /* FOC_MATH_H */
