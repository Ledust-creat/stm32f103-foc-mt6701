#ifndef FOC_FILTER_H
#define FOC_FILTER_H

#include <stdbool.h>

typedef struct
{
    float estimate;             /* 滤波估计值，单位与输入信号相同 */
    float error_covariance;     /* 估计误差方差，单位为输入单位的平方 */
    float measurement_noise;    /* 测量噪声方差R，单位为输入单位的平方 */
    float process_noise;        /* 单次更新过程噪声方差Q，单位为输入单位的平方 */
} FocKalmanFilter_t;

/* 一阶低通；alpha为本次输入权重，范围[0, 1]，上次输出由调用者保存。 */
float FocFilter_LowPass(float input, float last_output, float alpha);

/* 初始化独立状态；调用者保证R/Q为正有限值，初始估计取首个测量值。 */
bool FocFilter_KalmanInit(FocKalmanFilter_t *filter, float initial_measurement,
                          float measurement_noise, float process_noise);
/* 更新一次标量随机游走卡尔曼估计；measurement单位与estimate相同。 */
float FocFilter_KalmanUpdate(FocKalmanFilter_t *filter, float measurement);

#endif /* FOC_FILTER_H */
