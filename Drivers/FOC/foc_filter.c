#include "foc_filter.h"

/*
*********************************************************************************************************
*   函 数 名: FocFilter_LowPass
*   功能说明: 按固定权重计算一次一阶低通输出，不保存内部状态
*   形    参: input       - 当前输入，单位由调用者确定
*             last_output - 上一次滤波输出，单位与input相同
*             alpha       - 当前输入的权重，调用者保证范围[0, 1]
*   返 回 值: 本次滤波输出；首次采样建议由调用者直接作为初值
*********************************************************************************************************
*/
float FocFilter_LowPass(float input, float last_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * last_output;
}

/*
*********************************************************************************************************
*   函 数 名: FocFilter_KalmanInit
*   功能说明: 为一路标量信号初始化独立卡尔曼滤波状态
*   形    参: filter              - 调用者保存的滤波器状态
*             initial_measurement - 首个测量值，作为初始估计
*             measurement_noise - 测量噪声方差R，单位为输入单位的平方，调用者保证有限
*             process_noise     - 单次更新过程噪声方差Q，单位为输入单位的平方，调用者保证有限
*   返 回 值: true表示初始化成功；false表示状态指针为空或R/Q不为正
*********************************************************************************************************
*/
bool FocFilter_KalmanInit(FocKalmanFilter_t *filter, float initial_measurement,
                          float measurement_noise, float process_noise)
{
    if ((filter == 0) || !(measurement_noise > 0.0f) || !(process_noise > 0.0f))
    {
        return false;
    }

    filter->estimate = initial_measurement;
    filter->error_covariance = measurement_noise;
    filter->measurement_noise = measurement_noise;
    filter->process_noise = process_noise;
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: FocFilter_KalmanUpdate
*   功能说明: 按标量随机游走模型融合当前测量并更新估计状态
*   形    参: filter      - 已初始化且由当前信号独占的滤波器状态
*             measurement - 当前测量值，单位与初始测量值相同
*   返 回 值: 更新后的估计值，单位与measurement相同
*********************************************************************************************************
*/
float FocFilter_KalmanUpdate(FocKalmanFilter_t *filter, float measurement)
{
    float predicted_covariance;
    float kalman_gain;
    float measurement_residual;

    // 预测误差随过程噪声增加，再根据测量残差修正估计值。
    predicted_covariance = filter->error_covariance + filter->process_noise;
    kalman_gain = predicted_covariance /
                  (predicted_covariance + filter->measurement_noise);
    measurement_residual = measurement - filter->estimate;
    filter->estimate += kalman_gain * measurement_residual;
    filter->error_covariance = (1.0f - kalman_gain) * predicted_covariance;

    return filter->estimate;
}
