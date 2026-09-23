#include "foc_pid.h"
#include <stddef.h>

static float FocPid_Clamp(float value, float lower_limit, float upper_limit)
{
    if (value < lower_limit)
    {
        return lower_limit;
    }

    if (value > upper_limit)
    {
        return upper_limit;
    }

    return value;
}

/*
*********************************************************************************************************
*   函 数 名: FocPid_Init
*   功能说明: 用连续时间PID参数初始化一个固定周期的CMSIS-DSP PID控制器
*   形    参: pid        - 调用者长期保存的PID状态
*             kp         - 比例增益
*             ki_per_s   - 连续时间积分增益，单位输出/(误差·s)
*             kd_s       - 连续时间微分增益，单位输出·s/误差
*             period_s   - 固定控制周期，单位s，必须大于0
*             output_min - 输出下限
*             output_max - 输出上限，不小于output_min
*   返 回 值: true表示初始化成功；false表示空指针、周期非正或输出上下限颠倒
*             所有浮点参数必须由调用者保证为有限值
*********************************************************************************************************
*/
bool FocPid_Init(FocPid_t *pid, float kp, float ki_per_s, float kd_s,
                 float period_s, float output_min, float output_max)
{
    if ((pid == NULL) || (period_s <= 0.0f) || (output_min > output_max))
    {
        return false;
    }

    pid->cmsis.Kp = kp;
    pid->cmsis.Ki = ki_per_s * period_s;
    pid->cmsis.Kd = kd_s / period_s;
    pid->output_min = output_min;
    pid->output_max = output_max;

    /* CMSIS使用离散系数：Ki_d = Ki·dt，Kd_d = Kd/dt。 */
    arm_pid_init_f32(&pid->cmsis, 1U);
    return true;
}

/*
*********************************************************************************************************
*   函 数 名: FocPid_Reset
*   功能说明: 清除PID历史状态，防止旧积分直接作用于新的控制目标
*   形    参: pid - 已初始化的PID实例
*   返 回 值: 无；空指针时安全返回
*********************************************************************************************************
*/
void FocPid_Reset(FocPid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    arm_pid_reset_f32(&pid->cmsis);
}

/*
*********************************************************************************************************
*   函 数 名: FocPid_Update
*   功能说明: 计算并限幅PID输出，同时将CMSIS积分状态限制在输出范围
*   形    参: pid   - 已初始化的PID实例
*             error - 本控制周期误差，推荐为目标值减反馈值
*             error必须为有限值
*   返 回 值: 已限幅的控制输出；空指针时返回0
*********************************************************************************************************
*/
float FocPid_Update(FocPid_t *pid, float error)
{
    float output;

    if (pid == NULL)
    {
        return 0.0f;
    }

    output = arm_pid_f32(&pid->cmsis, error);
    output = FocPid_Clamp(output, pid->output_min, pid->output_max);
    /* CMSIS PID的state[2]保存上一拍输出；回写限幅值可抑制积分继续向外累积。 */
    pid->cmsis.state[2] = output;

    return output;
}
