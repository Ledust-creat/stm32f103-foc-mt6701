#ifndef MOTOR_TUNE_LINK_H
#define MOTOR_TUNE_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_TUNE_PING = 0,          /* 链路连通查询 */
    MOTOR_TUNE_SET_SPEED_PID,     /* 设置速度PI的Kp、Ki */
    MOTOR_TUNE_RUN_SPEED_STEP,    /* 启动一次限时速度阶跃 */
    MOTOR_TUNE_STOP               /* 立即关闭控制和PWM */
} MotorTuneCommandType_t;

typedef struct
{
    MotorTuneCommandType_t type;  /* 命令类型 */
    float value_a;                /* PID命令为Kp、阶跃命令为目标速度rad/s */
    float value_b;                /* PID命令为Ki，其他命令不使用 */
    uint32_t duration_ms;         /* 阶跃持续时间，单位ms */
} MotorTuneCommand_t;

/*
*********************************************************************************************************
*   函 数 名: MotorTuneLink_Init
*   功能说明: 启动USART2单字节中断接收；不初始化USART2外设
*   形    参: 无
*   返 回 值: true表示首次接收已启动
*********************************************************************************************************
*/
bool MotorTuneLink_Init(void);
/* 主循环取出并解析一条CRLF命令；无完整命令或命令格式不识别时返回false。 */
bool MotorTuneLink_ReadCommand(MotorTuneCommand_t *command);
/* 阻塞发送OK应答；仅对已接受命令调用，须在主循环使用。 */
void MotorTuneLink_SendOk(void);
/* 发送速度/电流遥测；速度单位rad/s，电流单位A，须在主循环低频调用。 */
void MotorTuneLink_SendData(uint32_t elapsed_ms, float speed_ref_rad_s,
                            float speed_rad_s, float iq_ref_a, float iq_a);

#endif /* MOTOR_TUNE_LINK_H */
