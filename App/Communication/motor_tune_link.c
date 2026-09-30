#include "motor_tune_link.h"
#include "usart.h"
#include "cmsis_compiler.h"
#include <stdio.h>
#include <string.h>

/*
*********************************************************************************************************
*   模块说明: USART2简单文本命令收发
*   接收路径: HAL回调逐字节缓存 -> 主循环读取整行并解析 -> 应用层执行命令
*   发送路径: 主循环发送OK或低频遥测；本模块不访问FOC/PID/PWM
*********************************************************************************************************
*/

#define MOTOR_TUNE_LINE_SIZE 128U /* UART单行接收缓冲容量，单位byte */

static uint8_t s_rx_byte;                            /* HAL单字节接收缓冲 */
static char s_rx_line[MOTOR_TUNE_LINE_SIZE];         /* 尚未被主循环取走的一行命令 */
static volatile uint8_t s_rx_length;                 /* 当前行已缓存字节数 */
static volatile bool s_line_ready;                   /* 换行到达，整行可供主循环读取 */
static volatile bool s_drop_line;                    /* 缓冲溢出或UART错误后丢弃到下一个换行 */

/*
*********************************************************************************************************
*   函 数 名: MotorTuneLink_Init
*   功能说明: 清空软件接收状态并启动USART2单字节中断接收
*   形    参: 无
*   返 回 值: true表示HAL已接受首次接收请求
*********************************************************************************************************
*/
bool MotorTuneLink_Init(void)
{
    s_rx_length = 0U;
    s_line_ready = false;
    s_drop_line = false;
    return HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) == HAL_OK;
}

/*
*********************************************************************************************************
*   函 数 名: MotorTuneLink_ReadCommand
*   功能说明: 原子取出接收缓冲中的完整命令行，并在主循环解析为命令结构
*   形    参: command - 调用者提供的命令输出存储
*   返 回 值: 成功解析一条已知命令时返回true；无完整行或格式未知时返回false
*********************************************************************************************************
*/
bool MotorTuneLink_ReadCommand(MotorTuneCommand_t *command)
{
    char line[MOTOR_TUNE_LINE_SIZE];
    uint8_t length;
    uint32_t primask;
    float first;
    float second;
    unsigned long duration;
    char extra;

    if ((command == 0) || !s_line_ready) return false;

    /* 仅在复制并清空共享行缓冲期间屏蔽中断，然后恢复进入前的中断状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    length = s_rx_length;
    memcpy(line, s_rx_line, length);
    s_rx_length = 0U;
    s_line_ready = false;
    __set_PRIMASK(primask);

    if (length == 0U) return false;
    if (line[length - 1U] == '\n') line[--length] = '\0';
    if ((length > 0U) && (line[length - 1U] == '\r')) line[--length] = '\0';
    if (length == 0U) return false;

    if (strcmp(line, "PING") == 0)
    {
        command->type = MOTOR_TUNE_PING;
        return true;
    }
    if (strcmp(line, "STOP") == 0)
    {
        command->type = MOTOR_TUNE_STOP;
        return true;
    }
    if (sscanf(line, "SET_SPEED_PID,%f,%f%c", &first, &second, &extra) == 2)
    {
        command->type = MOTOR_TUNE_SET_SPEED_PID;
        command->value_a = first;
        command->value_b = second;
        return true;
    }
    if (sscanf(line, "RUN_SPEED_STEP,%f,%lu%c", &first, &duration, &extra) == 2)
    {
        command->type = MOTOR_TUNE_RUN_SPEED_STEP;
        command->value_a = first;
        command->duration_ms = (uint32_t)duration;
        return true;
    }
    return false;
}

/*
*********************************************************************************************************
*   函 数 名: MotorTuneLink_SendOk
*   功能说明: 对已被应用层接受的命令发送最简单的OK应答
*   形    参: 无
*   返 回 值: 无；阻塞发送，必须在主循环调用
*********************************************************************************************************
*/
void MotorTuneLink_SendOk(void)
{
    static const uint8_t response[] = "OK\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)response, sizeof(response) - 1U, 20U);
}

/*
*********************************************************************************************************
*   函 数 名: MotorTuneLink_SendData
*   功能说明: 发送速度与q轴电流遥测行
*   形    参: elapsed_ms - 试验经过时间，单位ms
*             speed_ref_rad_s/speed_rad_s - 机械速度目标/反馈，单位rad/s
*             iq_ref_a/iq_a - q轴电流目标/反馈，单位A
*   返 回 值: 无；遥测格式为DATA,elapsed,speed_ref,speed,iq_ref,iq
*********************************************************************************************************
*/
void MotorTuneLink_SendData(uint32_t elapsed_ms, float speed_ref_rad_s,
                            float speed_rad_s, float iq_ref_a, float iq_a)
{
    char line[96];
    int length = snprintf(line, sizeof(line), "DATA,%lu,%.3f,%.3f,%.4f,%.4f\r\n",
                          (unsigned long)elapsed_ms,
                          (double)speed_ref_rad_s, (double)speed_rad_s,
                          (double)iq_ref_a, (double)iq_a);
    if ((length > 0) && (length < (int)sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)length, 20U);
    }
}

/* ========================================================================== */
/* HAL回调集中区：仅缓存接收字节并重新启动接收，不解析命令或控制电机。 */
/* ========================================================================== */

/*
*********************************************************************************************************
*   函 数 名: HAL_UART_RxCpltCallback
*   功能说明: USART2每接收1字节追加到行缓冲；整行交给主循环处理，不在中断中解析
*   形    参: huart - HAL通知的UART实例，仅响应USART2
*   返 回 值: 无；回调结束前重新启动下一字节接收
*********************************************************************************************************
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) return;

    if (!s_drop_line && !s_line_ready)
    {
        if (s_rx_length < MOTOR_TUNE_LINE_SIZE)
        {
            s_rx_line[s_rx_length++] = (char)s_rx_byte;
            if (s_rx_byte == '\n') s_line_ready = true;
        }
        else
        {
            s_rx_length = 0U;
            s_drop_line = true; /* 超长命令整行丢弃，避免把残片误解析成新命令。 */
        }
    }
    else if (s_line_ready && (s_rx_byte != '\n'))
    {
        s_drop_line = true;
    }

    if (s_drop_line && (s_rx_byte == '\n'))
    {
        s_drop_line = false;
        s_rx_length = 0U;
    }
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/*
*********************************************************************************************************
*   函 数 名: HAL_UART_ErrorCallback
*   功能说明: USART2接收错误后清除未完成行，并丢弃输入直到下一换行
*   形    参: huart - HAL通知的UART实例，仅响应USART2
*   返 回 值: 无；重新启动单字节接收
*********************************************************************************************************
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2) return;
    s_rx_length = 0U;
    s_line_ready = false;
    s_drop_line = true;
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}
