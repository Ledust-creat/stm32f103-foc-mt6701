#ifndef FOC_PORT_H
#define FOC_PORT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 平台适配契约：FOC算法层不得包含HAL头文件，也不得访问TIM/ADC/SPI寄存器。
 * 各BSP仅实现这些最小能力，例如BSP/FOC/foc_port_stm32f1_hal.c。
 */

typedef struct
{
    float phase_u_a;      /* U相电流，单位A；正方向必须与Clarke定义一致 */
    float phase_v_a;      /* V相电流，单位A；W相由-(U+V)重建 */
    uint32_t sample_tick; /* 采样完成的单调时间戳；单位由BSP文档定义 */
    bool valid;           /* ADC校准完成、未超时且未触发采样故障时为true */
} FOC_PortCurrentSample_t;

bool FOC_Port_ReadCurrent(FOC_PortCurrentSample_t *sample);
bool FOC_Port_ReadElectricalAngle(float *electrical_angle_rad);
bool FOC_Port_ApplyDuty(float duty_u, float duty_v, float duty_w);
void FOC_Port_DisableOutput(void);

#endif /* FOC_PORT_H */
