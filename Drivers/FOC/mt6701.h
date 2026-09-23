#ifndef MT6701_H
#define MT6701_H

#include <stdbool.h>
#include <stdint.h>

/* 清除角度和有效标志；SPI、DMA和CSN由MotorHw管理。 */
void MT6701_Init(void);

/* SPI DMA完成回调传入3字节帧；提取前14位角度，发布[0, 2π)机械角。 */
void MT6701_OnSpiFrameISR(const uint8_t frame[3]);

/* 读取最新单圈机械角(rad)；首帧未收到或指针无效时返回false。 */
bool MT6701_GetMechanicalAngleRad(float *angle_rad);
/* 取得角度及已完成帧计数；供周期速度估算识别停更。 */
bool MT6701_GetSample(float *angle_rad, uint32_t *frame_count);
#endif /* MT6701_H */
