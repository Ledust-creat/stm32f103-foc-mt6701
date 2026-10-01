#include "mt6701.h"

#define MT6701_COUNTS_PER_REVOLUTION    (16384.0f)             /* 14bit单圈编码器计数 */
#define MT6701_TWO_PI_F                 (6.283185307179586f)   /* 一圈机械角，单位rad */

/*
*********************************************************************************************************
*   模块变量
*********************************************************************************************************
*/

static volatile float s_mechanical_angle_rad; /* DMA完成ISR写入；单圈机械角，单位rad，范围[0, 2π) */
static volatile bool s_angle_valid;           /* 收到至少一帧非空DMA数据后置true；不代表帧校验通过 */
static volatile uint32_t s_frame_count;        /* 每收到一帧递增，供速度估算识别新数据 */
uint8_t calculate_crc(uint32_t data)
{
  uint8_t crc = 0;
  uint32_t polynomial = 0x43; // (X^6 + X + 1)的二进制值

  // (D[13:0] 和 Mg[3:0])
  for (int i = 17; i >= 0; i--)
  {
    uint8_t bit = (data >> i) & 1;
    crc <<= 1;
    if ((crc >> 6) ^ bit)
      crc ^= polynomial;
    crc &= 0x3F;
  }
  return crc;
}
/*
*********************************************************************************************************
*   函 数 名: MT6701_Init
*   功能说明: 清除编码器软件状态；首次DMA读取由硬件层随后启动
*   返 回 值: 无
*********************************************************************************************************
*/
void MT6701_Init(void)
{
    s_mechanical_angle_rad = 0.0f;
    s_angle_valid = false;
    s_frame_count = 0U;
}

/*
*********************************************************************************************************
*   函 数 名: MT6701_OnSpiFrameISR
*   功能说明: 在SPI DMA完成中断中提取MT6701固定3字节帧的单圈机械角
*   形    参: frame - 本次DMA接收帧；前14bit为绝对机械角原始计数
*   返 回 值: 无；空指针帧丢弃；当前未校验帧状态位或CRC
*********************************************************************************************************
*/
void MT6701_OnSpiFrameISR(const uint8_t frame[3])
{
    uint16_t raw_angle;

    if (frame == 0)
    {
        return;
    }
    uint8_t crc_raw = frame[2] & ((1 << 6) - 1);
    uint32_t crc_data = (frame[0] << 16 | frame[1] << 8 | frame[2]) >> 6;
    if (calculate_crc(crc_data) != crc_raw)
    {

      return;
    }
    raw_angle = (uint16_t)(((uint16_t)frame[0] << 6U) | ((uint16_t)frame[1] >> 2U)); // 取帧前14bit角度字段
    s_mechanical_angle_rad = ((float)raw_angle * MT6701_TWO_PI_F) / MT6701_COUNTS_PER_REVOLUTION;
    s_frame_count++;
    s_angle_valid = true;
}

/*
*********************************************************************************************************
*   函 数 名: MT6701_GetMechanicalAngleRad
*   功能说明: 复制最新单圈机械角快照，供速度估算和电角度计算使用
*   形    参: angle_rad - 调用者提供的输出存储，单位rad
*   返 回 值: true表示已收到首帧；false表示输出指针无效或尚未完成首帧读取
*********************************************************************************************************
*/
bool MT6701_GetMechanicalAngleRad(float *angle_rad)
{
    if (angle_rad == 0)
    {
        return false;
    }

    *angle_rad = s_mechanical_angle_rad;
    return s_angle_valid;
}

/*
*********************************************************************************************************
*   函 数 名: MT6701_GetSample
*   功能说明: 复制最新单圈角度和帧序号；
*********************************************************************************************************
*/
bool MT6701_GetSample(float *angle_rad, uint32_t *frame_count)
{
    if ((angle_rad == 0) || (frame_count == 0) || !s_angle_valid)
    {
        return false;
    }

    *angle_rad = s_mechanical_angle_rad;
    *frame_count = s_frame_count;
    return true;
}
