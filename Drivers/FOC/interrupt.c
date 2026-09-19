#include "main.h"
uint8_t spi1_rx_data[3];
void Init(void)
{
  HAL_SPI_TransmitReceive_DMA(&hspi1, spi1_rx_data, spi1_rx_data, 3);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    // int angle_raw = (mt6701_rx_data[1] >> 2) | (mt6701_rx_data[0] << 6);
    // encoder_angle = 2 * 3.1415926 * angle_raw / ((1 << 14) - 1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive_DMA(&hspi1, spi1_rx_data, spi1_rx_data, 3);
    return;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  printf("HAL_SPI_ErrorCallback:%d\n", hspi->ErrorCode);
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, spi1_rx_data, spi1_rx_data, 3);
  }
  return;
}
