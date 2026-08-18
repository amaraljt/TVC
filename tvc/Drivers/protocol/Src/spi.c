#include "spi.h"
#include "main.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>

#define READ_MASK    0x80
#define WRITE_MASK   0x7F

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

void SPI_CS_Low(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void SPI_CS_High(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

uint8_t SPI1_Send(uint8_t reg, uint8_t data, uint8_t read)
{
    uint8_t rx;

    if (read)
        reg |= READ_MASK;
    else
        reg &= WRITE_MASK;

    HAL_SPI_TransmitReceive(&hspi1, &reg, &rx, 1, HAL_MAX_DELAY);   // address phase
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);  // data phase

    return rx;
}

uint8_t SPI2_Send(uint8_t reg, uint8_t data)
{
    uint8_t rx;

    HAL_SPI_TransmitReceive(&hspi2, &reg, &rx, 1, HAL_MAX_DELAY);   // address phase
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, HAL_MAX_DELAY);   // data phase

    return rx;
}

void SPI2_Send_Cmd(uint8_t opcode)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi2, &opcode, &rx, 1, HAL_MAX_DELAY);
}

void SPI_BurstRead(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t dummy = 0xFF;
    uint8_t rx;

    reg |= READ_MASK;

    SPI_CS_Low(port, pin);
    HAL_SPI_TransmitReceive(&hspi1, &reg, &rx, 1, HAL_MAX_DELAY);  // address phase
    for (uint8_t i = 0; i < len; i++)
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &buf[i], 1, HAL_MAX_DELAY);
    SPI_CS_High(port, pin);
}

#if 0
/* DMA|SPI helper functions for W25Q32 */
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);

void SPI_DMA_Read()
{
    uint8_t buf;
    buf = HAL_SPI_Receive_DMA(*hspi, *pData, size);
    return buf;
}
#endif

void SPI_Init(void)
{
    /* SPI1 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    /* SPI2 */
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }

}
