#include "spi.h"
#include "main.h"

#define SPI_CS_PORT  GPIOB
#define SPI_CS_PIN   GPIO_PIN_1
#define SPI_CE_PORT  GPIOB
#define SPI_CE_PIN   GPIO_PIN_0
#define RF_CHANNEL   76

static SPI_HandleTypeDef *g_spi;

void SPI_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
}

void SPI_CS_High(void)
{
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
}

void SPI_CE_Low(void)
{
    HAL_GPIO_WritePin(SPI_CE_PORT, SPI_CE_PIN, GPIO_PIN_RESET);
}

void SPI_CE_High(void)
{
    HAL_GPIO_WritePin(SPI_CE_PORT, SPI_CE_PIN, GPIO_PIN_SET);
}

uint8_t SPI_Transfer(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(g_spi, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

void SPI_WriteReg(uint8_t reg, uint8_t value)
{
    SPI_CS_Low();
    SPI_Transfer(W_REGISTER | reg);
    SPI_Transfer(value);
    SPI_CS_High();
}

uint8_t SPI_ReadReg(uint8_t reg)
{
    uint8_t value;

    SPI_CS_Low();
    SPI_Transfer(reg);
    value = SPI_Transfer(0xFF);
    SPI_CS_High();

    return value;
}

void SPI_WriteBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI_CS_Low();
    SPI_Transfer(W_REGISTER | reg);
    for (int i = 0; i < len; i++)
        SPI_Transfer(buf[i]);
    SPI_CS_High();
}

void SPI_SetHandle(SPI_HandleTypeDef *hspi)
{
    g_spi = hspi;
}

void SPI_InitTX(void)
{
    SPI_CE_Low();
    HAL_Delay(5);

    SPI_WriteReg(SETUP_AW, AW_5);
    SPI_WriteReg(RF_CH, RF_CHANNEL);
    SPI_WriteReg(RF_SETUP, 0x0E);
    SPI_WriteReg(EN_AA, ENAA_P0);
    SPI_WriteReg(SETUP_RETR, ARD_500 | ARC_10);

    uint8_t addr[5] = {0x78, 0x78, 0x78, 0x78, 0x78};
    SPI_WriteBuf(TX_ADDR, addr, 5);
    SPI_WriteBuf(RX_ADDR_P0, addr, 5);

    SPI_WriteReg(RX_PW_P0, 2);
    SPI_WriteReg(STATUS, 0x70);
    SPI_WriteReg(CONFIG, 0x0E);
    HAL_Delay(2);
}

void SPI_InitRX(void)
{
    SPI_CE_Low();
    HAL_Delay(5);

    SPI_WriteReg(SETUP_AW, AW_5);
    SPI_WriteReg(RF_CH, RF_CHANNEL);
    SPI_WriteReg(RF_SETUP, 0x0E);
    SPI_WriteReg(EN_AA, ENAA_P0);
    SPI_WriteReg(SETUP_RETR, 0x1A);

    uint8_t addr[5] = {0x78, 0x78, 0x78, 0x78, 0x78};
    SPI_WriteBuf(RX_ADDR_P0, addr, 5);

    SPI_WriteReg(RX_PW_P0, 2);
    SPI_WriteReg(STATUS, 0x70);
    SPI_WriteReg(CONFIG, 0x0F);
    HAL_Delay(2);

    SPI_CE_High();
}

void SPI_Send(uint8_t *data, uint8_t len)
{
    SPI_CE_Low();

    SPI_CS_Low();
    SPI_Transfer(0xA0);  // W_TX_PAYLOAD
    for (int i = 0; i < len; i++)
        SPI_Transfer(data[i]);
    SPI_CS_High();

    SPI_CE_High();
    HAL_Delay(1);
    SPI_CE_Low();

    while (!(SPI_ReadReg(STATUS) & 0x30));
    SPI_WriteReg(STATUS, 0x30);
}

void SPI_Read(uint8_t *data, uint8_t len)
{
    SPI_CS_Low();
    SPI_Transfer(0x61);  // R_RX_PAYLOAD
    for (int i = 0; i < len; i++)
        data[i] = SPI_Transfer(0xFF);
    SPI_CS_High();

    SPI_WriteReg(STATUS, 0x40);
}

uint8_t SPI_DataAvailable(void)
{
    return (SPI_ReadReg(STATUS) & 0x40);
}

void SPI_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}
