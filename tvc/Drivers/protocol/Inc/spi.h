#ifndef SPI_H
#define SPI_H

#include "stm32f4xx_hal.h"

// NRF24L01 Register Addresses
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define OBSERVE_TX      0x08
#define RPD             0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17

/* EN_AA */
#define ENAA_P0         0x01
#define ENAA_P1         0x02
#define ENAA_P2         0x04
#define ENAA_P3         0x08
#define ENAA_P4         0x10
#define ENAA_P5         0x20

/* SETUP_RETR - Auto Retransmit Delay */
#define ARD_250         0x00
#define ARD_500         0x10
#define ARD_750         0x20
#define ARD_1000        0x30

/* SETUP_RETR - Auto Retransmit Count */
#define ARC_0           0x00
#define ARC_1           0x01
#define ARC_2           0x02
#define ARC_3           0x03
#define ARC_4           0x04
#define ARC_5           0x05
#define ARC_6           0x06
#define ARC_7           0x07
#define ARC_8           0x08
#define ARC_9           0x09
#define ARC_10          0x0A
#define ARC_11          0x0B
#define ARC_12          0x0C
#define ARC_13          0x0D
#define ARC_14          0x0E
#define ARC_15          0x0F

/* SETUP_AW - Address Width */
#define AW_3            0x01
#define AW_4            0x02
#define AW_5            0x03

/* Commands */
#define W_REGISTER      0b00100000
#define R_REGISTER      0b00000000
#define R_RX_PAYLOAD    0b01100001
#define W_TX_PAYLOAD    0b10100000
#define FLUSH_TX        0b11100001
#define FLUSH_RX        0b11100010
#define REUSE_TX_PL     0b11100011
#define R_RX_PL_WID     0b01100000
#define W_TX_PAYLOAD_NOACK 0b10110000
#define W_ACK_PAYLOAD   0b10101000
#define NOP_CMD         0b11111111

extern SPI_HandleTypeDef hspi1;

/* GPIO Control */
void SPI_CS_Low(void);
void SPI_CS_High(void);
void SPI_CE_Low(void);
void SPI_CE_High(void);

/* SPI Transfer */
uint8_t SPI_Transfer(uint8_t data);
void SPI_WriteReg(uint8_t reg, uint8_t value);
uint8_t SPI_ReadReg(uint8_t reg);
void SPI_WriteBuf(uint8_t reg, uint8_t *buf, uint8_t len);

void SPI_SetHandle(SPI_HandleTypeDef *hspi);
void SPI_InitTX(void);
void SPI_InitRX(void);

void SPI_Send(uint8_t *data, uint8_t len);
void SPI_Read(uint8_t *data, uint8_t len);
uint8_t SPI_DataAvailable(void);

void SPI_Init(void);

#endif /* SPI_H */
