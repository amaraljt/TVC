#include "w25qxx.h"
#include "spi.h"
#include <stdint.h>
#include <string.h>

#define FLASH_CS_PORT   GPIOA
#define FLASH_CS_PIN    GPIO_PIN_11

#define WIP_BIT             0x01
#define WRITE_ENABLE        0x06
#define WRITE_DISABLE       0x04
#define READ_DATA           0x03
#define PAGE_PROGRAM        0x02
#define WRITE_STATUS_REG1   0x01
#define READ_STATUS_REG1    0x05

/* Page-program payload buffer. Static because W25_Write_Page returns before
   the DMA transfer finishes - a stack-local buffer would be gone by the time
   the hardware actually reads it. Sized for opcode + 3-byte address + one
   full page. */
static uint8_t s_tx_buf[4 + W25_PAGE_SIZE];

static volatile uint8_t s_write_pending = 0;
static volatile uint8_t s_read_pending = 0;

void W25_Write_Enable(void)
{
    SPI_CS_Low(FLASH_CS_PORT, FLASH_CS_PIN);
    SPI2_Send_Cmd(WRITE_ENABLE);
    SPI_CS_High(FLASH_CS_PORT, FLASH_CS_PIN);
}

uint8_t W25_Read_Status(void)
{
    uint8_t status;

    SPI_CS_Low(FLASH_CS_PORT, FLASH_CS_PIN);
    status = SPI2_Send(READ_STATUS_REG1, 0xFF);
    SPI_CS_High(FLASH_CS_PORT, FLASH_CS_PIN);

    return status;
}

void W25_Wait_Busy(void)
{
    uint8_t status;

    do {
        status = W25_Read_Status();
    } while (status & WIP_BIT);
}

/* Queues a DMA page program. addr/len must fit within a single 256-byte
   page - a program that crosses a page boundary wraps within the page on
   this chip instead of spilling into the next one, silently corrupting the
   start of the page, so that case is rejected rather than attempted. */
void W25_Write_Page(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > W25_PAGE_SIZE)
        return;
    if ((addr % W25_PAGE_SIZE) + len > W25_PAGE_SIZE)
        return;

    W25_Wait_Busy();
    W25_Write_Enable();

    s_tx_buf[0] = PAGE_PROGRAM;
    s_tx_buf[1] = (uint8_t)(addr >> 16);
    s_tx_buf[2] = (uint8_t)(addr >> 8);
    s_tx_buf[3] = (uint8_t)(addr);
    memcpy(&s_tx_buf[4], data, len);

    SPI_CS_Low(FLASH_CS_PORT, FLASH_CS_PIN);
    s_write_pending = 1;
    HAL_SPI_Transmit_DMA(&hspi2, s_tx_buf, 4 + len);
    /* CS_High happens in HAL_SPI_TxCpltCallback below, once the DMA has
       actually finished clocking this out */
}

/* Queues a DMA read starting at addr into buf. See the "must stay valid"
   note on this prototype in w25qxx.h before passing a stack buffer. */
void W25_Read_Flash(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4], cmd_rx[4];

    if (len == 0)
        return;

    cmd[0] = READ_DATA;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);

    SPI_CS_Low(FLASH_CS_PORT, FLASH_CS_PIN);

    HAL_SPI_TransmitReceive(&hspi2, cmd, cmd_rx, 4, HAL_MAX_DELAY);

    s_read_pending = 1;
    HAL_SPI_Receive_DMA(&hspi2, buf, len);
    /* CS_High happens in HAL_SPI_RxCpltCallback below. */
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2 && s_write_pending) {
        SPI_CS_High(FLASH_CS_PORT, FLASH_CS_PIN);
        s_write_pending = 0;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2 && s_read_pending) {
        SPI_CS_High(FLASH_CS_PORT, FLASH_CS_PIN);
        s_read_pending = 0;
    }
}
