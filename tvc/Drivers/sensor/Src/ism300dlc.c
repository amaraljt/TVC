#include "ism300dlc.h"
#include "spi.h"
#include "uart.h"
#include <stdint.h>

#define IMU_CS_PORT   GPIOA
#define IMU_CS_PIN    GPIO_PIN_8

#define SPI_READ    1
#define SPI_WRITE   0

#define R_WHOAMI              0x0F
#define R_STATUS              0x1E

#define R_ACCEL_CFG   0x10
#define R_GYRO_CFG    0x11

#define R_GYRO_OUT_X    0x22
#define R_ACCEL_OUT_X   0x28

#define ODR_416    0x60

#define STATUS_XLDA   0x01
#define STATUS_GDA    0x02

GyroPosition g_gyro_pos = {0};
AccelPosition g_accel_pos = {0};

uint8_t IMU_Whoami(void)
{
    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t id = SPI_Send(R_WHOAMI, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    if (id != 0x6B) {
        UART_Print("WHOAMI FAILED: 0x%02X\r\n", id);
        return 1;
    }
    return 0;
}

void IMU_Init(void)
{
    if (IMU_Whoami() != 0)
        return;

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    SPI_Send(R_ACCEL_CFG, ODR_416, SPI_WRITE);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    SPI_Send(R_GYRO_CFG, ODR_416, SPI_WRITE);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t xl_readback = SPI_Send(R_ACCEL_CFG, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t g_readback = SPI_Send(R_GYRO_CFG, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    if (xl_readback != ODR_416)
        UART_Print("CTRL1_XL MISMATCH: wrote 0x%02X, read 0x%02X\r\n", ODR_416, xl_readback);
    if (g_readback != ODR_416)
        UART_Print("CTRL2_G MISMATCH: wrote 0x%02X, read 0x%02X\r\n", ODR_416, g_readback);
}

void IMU_Read_Burst(uint8_t reg, uint8_t *buf)
{
    SPI_BurstRead(IMU_CS_PORT, IMU_CS_PIN, reg, buf, 6);
}

uint8_t IMU_Read_Status(void)
{
    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t status = SPI_Send(R_STATUS, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);
    return status;
}

void IMU_Get_Gyro_Out(void)
{
    uint8_t buf[6];
    uint8_t status = IMU_Read_Status();
    if (!(status & STATUS_GDA)) {
        UART_Print("GYRO NOT READY: STATUS=0x%02X\r\n", status);
        return;
    }

    IMU_Read_Burst(R_GYRO_OUT_X, buf);

    g_gyro_pos.gyro_x = (int16_t)((buf[1] << 8) | buf[0]);
    g_gyro_pos.gyro_y = (int16_t)((buf[3] << 8) | buf[2]);
    g_gyro_pos.gyro_z = (int16_t)((buf[5] << 8) | buf[4]);

    UART_Print("Gyro X: %d  Y: %d  Z: %d\r\n",
               g_gyro_pos.gyro_x,
               g_gyro_pos.gyro_y,
               g_gyro_pos.gyro_z);
}

void IMU_Get_Accel_Out(void)
{
    uint8_t buf[6];
    uint8_t status = IMU_Read_Status();
    if (!(status & STATUS_XLDA)) {
        UART_Print("ACCEL NOT READY: STATUS=0x%02X\r\n", status);
        return;
    }

    IMU_Read_Burst(R_ACCEL_OUT_X, buf);

    g_accel_pos.accel_x = (int16_t)((buf[1] << 8) | buf[0]);
    g_accel_pos.accel_y = (int16_t)((buf[3] << 8) | buf[2]);
    g_accel_pos.accel_z = (int16_t)((buf[5] << 8) | buf[4]);

    UART_Print("Accel X: %d  Y: %d  Z: %d\r\n",
               g_accel_pos.accel_x,
               g_accel_pos.accel_y,
               g_accel_pos.accel_z);
}
