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

#define ODR_416_2G         0x60
#define ODR_416_250_DPS    0x60

#define STATUS_XLDA   0x01
#define STATUS_GDA    0x02

#define NUM_SAMPLES    200
#define GYRO_SENSITIVITY     8.75f /* 250dps */
#define ACCEL_SENSITIVITY    0.061f /* +-2g */
#define DEG_TO_RAD    0.017453293f

GyroBias g_gyro_bias = {0};
GyroRps g_gyro_rps = {0};
AccelGs g_accel_gs = {0};

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

uint8_t IMU_Get_Gyro_Out(void)
{
    uint8_t buf[6];
    uint8_t status = IMU_Read_Status();
    if (!(status & STATUS_GDA)) {
        UART_Print("GYRO NOT READY: STATUS=0x%02X\r\n", status);
        return 1;
    }

    IMU_Read_Burst(R_GYRO_OUT_X, buf);

    int16_t g_lsb_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t g_lsb_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t g_lsb_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Convert LSB to rad/s
    g_gyro_rps.gyro_x = ((float)(g_lsb_x * GYRO_SENSITIVITY) / 1000) * DEG_TO_RAD;
    g_gyro_rps.gyro_y = ((float)(g_lsb_y * GYRO_SENSITIVITY) / 1000) * DEG_TO_RAD;
    g_gyro_rps.gyro_z = ((float)(g_lsb_z * GYRO_SENSITIVITY) / 1000) * DEG_TO_RAD;

    return 0;
}

void IMU_Callibrate_Gyro(void)
{
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    for (int i = 0; i < NUM_SAMPLES; ) {
        if (IMU_Get_Gyro_Out() != 0) // rad/s data; skip and retry if not ready yet
            continue;

        gyro_sum_x += g_gyro_rps.gyro_x;
        gyro_sum_y += g_gyro_rps.gyro_y;
        gyro_sum_z += g_gyro_rps.gyro_z;
        i++;
    }

    g_gyro_bias.gyro_x = gyro_sum_x / NUM_SAMPLES;
    g_gyro_bias.gyro_y = gyro_sum_y / NUM_SAMPLES;
    g_gyro_bias.gyro_z = gyro_sum_z / NUM_SAMPLES;
}

uint8_t IMU_Get_Accel_Out(void)
{
    uint8_t buf[6];
    uint8_t status = IMU_Read_Status();
    if (!(status & STATUS_XLDA)) {
        UART_Print("ACCEL NOT READY: STATUS=0x%02X\r\n", status);
        return 1;
    }

    IMU_Read_Burst(R_ACCEL_OUT_X, buf);

    int16_t a_lsb_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t a_lsb_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t a_lsb_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Convert LSB to gs
    g_accel_gs.accel_x = ((float)(a_lsb_x * ACCEL_SENSITIVITY) / 1000);
    g_accel_gs.accel_y = ((float)(a_lsb_y * ACCEL_SENSITIVITY) / 1000);
    g_accel_gs.accel_z = ((float)(a_lsb_z * ACCEL_SENSITIVITY) / 1000);

    return 0;
}

void IMU_Print(void)
{
    UART_Print("Gyro X: %.4f  Y: %.4f  Z: %.4f\r\n",
            g_gyro_rps.gyro_x - g_gyro_bias.gyro_x,
            g_gyro_rps.gyro_y - g_gyro_bias.gyro_y,
            g_gyro_rps.gyro_z - g_gyro_bias.gyro_z);

    UART_Print("Accel X: %.4f  Y: %.4f  Z: %.4f\r\n",
            g_accel_gs.accel_x,
            g_accel_gs.accel_y,
            g_accel_gs.accel_z);
}

void IMU_Init(void)
{
    if (IMU_Whoami() != 0)
        return;

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    SPI_Send(R_ACCEL_CFG, ODR_416_2G, SPI_WRITE);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    SPI_Send(R_GYRO_CFG, ODR_416_2G, SPI_WRITE);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t xl_readback = SPI_Send(R_ACCEL_CFG, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t g_readback = SPI_Send(R_GYRO_CFG, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    if (xl_readback != ODR_416_2G)
        UART_Print("CTRL1_XL MISMATCH: wrote 0x%02X, read 0x%02X\r\n", ODR_416_2G, xl_readback);
    if (g_readback != ODR_416_2G)
        UART_Print("CTRL2_G MISMATCH: wrote 0x%02X, read 0x%02X\r\n", ODR_416_2G, g_readback);

    IMU_Callibrate_Gyro();
}