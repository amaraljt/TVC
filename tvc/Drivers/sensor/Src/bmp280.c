#include "bmp280.h"
#include "spi.h"
#include "uart.h"
#include <stdint.h>

#define ALT_CS_PORT   GPIOC
#define ALT_CS_PIN    GPIO_PIN_9

#define SPI_READ    1
#define SPI_WRITE   0

#define R_ID          0xD0
#define R_CALIB_START 0x88

#define R_OUT_CFG     0xF4
#define R_PRESS_OUT   0xF7

#define R_CONFIG      0xF5
#define R_STATUS      0xF3

#define STATUS_MEASURING  0x08
#define STATUS_IM_UPDATE  0x01

/* osrs_t = x2, osrs_p = x16, mode = normal */
#define OSRS_T_X2     (0x02 << 5)
#define OSRS_P_X16    (0x05 << 2)
#define MODE_NORMAL   0x03
#define CTRL_MEAS_CFG (OSRS_T_X2 | OSRS_P_X16 | MODE_NORMAL)

/* t_sb = 0.5ms, filter = x16, spi3w_en = 0 */
#define CONFIG_CFG    ((0x00 << 5) | (0x04 << 2))

BaroData g_baro_data = {0};
static BaroCalib s_calib = {0};
static int32_t s_t_fine = 0;

uint8_t BMP_Whoami(void)
{
    SPI_CS_Low(ALT_CS_PORT, ALT_CS_PIN);
    uint8_t id = SPI_Send(R_ID, 0xFF, SPI_READ);
    SPI_CS_High(ALT_CS_PORT, ALT_CS_PIN);

    if (id != 0x58) {
        UART_Print("BMP280 WHOAMI FAILED: 0x%02X\r\n", id);
        return 1;
    }
    return 0;
}

uint8_t BMP_Read_Status(void)
{
    SPI_CS_Low(ALT_CS_PORT, ALT_CS_PIN);
    uint8_t status = SPI_Send(R_STATUS, 0xFF, SPI_READ);
    SPI_CS_High(ALT_CS_PORT, ALT_CS_PIN);
    return status;
}

static void BMP_Read_Calib(void)
{
    uint8_t buf[24];
    SPI_BurstRead(ALT_CS_PORT, ALT_CS_PIN, R_CALIB_START, buf, 24);

    s_calib.dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
    s_calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    s_calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);

    s_calib.dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
    s_calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
    s_calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    s_calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    s_calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    s_calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    s_calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    s_calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    s_calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);
}

/* Bosch datasheet 3.11.3 float compensation; must be called before pressure */
static float BMP_Compensate_Temp(int32_t adc_T)
{
    float var1, var2, T;

    var1 = (((float)adc_T) / 16384.0f - ((float)s_calib.dig_T1) / 1024.0f) * ((float)s_calib.dig_T2);
    var2 = ((((float)adc_T) / 131072.0f - ((float)s_calib.dig_T1) / 8192.0f) *
            (((float)adc_T) / 131072.0f - ((float)s_calib.dig_T1) / 8192.0f)) * ((float)s_calib.dig_T3);

    s_t_fine = (int32_t)(var1 + var2);
    T = (var1 + var2) / 5120.0f;
    return T;
}

static float BMP_Compensate_Press(int32_t adc_P)
{
    float var1, var2, p;

    var1 = ((float)s_t_fine / 2.0f) - 64000.0f;
    var2 = var1 * var1 * ((float)s_calib.dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)s_calib.dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)s_calib.dig_P4) * 65536.0f);
    var1 = (((float)s_calib.dig_P3) * var1 * var1 / 524288.0f + ((float)s_calib.dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)s_calib.dig_P1);

    if (var1 == 0.0f)
        return 0.0f; // avoid divide-by-zero

    p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)s_calib.dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)s_calib.dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)s_calib.dig_P7)) / 16.0f;

    return p;
}

uint8_t BMP_Init(void)
{
    if (BMP_Whoami() != 0)
        return 1;

    uint32_t timeout = 100000;
    while ((BMP_Read_Status() & STATUS_IM_UPDATE) && --timeout)
        ;
    if (timeout == 0)
        UART_Print("BMP280 IM_UPDATE TIMEOUT\r\n");

    BMP_Read_Calib();

    SPI_CS_Low(ALT_CS_PORT, ALT_CS_PIN);
    SPI_Send(R_CONFIG, CONFIG_CFG, SPI_WRITE);
    SPI_CS_High(ALT_CS_PORT, ALT_CS_PIN);

    SPI_CS_Low(ALT_CS_PORT, ALT_CS_PIN);
    SPI_Send(R_OUT_CFG, CTRL_MEAS_CFG, SPI_WRITE);
    SPI_CS_High(ALT_CS_PORT, ALT_CS_PIN);

    SPI_CS_Low(ALT_CS_PORT, ALT_CS_PIN);
    uint8_t readback = SPI_Send(R_OUT_CFG, 0xFF, SPI_READ);
    SPI_CS_High(ALT_CS_PORT, ALT_CS_PIN);

    if (readback != CTRL_MEAS_CFG)
        UART_Print("BMP280 CTRL_MEAS MISMATCH: wrote 0x%02X, read 0x%02X\r\n", CTRL_MEAS_CFG, readback);

    /* wait out one full measure+standby cycle so the output registers hold real data,
       not the power-on-reset default; every read after this is safe unconditionally
       since normal mode double-buffers the output registers. a fixed delay is used
       instead of polling STATUS_MEASURING since the bit may not have risen yet by
       the time we'd check it, right after enabling normal mode */
    HAL_Delay(50);

    return 0;
}

uint8_t BMP_Get_Baro_Out(void)
{
    uint8_t buf[6];

    SPI_BurstRead(ALT_CS_PORT, ALT_CS_PIN, R_PRESS_OUT, buf, 6);

    int32_t press_raw = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t temp_raw  = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    g_baro_data.temp_c   = BMP_Compensate_Temp(temp_raw);  // must run first, sets t_fine
    g_baro_data.press_pa = BMP_Compensate_Press(press_raw);

    return 0;
}

void BMP_Print(void)
{
    UART_Print("Baro Temp: %.2f C  Press: %.2f Pa\r\n\n\n",
            g_baro_data.temp_c,
            g_baro_data.press_pa);
}
