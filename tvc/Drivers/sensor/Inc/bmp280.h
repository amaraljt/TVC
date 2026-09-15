#ifndef BMP280_H
#define BMP280_H

#include "spi.h"
#include <stdint.h>

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BaroCalib;

typedef struct {
    float temp_c;
    float press_pa;
} BaroData;

extern BaroData g_baro_data;

uint8_t BMP_Whoami(void);
uint8_t BMP_Init(void);
uint8_t BMP_Read_Status(void);
uint8_t BMP_Get_Baro_Out(void);
void BMP_Print(void);

#endif /* BMP280_H */
