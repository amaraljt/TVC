#ifndef ISM300DLC_H
#define ISM300DLC_H

#include "spi.h"
#include <stdint.h>

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} GyroBias;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} GyroRps;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} AccelPosition;

/* TODO */
extern GyroBias g_gyro_bias;
extern GyroRps g_gyro_rps;
extern AccelPosition g_accel_pos;

void IMU_Init(void);
void IMU_Read_Burst(uint8_t reg, uint8_t *buf);
uint8_t IMU_Read_Status(void);
uint8_t IMU_Get_Gyro_Out(void);
void IMU_Get_Accel_Out(void);
void IMU_Callibrate_Gyro(void);
void Gyro_Print(void);

#endif /* ISM300DLC_H */
