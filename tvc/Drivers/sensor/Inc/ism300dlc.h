#ifndef ISM300DLC_H
#define ISM300DLC_H

#include "spi.h"
#include <stdint.h>

typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} GyroPosition;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} AccelPosition;

/* TODO */
extern GyroPosition g_gyro_pos;
extern AccelPosition g_accel_pos;

void IMU_Init(void);
void IMU_Read_Burst(uint8_t reg, uint8_t *buf);
uint8_t IMU_Read_Status(void);
void IMU_Get_Gyro_Out(void);
void IMU_Get_Accel_Out(void);

#endif /* ISM300DLC_H */
