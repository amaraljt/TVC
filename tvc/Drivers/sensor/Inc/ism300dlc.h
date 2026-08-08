#ifndef ISM300DLC_H
#define ISM300DLC_H

#include "spi.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quat;

typedef struct {
    float x;
    float y;
    float z;
} Vector;

extern Vector g_gyro_rps;
extern Vector g_gyro_bias;
extern Vector g_accel_gs;
extern Quat   g_cur_quat;

void IMU_Init(void);
void IMU_Read_Burst(uint8_t reg, uint8_t *buf);
uint8_t IMU_Read_Status(void);
uint8_t IMU_Get_Gyro_Out(void);
uint8_t IMU_Get_Accel_Out(void);
void IMU_Callibrate_Gyro(void);
void IMU_Print(void);

void IMU_Mahony_Filter(void);
Vector IMU_Cross_Product(Vector v1, Vector v2);
Quat IMU_Quat_Mult(Quat q1, Quat q2);
Vector IMU_Acceleration_Error(Vector v_accel_meas, Vector v_grav_body);
Vector IMU_Predicted_Gravity_Direction();
void IMU_Normalize_Vec(Vector *v);
void IMU_Normalize_Quat(Quat *q);
Vector IMU_Corrected_Orientation(Vector accel_err);
Quat IMU_Rate_Of_Change(Vector v_gyro_corrected);
Quat IMU_Update_Orientation(Quat q_gyro_rate_of_change);


#endif /* ISM300DLC_H */
