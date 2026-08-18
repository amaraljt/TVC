#ifndef CONTROL_H
#define CONTROL_H

#include "ism300dlc.h"

typedef struct {
    float yaw;
    float pitch;
    float roll;
} EulerAngle;

typedef struct {
    float integral;
    float prev_err;
} PidState;

extern EulerAngle g_pid_euler_angle;

void  PID_Control_Loop(void);
void  PID_Quat_To_Euler(Quat q);
float PID_Control(PidState *pid, float err);
void  PID_Print(void);

#endif /* CONTROL_H */
