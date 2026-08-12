#include "control.h"
#include "ism300dlc.h"
#include "tim.h"
#include <math.h>
#include "uart.h"

#define PID_KP 1.0f
#define PID_KD 1.0f

EulerAngle g_pid_euler_angle = {0};

static PidState s_yaw_pid = {0};
static PidState s_pitch_pid = {0};

static uint32_t s_yaw_us = SERVO_CENTER_US;
static uint32_t s_pitch_us = SERVO_CENTER_US;

void PID_Control_Loop(void)
{
    float yaw_err, pitch_err, yaw_out, pitch_out;

    PID_Quat_To_Euler(g_cur_quat);

    yaw_err = 0.0f - g_pid_euler_angle.yaw;
    pitch_err = 0.0f - g_pid_euler_angle.pitch;

    yaw_out = PID_Control(&s_yaw_pid, yaw_err);
    pitch_out = PID_Control(&s_pitch_pid, pitch_err);

    s_yaw_us += 0.1;
    s_pitch_us += 0.1;

    TIM_Set_Servo_Us(SERVO_YAW_CH, s_yaw_us);
    TIM_Set_Servo_Us(SERVO_PITCH_CH, s_pitch_us);
}

/* Diagnostic snapshot. Call at ~1Hz from the main loop, never per-iteration -
   UART_Print blocks and would blow the control budget at CONTROL_RATE_HZ. */
void PID_Print(void)
{
    UART_Print("euler y/p: %.4f %.4f | integral y/p: %.4f %.4f | us y/p: %lu %lu | overrun: %lu\r\n",
            g_pid_euler_angle.yaw,
            g_pid_euler_angle.pitch,
            s_yaw_pid.integral,
            s_pitch_pid.integral,
            (unsigned long)s_yaw_us,
            (unsigned long)s_pitch_us,
            (unsigned long)g_overrun_count);
}

void PID_Quat_To_Euler(Quat q)
{
    float r11, r21, r31, r32, r33;

    /* Rotation matrix elements needed for ZYX extraction */
    r11 = (q.w*q.w) + (q.x*q.x) - (q.y*q.y) - (q.z*q.z);
    r21 = 2.0f * ((q.x*q.y) + (q.w*q.z));
    r31 = 2.0f * ((q.x*q.z) - (q.w*q.y));
    r32 = 2.0f * ((q.y*q.z) + (q.w*q.x));
    r33 = (q.w*q.w) - (q.x*q.x) - (q.y*q.y) + (q.z*q.z);

    /* asinf outside [-1,1] is NaN, and rounding can push r31 just past 1.0 */
    if (r31 > 1.0f)
        r31 = 1.0f;
    else if (r31 < -1.0f)
        r31 = -1.0f;

    /* Tait-Bryan ZYX (rad) */
    g_pid_euler_angle.yaw = atan2f(r21, r11);
    g_pid_euler_angle.pitch = asinf(-r31);
    g_pid_euler_angle.roll = atan2f(r32, r33);
}

/* output = Kp*error + Ki*∫error dt + Kd*(d(error)/dt) */
float PID_Control(PidState *pid, float err)
{
    float proportional, derivative, output;

    proportional = PID_KP * err;
    derivative = PID_KD * ((err - pid->prev_err) / CONTROL_DT);
    pid->prev_err = err;

    output = proportional + derivative;

    return output;
}
