#include "control.h"
#include "ism300dlc.h"
#include "tim.h"
#include <math.h>
#include "uart.h"

/* The derivative term is Kd*(d(err)/dt), which reduces to Kd * angular rate,
   so Kp is in output-per-radian and Kd is in output-per-(radian/second).
   Holding them equal makes 1 rad/s of hand movement worth as much as a 57
   degree static tilt, which buries the proportional term - fast motion drove
   the servos and slow motion did nothing.

   Kp: full deflection (output 4.0, i.e. SERVO_MAX_DEFLECT_US) at ~15 deg
       tilt -> 4.0 / 0.262 rad ~= 15
   Kd: Kd/Kp ~= 0.13 s, roughly zeta 0.7 at a ~10 rad/s bandwidth */
#define PID_KP 15.0f
#define PID_KD 2.0f

/* Microseconds of servo deflection per unit of PID output. Tune this on the
   bench with the gimbal free to move before touching PID_KP/PID_KD. */
#define SERVO_US_PER_PID_UNIT   100.0f

/* Deflection limit from center, in us. Kept inside the 1000/2000 rails so the
   gimbal never commands the servo into its mechanical stops. */
#define SERVO_MAX_DEFLECT_US    400.0f

EulerAngle g_pid_euler_angle = {0};

static PidState s_yaw_pid = {0};
static PidState s_pitch_pid = {0};

static uint32_t s_yaw_us = SERVO_CENTER_US;
static uint32_t s_pitch_us = SERVO_CENTER_US;

/* Maps a PID output to a servo pulse width centered on SERVO_CENTER_US. */
static uint32_t PID_Out_To_Servo_Us(float pid_out)
{
    float deflect_us = pid_out * SERVO_US_PER_PID_UNIT;

    if (deflect_us > SERVO_MAX_DEFLECT_US)
        deflect_us = SERVO_MAX_DEFLECT_US;
    else if (deflect_us < -SERVO_MAX_DEFLECT_US)
        deflect_us = -SERVO_MAX_DEFLECT_US;

    return (uint32_t)((float)SERVO_CENTER_US + deflect_us);
}

/* Which Euler angle drives which servo depends on how the IMU is mounted.
   The Mahony correction is a_meas x a_pred, and a cross product is always
   perpendicular to its inputs, so the accelerometer can never correct
   rotation ABOUT the gravity vector. That axis is pure gyro integration, it
   drifts without bound, and a servo driven from it walks to its rail and
   parks there. It must never feed a servo.

   BENCH  - board flat, IMU +Z up:  yaw drifts  -> drive from roll and pitch
   ROCKET - IMU +X up:              roll drifts -> drive from yaw and pitch,
                                    and set g_gravity_quat to {0,1,0,0}

   Currently configured for BENCH. */
void PID_Control_Loop(void)
{
    float yaw_err, pitch_err, yaw_out, pitch_out;

    PID_Quat_To_Euler(g_cur_quat);

    /* BENCH: the yaw channel is fed from roll, see the note above. */
    yaw_err = 0.0f - g_pid_euler_angle.roll;
    pitch_err = 0.0f - g_pid_euler_angle.pitch;

    yaw_out = PID_Control(&s_yaw_pid, yaw_err);
    pitch_out = PID_Control(&s_pitch_pid, pitch_err);

    s_yaw_us = PID_Out_To_Servo_Us(yaw_out);
    s_pitch_us = PID_Out_To_Servo_Us(pitch_out);

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

    /* asinf outside [-1,1] returns NaN, and rounding can push r31 just past
       1.0. A NaN here poisons prev_err and every servo command after it, and
       nothing clears it short of a reboot - so clamp before asinf. */
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
