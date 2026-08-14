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

   PID output is now in GIMBAL DEGREES, so Kp is gimbal-deg per radian of
   attitude error and Kd is gimbal-deg per (radian/second).

   Kp: saturate the gimbal (MAX_DEFLECT, 6 deg) at ~15 deg of tilt
       -> 6.0 / 0.262 rad ~= 23
   Kd: Kd/Kp ~= 0.13 s, roughly zeta 0.7 at a ~10 rad/s bandwidth */
#define PID_KP 23.0f
#define PID_KD 3.0f

/* ---- Servo / gimbal calibration ---------------------------------------
   !! PLACEHOLDERS - these are Jared's friend's numbers, measured on a
   !! DIFFERENT build (6.75" lever arm, 2026-07-31). They will be wrong for
   !! this gimbal. TRIM in particular depends on where the horn happens to
   !! sit on the servo spline, and USPD depends on the lever arm and linkage
   !! geometry. Measure all of it on this hardware before flying.

   TRIM  us that puts the gimbal at neutral (thrust line through the CG)
   USPD  us per degree of gimbal deflection
   MIN   us, kept inside the mechanical stop
   MAX   us, kept inside the mechanical stop
   SIGN  +1 or -1, whichever makes a positive command deflect the gimbal so
         the rocket is pushed BACK toward upright. Get this backwards and the
         loop is positive feedback. Verify by hand, one axis at a time.

   Channel map here: SERVO_YAW_CH = TIM2_CH1 = PA0
                     SERVO_PITCH_CH = TIM2_CH2 = PA1                       */
#define SERVO_Y_TRIM    1575u
#define SERVO_Y_USPD    44.4f
#define SERVO_Y_MIN     1200u
#define SERVO_Y_MAX     1900u
#define SERVO_Y_SIGN    (+1.0f)

#define SERVO_P_TRIM    1825u
#define SERVO_P_USPD    48.5f
#define SERVO_P_MIN     1475u
#define SERVO_P_MAX     2300u
#define SERVO_P_SIGN    (+1.0f)

/* Gimbal deflection limit, in degrees, both axes. This is the saturation
   point of the controller - PID output is in gimbal degrees. */
#define MAX_DEFLECT     6.0f

EulerAngle g_pid_euler_angle = {0};

static PidState s_yaw_pid = {0};
static PidState s_pitch_pid = {0};

static uint32_t s_yaw_us = SERVO_Y_TRIM;
static uint32_t s_pitch_us = SERVO_P_TRIM;

/* Converts a commanded gimbal angle in degrees to a servo pulse width.
   Clamped twice: once on the commanded angle (MAX_DEFLECT, the aerodynamic
   /control limit) and once on the resulting pulse (MIN/MAX, the mechanical
   stop guard). The second clamp is the one that protects the hardware. */
static uint32_t Gimbal_To_Pulse(float cmd_deg, float sign, uint32_t trim,
                                float uspd, uint32_t min_us, uint32_t max_us)
{
    float pulse;

    if (cmd_deg > MAX_DEFLECT)
        cmd_deg = MAX_DEFLECT;
    else if (cmd_deg < -MAX_DEFLECT)
        cmd_deg = -MAX_DEFLECT;

    pulse = (float)trim + (sign * cmd_deg * uspd);

    if (pulse < (float)min_us)
        pulse = (float)min_us;
    else if (pulse > (float)max_us)
        pulse = (float)max_us;

    return (uint32_t)pulse;
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

    s_yaw_us = Gimbal_To_Pulse(yaw_out, SERVO_Y_SIGN, SERVO_Y_TRIM,
                               SERVO_Y_USPD, SERVO_Y_MIN, SERVO_Y_MAX);
    s_pitch_us = Gimbal_To_Pulse(pitch_out, SERVO_P_SIGN, SERVO_P_TRIM,
                                 SERVO_P_USPD, SERVO_P_MIN, SERVO_P_MAX);

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
