#include "flight.h"
#include "ism300dlc.h"
#include "control.h"
#include "w25qxx.h"
#include "tim.h"
#include <math.h>

/* ---- Transition thresholds ----------------------------------------------
   Motor: Estes F15-0 - avg thrust 14.4N, peak 25.3N
   (~0.4s in), 3.5s burn. Liftoff mass not yet weighed...

     accel_g ~= thrust(N) / (mass_kg * 9.81) */
#define BOOST_ACCEL_G        2.0f
#define BOOST_HOLD_TICKS     3      /* ~60ms @ 50Hz - rejects handling bumps */

/* Burnout detect: thrust falls away, no longer need boost-level margin. */
#define COAST_ACCEL_G        1.3f
#define COAST_HOLD_TICKS     5      /* ~100ms @ 50Hz */

/* "At rest" - used both for the pad-arm check and the post-flight landed
   check. */
#define STATIONARY_ACCEL_TOL  0.15f   /* |accel_g - 1.0| must stay under this */
#define ARM_HOLD_TICKS        (30UL * CONTROL_RATE_HZ)   /* 30s continuous */
#define LANDED_HOLD_TICKS     (5UL * CONTROL_RATE_HZ)    /* 5s continuous */

/* "Upright" is checked as closeness of g_cur_quat to identity, NOT a
   specific Euler angle - this makes the check independent of which axis is
   currently configured as "up" (BENCH today, ROCKET later, see control.c),
   since the Mahony filter always converges to identity when resting in
   whatever attitude g_gravity_quat currently defines as the reference.

   For a unit quaternion, x^2+y^2+z^2 = sin^2(theta/2) where theta is the
   rotation angle away from identity. The threshold below is sin^2(5 deg),
   i.e. ~10 degrees of total tilt. */
#define ARM_TILT_VECMAG_SQ_MAX  0.0076f

volatile FlightState g_flight_state = FLIGHT_DISARM;

static uint32_t s_state_ticks = 0;   /* ticks continuously satisfying the current state's exit condition */

/* ---- Minimal flight logger: quaternion + tick, 20 bytes/record -------- */
typedef struct {
    uint32_t tick;
    float qw, qx, qy, qz;
} LogRecord;

#define LOG_RECORDS_PER_FLUSH  (W25_PAGE_SIZE / sizeof(LogRecord))   /* 256/20 = 12, 16B/page unused */

static LogRecord s_log_buf[LOG_RECORDS_PER_FLUSH];
static uint16_t  s_log_count = 0;
static uint32_t  s_log_addr = 0;

static void Log_Flush(void)
{
    if (s_log_count == 0)
        return;

    W25_Write_Page(s_log_addr, (const uint8_t *)s_log_buf, s_log_count * sizeof(LogRecord));
    s_log_addr += W25_PAGE_SIZE;   /* always a full page pitch, even on a partial final flush */
    s_log_count = 0;
}

static void Log_Sample(uint32_t tick)
{
    if (s_log_count >= LOG_RECORDS_PER_FLUSH)
        Log_Flush();

    s_log_buf[s_log_count].tick = tick;
    s_log_buf[s_log_count].qw = g_cur_quat.w;
    s_log_buf[s_log_count].qx = g_cur_quat.x;
    s_log_buf[s_log_count].qy = g_cur_quat.y;
    s_log_buf[s_log_count].qz = g_cur_quat.z;
    s_log_count++;
}

static void Wiggle(void)
{
    const float amp = 3.0f;
    int cycle;

    for (cycle = 0; cycle < 3; cycle++) {
        Servo_Set_Gimbal_Deg(amp, amp);
        HAL_Delay(150);
        Servo_Set_Gimbal_Deg(-amp, -amp);
        HAL_Delay(150);
    }
    Servo_Set_Gimbal_Deg(0.0f, 0.0f);
}

static uint8_t Is_Stationary(void)
{
    return fabsf(g_accel_mag_g - 1.0f) < STATIONARY_ACCEL_TOL;
}

static uint8_t Is_Stationary_And_Upright(void)
{
    float tilt_sq = g_cur_quat.x * g_cur_quat.x
                   + g_cur_quat.y * g_cur_quat.y
                   + g_cur_quat.z * g_cur_quat.z;

    return Is_Stationary() && (tilt_sq < ARM_TILT_VECMAG_SQ_MAX);
}

void Flight_Update(void)
{
    static uint32_t tick = 0;
    tick++;

    switch (g_flight_state) {

    case FLIGHT_DISARM:
        if (Is_Stationary_And_Upright()) {
            s_state_ticks++;
            if (s_state_ticks >= ARM_HOLD_TICKS) {
                g_flight_state = FLIGHT_ARMED;
                s_state_ticks = 0;
                Wiggle();
            }
        } else {
            s_state_ticks = 0;   /* moved or tilted - restart the 30s count */
        }
        break;

    case FLIGHT_ARMED:
        if (g_accel_mag_g > BOOST_ACCEL_G) {
            s_state_ticks++;
            if (s_state_ticks >= BOOST_HOLD_TICKS) {
                g_flight_state = FLIGHT_BOOST;
                s_state_ticks = 0;
                s_log_addr = 0;    /* fresh log for this flight */
                s_log_count = 0;
            }
        } else {
            s_state_ticks = 0;
        }
        break;

    case FLIGHT_BOOST:
        Log_Sample(tick);
        if (g_accel_mag_g < COAST_ACCEL_G) {
            s_state_ticks++;
            if (s_state_ticks >= COAST_HOLD_TICKS) {
                g_flight_state = FLIGHT_COAST;
                s_state_ticks = 0;
                Log_Flush();   /* logging stops here - flush whatever's left */
            }
        } else {
            s_state_ticks = 0;
        }
        break;

    case FLIGHT_COAST:
        if (Is_Stationary()) {
            s_state_ticks++;
            if (s_state_ticks >= LANDED_HOLD_TICKS) {
                g_flight_state = FLIGHT_LANDED;
                s_state_ticks = 0;
            }
        } else {
            s_state_ticks = 0;
        }
        break;

    case FLIGHT_LANDED:
        /* Terminal. No auto re-arm - a second flight without an explicit
           reset/erase would append onto or overwrite this one's log. */
        break;
    }
}
