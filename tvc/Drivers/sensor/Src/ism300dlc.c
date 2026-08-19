#include "ism300dlc.h"
#include "spi.h"
#include "uart.h"
#include "tim.h"
#include <stdint.h>

#define IMU_CS_PORT   GPIOA
#define IMU_CS_PIN    GPIO_PIN_8

#define SPI_READ    1
#define SPI_WRITE   0

#define R_WHOAMI              0x0F
#define R_STATUS              0x1E

#define R_ACCEL_CFG   0x10
#define R_GYRO_CFG    0x11

#define R_GYRO_OUT_X    0x22
#define R_ACCEL_OUT_X   0x28

#define ODR_416_2G         0x60
#define ODR_416_250_DPS    0x60

#define STATUS_XLDA   0x01
#define STATUS_GDA    0x02

#define NUM_SAMPLES    200
#define GYRO_SENSITIVITY     8.75f /* 250dps */
#define ACCEL_SENSITIVITY    0.061f /* +-2g */
#define DEG_TO_RAD    0.017453293f

#define MAHONY_KP    1.0f
#define MAHONY_KI    0.1f

Vector g_gyro_rps = {0,0,0};
Vector g_gyro_bias = {0,0,0};
Vector g_accel_gs = {0,0,0};
float g_accel_mag_g = 0.0f;
Quat g_gravity_quat = {0,0,0,1};
Quat g_cur_quat = {1,0,0,0};
Vector v_res_bias = {0,0,0};

uint8_t gyro_callibration = -1;

uint8_t IMU_Whoami(void)
{
    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t id = SPI1_Send(R_WHOAMI, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    if (id != 0x6B) {
        UART_Print("WHOAMI FAILED: 0x%02X\r\n", id);
        return 1;
    }
    return 0;
}

void IMU_Read_Burst(uint8_t reg, uint8_t *buf)
{
    SPI_BurstRead(IMU_CS_PORT, IMU_CS_PIN, reg, buf, 6);
}

uint8_t IMU_Read_Status(void)
{
    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t status = SPI1_Send(R_STATUS, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);
    return status;
}

uint8_t IMU_Get_Gyro_Out(void)
{
    uint8_t buf[6];
    uint8_t status = IMU_Read_Status();
    if (!(status & STATUS_GDA)) {
        UART_Print("[gyro] NOT READY: STATUS=0x%02X (last sample reused)\r\n", status);
        return 1;
    }

    IMU_Read_Burst(R_GYRO_OUT_X, buf);

    int16_t g_lsb_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t g_lsb_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t g_lsb_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Convert LSB to rad/s
    g_gyro_rps.x = ((float)(g_lsb_x * GYRO_SENSITIVITY) / 1000) * DEG_TO_RAD;
    g_gyro_rps.y = ((float)(g_lsb_y * GYRO_SENSITIVITY) / 1000) * DEG_TO_RAD;
    g_gyro_rps.z = ((float)(g_lsb_z * GYRO_SENSITIVITY) / 1000) * DEG_TO_RAD;

    if(!gyro_callibration) {
        g_gyro_rps.x -= g_gyro_bias.x;
        g_gyro_rps.y -= g_gyro_bias.y;
        g_gyro_rps.z -= g_gyro_bias.z;
    }

    return 0;
}

void IMU_Callibrate_Gyro(void)
{
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    gyro_callibration = 1;

    for (int i = 0; i < NUM_SAMPLES; ) {
        if (IMU_Get_Gyro_Out() != 0) // rad/s data; skip and retry if not ready yet
            continue;

        gyro_sum_x += g_gyro_rps.x;
        gyro_sum_y += g_gyro_rps.y;
        gyro_sum_z += g_gyro_rps.z;
        i++;
    }

    g_gyro_bias.x = gyro_sum_x / NUM_SAMPLES;
    g_gyro_bias.y = gyro_sum_y / NUM_SAMPLES;
    g_gyro_bias.z = gyro_sum_z / NUM_SAMPLES;

    /* Bias should be small and steady (a few mrad/s). A large value means the
       board moved during calibration and every angle after this will ramp. */
    UART_Print("[calib] gyro bias rad/s: %.6f %.6f %.6f (%d samples)\r\n",
            g_gyro_bias.x, g_gyro_bias.y, g_gyro_bias.z, NUM_SAMPLES);

    gyro_callibration = 0;
}

uint8_t IMU_Get_Accel_Out(void)
{
    uint8_t buf[6];
    uint8_t status = IMU_Read_Status();
    if (!(status & STATUS_XLDA)) {
        UART_Print("[accel] NOT READY: STATUS=0x%02X (last sample reused)\r\n", status);
        return 1;
    }

    IMU_Read_Burst(R_ACCEL_OUT_X, buf);

    int16_t a_lsb_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t a_lsb_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t a_lsb_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Convert LSB to gs
    g_accel_gs.x = ((float)(a_lsb_x * ACCEL_SENSITIVITY) / 1000);
    g_accel_gs.y = ((float)(a_lsb_y * ACCEL_SENSITIVITY) / 1000);
    g_accel_gs.z = ((float)(a_lsb_z * ACCEL_SENSITIVITY) / 1000);

    /* Captured here, before IMU_Mahony_Filter normalizes g_accel_gs to unit
       length and destroys the real magnitude. Flight-state thresholds in
       flight.c read this, not g_accel_gs. */
    g_accel_mag_g = sqrtf((g_accel_gs.x*g_accel_gs.x) +
                           (g_accel_gs.y*g_accel_gs.y) +
                           (g_accel_gs.z*g_accel_gs.z));

    return 0;
}

void IMU_Print(void)
{
    UART_Print("Gyro  X: %.4f  Y: %.4f  Z: %.4f\r\n",
            g_gyro_rps.x, g_gyro_rps.y, g_gyro_rps.z);
    UART_Print("Accel X: %.4f  Y: %.4f  Z: %.4f\r\n",
            g_accel_gs.x, g_accel_gs.y, g_accel_gs.z);
}

void IMU_Init(void)
{
    if (IMU_Whoami() != 0)
        return;

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    SPI1_Send(R_ACCEL_CFG, ODR_416_2G, SPI_WRITE);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    SPI1_Send(R_GYRO_CFG, ODR_416_2G, SPI_WRITE);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t xl_readback = SPI1_Send(R_ACCEL_CFG, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    SPI_CS_Low(IMU_CS_PORT, IMU_CS_PIN);
    uint8_t g_readback = SPI1_Send(R_GYRO_CFG, 0xFF, SPI_READ);
    SPI_CS_High(IMU_CS_PORT, IMU_CS_PIN);

    if (xl_readback != ODR_416_2G)
        UART_Print("CTRL1_XL MISMATCH: wrote 0x%02X, read 0x%02X\r\n", ODR_416_2G, xl_readback);
    if (g_readback != ODR_416_2G)
        UART_Print("CTRL2_G MISMATCH: wrote 0x%02X, read 0x%02X\r\n", ODR_416_2G, g_readback);

    IMU_Callibrate_Gyro();
}

// TODO: add feedback loop
void IMU_Mahony_Filter()
{
    Vector v_grav_body, v_accel_err, v_gyro_corrected;
    Quat q_gyro_rate_of_change, q_orientation_new;

    IMU_Normalize_Vec(&g_accel_gs);  // reduce to direction only, magnitude discarded

    /* Gravity Reference from Earth to Body Frame */
    v_grav_body = IMU_Predicted_Gravity_Direction();

    /* acceleration error */
    v_accel_err = IMU_Acceleration_Error(g_accel_gs, v_grav_body);

    /* Get corrected orientation using PI controller Kp/Ki */
    v_gyro_corrected = IMU_Corrected_Orientation(v_accel_err);

    /* Take derivative of quaternion */
    q_gyro_rate_of_change = IMU_Rate_Of_Change(v_gyro_corrected);

    /* Integrate */
    q_orientation_new = IMU_Update_Orientation(q_gyro_rate_of_change);

    /* Normalize quaternion */
    IMU_Normalize_Quat(&q_orientation_new);

    g_cur_quat = q_orientation_new;
}

Vector IMU_Cross_Product(Vector v1, Vector v2)
{
    Vector ret;

    ret.x = (v1.y*v2.z - v1.z*v2.y);
    ret.y = (v1.z*v2.x - v1.x*v2.z);
    ret.z = (v1.x*v2.y - v1.y*v2.x);

    return ret;
}

Quat IMU_Quat_Mult(Quat q1, Quat q2)
{
    Quat ret;

    // Order matters
    ret.w = (q1.w*q2.w) - (q1.x*q2.x) - (q1.y*q2.y) - (q1.z*q2.z);
    ret.x = (q1.w*q2.x) + (q1.x*q2.w) + (q1.y*q2.z) - (q1.z*q2.y);
    ret.y = (q1.w*q2.y) - (q1.x*q2.z) + (q1.y*q2.w) + (q1.z*q2.x);
    ret.z = (q1.w*q2.z) + (q1.x*q2.y) - (q1.y*q2.x) + (q1.z*q2.w);

    return ret;
}

/* e = a_meas X q_grav_body */
Vector IMU_Acceleration_Error(Vector v_accel_meas, Vector v_grav_body)
{
    Vector ret;

    ret = IMU_Cross_Product(v_accel_meas, v_grav_body);

    return ret;
}

Vector IMU_Predicted_Gravity_Direction()
{
    // Inverse q_cur
    Quat q_cur_inv;
    Quat q_grav_body;
    Vector v_grav_body;

    q_cur_inv.w = g_cur_quat.w;
    q_cur_inv.x = g_cur_quat.x * -1.0;
    q_cur_inv.y = g_cur_quat.y * -1.0;
    q_cur_inv.z = g_cur_quat.z * -1.0;

    // quaternion sandwich
    q_grav_body = IMU_Quat_Mult(IMU_Quat_Mult(q_cur_inv, g_gravity_quat),g_cur_quat);

    // quat to vector (drop w)
    v_grav_body.x = q_grav_body.x;
    v_grav_body.y = q_grav_body.y;
    v_grav_body.z = q_grav_body.z;

    return v_grav_body;
}

void IMU_Normalize_Vec(Vector *v)
{
    double magnitude;

    magnitude = sqrt((v->x*v->x) + (v->y*v->y) + (v->z*v->z));

    v->x /= (float)magnitude;
    v->y /= (float)magnitude;
    v->z /= (float)magnitude;
}

void IMU_Normalize_Quat(Quat *q)
{
    double magnitude;

    magnitude = sqrt((q->w*q->w) + (q->x*q->x) + (q->y*q->y) + (q->z*q->z));

    q->w /= (float)magnitude;
    q->x /= (float)magnitude;
    q->y /= (float)magnitude;
    q->z /= (float)magnitude;
}

Vector IMU_Corrected_Orientation(Vector v_accel_err)
{
    Vector v_gyro_corrected;

    /* Mahony is w + Kp*e + Ki*integral(e) - both corrections carry the same
       sign. v_res_bias is defined as a bias to SUBTRACT below, so it has to
       accumulate -Ki*e for the integral term to end up positive. Accumulating
       +Ki*e here made the I term fight the P term and slowly diverge. */
    v_res_bias.x -= MAHONY_KI * v_accel_err.x * CONTROL_DT;
    v_res_bias.y -= MAHONY_KI * v_accel_err.y * CONTROL_DT;
    v_res_bias.z -= MAHONY_KI * v_accel_err.z * CONTROL_DT;

    v_gyro_corrected.x = g_gyro_rps.x - v_res_bias.x + MAHONY_KP * v_accel_err.x;
    v_gyro_corrected.y = g_gyro_rps.y - v_res_bias.y + MAHONY_KP * v_accel_err.y;
    v_gyro_corrected.z = g_gyro_rps.z - v_res_bias.z + MAHONY_KP * v_accel_err.z;

    return v_gyro_corrected;
}

/* Grabs rate of change of our orientation */
Quat IMU_Rate_Of_Change(Vector v_gyro_corrected)
{
    Quat q_gyro_corrected, ret;

    q_gyro_corrected.w = 0;
    q_gyro_corrected.x = v_gyro_corrected.x;
    q_gyro_corrected.y = v_gyro_corrected.y;
    q_gyro_corrected.z = v_gyro_corrected.z;

    ret = IMU_Quat_Mult(g_cur_quat, q_gyro_corrected);

    ret.w /= 2.0;
    ret.x /= 2.0;
    ret.y /= 2.0;
    ret.z /= 2.0;

    return ret;
}

Quat IMU_Update_Orientation(Quat q_gyro_rate_of_change)
{
    Quat ret;

    ret.w = q_gyro_rate_of_change.w * CONTROL_DT;
    ret.x = q_gyro_rate_of_change.x * CONTROL_DT;
    ret.y = q_gyro_rate_of_change.y * CONTROL_DT;
    ret.z = q_gyro_rate_of_change.z * CONTROL_DT;

    ret.w += g_cur_quat.w;
    ret.x += g_cur_quat.x;
    ret.y += g_cur_quat.y;
    ret.z += g_cur_quat.z;
    
    return ret;
}
