#include "imu.h"

#include <math.h>

#include "pico/stdlib.h"

#define IMU_I2C_PORT i2c0

#define MPU_REG_SMPLRT_DIV     0x19
#define MPU_REG_CONFIG         0x1A
#define MPU_REG_GYRO_CONFIG    0x1B
#define MPU_REG_ACCEL_CONFIG   0x1C
#define MPU_REG_ACCEL_CONFIG2  0x1D
#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_PWR_MGMT_2     0x6C
#define MPU_REG_WHO_AM_I       0x75

#define MPU_WHO_AM_I_9250      0x71
#define MPU_WHO_AM_I_9255      0x73
#define MPU_WHO_AM_I_6500      0x70

#define ACCEL_LSB_PER_G        16384.0f
#define RAD_TO_DEG             57.2957795f

static uint8_t g_imu_addr = IMU_ADDR_LOW;
static bool g_imu_initialized = false;

static bool imu_write8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int r = i2c_write_blocking(IMU_I2C_PORT, g_imu_addr, buf, 2, false);
    return r == 2;
}

static bool imu_read(uint8_t reg, uint8_t *buf, size_t len)
{
    int w = i2c_write_blocking(IMU_I2C_PORT, g_imu_addr, &reg, 1, true);
    int r = i2c_read_blocking(IMU_I2C_PORT, g_imu_addr, buf, len, false);
    return w == 1 && r == (int)len;
}

static bool imu_read8(uint8_t reg, uint8_t *val)
{
    return imu_read(reg, val, 1);
}

static bool imu_who_am_i_is_valid(uint8_t who_am_i)
{
    return who_am_i == MPU_WHO_AM_I_9250 ||
           who_am_i == MPU_WHO_AM_I_9255 ||
           who_am_i == MPU_WHO_AM_I_6500;
}

bool imu_init(void)
{
    uint8_t who_am_i = 0;
    g_imu_initialized = false;

    g_imu_addr = IMU_ADDR_LOW;
    if (!imu_read8(MPU_REG_WHO_AM_I, &who_am_i) || !imu_who_am_i_is_valid(who_am_i)) {
        g_imu_addr = IMU_ADDR_HIGH;
        if (!imu_read8(MPU_REG_WHO_AM_I, &who_am_i) || !imu_who_am_i_is_valid(who_am_i)) {
            return false;
        }
    }

    if (!imu_write8(MPU_REG_PWR_MGMT_1, 0x80)) return false;
    sleep_ms(100);

    // PLL gyro X comme horloge, accelerometre et gyro actifs.
    if (!imu_write8(MPU_REG_PWR_MGMT_1, 0x01)) return false;
    if (!imu_write8(MPU_REG_PWR_MGMT_2, 0x00)) return false;
    sleep_ms(10);

    // Bande passante moderee, accel +/-2g, gyro +/-250 dps.
    if (!imu_write8(MPU_REG_CONFIG, 0x03)) return false;
    if (!imu_write8(MPU_REG_SMPLRT_DIV, 0x04)) return false;
    if (!imu_write8(MPU_REG_GYRO_CONFIG, 0x00)) return false;
    if (!imu_write8(MPU_REG_ACCEL_CONFIG, 0x00)) return false;
    if (!imu_write8(MPU_REG_ACCEL_CONFIG2, 0x03)) return false;

    g_imu_initialized = true;
    return true;
}

bool imu_read_raw(imu_raw_t *raw)
{
    uint8_t buf[14];

    if (!raw || !g_imu_initialized) return false;
    if (!imu_read(MPU_REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

    raw->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    raw->temp    = (int16_t)((buf[6] << 8) | buf[7]);
    raw->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    raw->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

bool imu_read_accel_g(imu_accel_t *accel)
{
    imu_raw_t raw;

    if (!accel) return false;
    if (!imu_read_raw(&raw)) return false;

    accel->accel_x_g = (float)raw.accel_x / ACCEL_LSB_PER_G;
    accel->accel_y_g = (float)raw.accel_y / ACCEL_LSB_PER_G;
    accel->accel_z_g = (float)raw.accel_z / ACCEL_LSB_PER_G;

    return true;
}

float imu_pitch_deg_from_accel(const imu_accel_t *accel)
{
    if (!accel) return 0.0f;

    float ax = accel->accel_x_g;
    float ay = accel->accel_y_g;
    float az = accel->accel_z_g;

    return atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
}

float imu_roll_deg_from_accel(const imu_accel_t *accel)
{
    if (!accel) return 0.0f;

    float ax = accel->accel_x_g;
    float ay = accel->accel_y_g;
    float az = accel->accel_z_g;

    return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

bool imu_read_angles(imu_angles_t *angles)
{
    imu_accel_t accel;

    if (!angles) return false;
    if (!imu_read_accel_g(&accel)) return false;

    angles->pitch_deg = imu_pitch_deg_from_accel(&accel);
    angles->roll_deg = imu_roll_deg_from_accel(&accel);

    return true;
}

uint8_t imu_get_addr(void)
{
    return g_imu_addr;
}
