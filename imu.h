#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define IMU_ADDR_LOW  0x68
#define IMU_ADDR_HIGH 0x69
#define IMU_MEAN_WINDOW 20

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_raw_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
} imu_accel_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
} imu_angles_t;

bool imu_init(void);
bool imu_read_raw(imu_raw_t *raw);
bool imu_read_accel_g(imu_accel_t *accel);
bool imu_read_angles(imu_angles_t *angles);
bool imu_capture(void);

float imu_pitch_deg_from_accel(const imu_accel_t *accel);
float imu_roll_deg_from_accel(const imu_accel_t *accel);
float get_instant_pitch(void);
float get_instant_roll(void);
float get_mean_pitch(void);
float get_mean_roll(void);
uint8_t imu_get_addr(void);

#endif
