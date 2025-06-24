// Written by: Ayob II M. Tuanadatu | June 2025
// We can upgrade the library included so that
// it can be reused in other platform than Arduino framework.

// Modified for NED orientation
// x (forward), y (right), z (down)

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

#define MPU6050_ADDR 0x68

#define MPU6050_GYRO_SENSITIVITY 65.5f
#define MPU6050_ACCEL_SENSITIVITY 4096.0f

#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.0174533f

typedef struct {
  float gx, gy, gz;
  float ax, ay, az;
  float wx, wy, wz; // rate Roll, Pitch, Yaw
  float angleRoll, anglePitch;
} mpu6050Data_t;

bool MPU6050_init(void);
bool MPU6050_read(mpu6050Data_t* data);

#endif
