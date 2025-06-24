// #include "Arduino.h"
#include "MPU6050.h"
#include <Wire.h>
#include <math.h>

bool MPU6050_init(void) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Power management
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  // Low-pass filter config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05); // 0x01 for faster response
  if (Wire.endTransmission() != 0) return false;

  // Accelerometer config (+/- 8g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  if (Wire.endTransmission() != 0) return false;

  // Gyroscope config (+/- 500 deg/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  if (Wire.endTransmission() != 0) return false;

  return true;
}

bool MPU6050_read(mpu6050Data_t* d) {
  // Read accelerometer
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDR, 6) != 6) return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // Read gyroscope
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDR, 6) != 6) return false;

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  d->gx = gx;
  d->gy = gy;
  d->gz = gz;
  d->ax = - ax / MPU6050_ACCEL_SENSITIVITY;
  d->ay = ay / MPU6050_ACCEL_SENSITIVITY;
  d->az = az / MPU6050_ACCEL_SENSITIVITY - 0.25f;

  d->wx  = (gx / MPU6050_GYRO_SENSITIVITY) * DEG_TO_RAD;
  d->wy = - (gy / MPU6050_GYRO_SENSITIVITY) * DEG_TO_RAD;
  d->wz   = - (gz / MPU6050_GYRO_SENSITIVITY) * DEG_TO_RAD;

  d->angleRoll = atan2f(d->ay, sqrtf(d->ax * d->ax + d->az * d->az)) * RAD_TO_DEG;
  d->anglePitch = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD_TO_DEG;

  return true;
}
