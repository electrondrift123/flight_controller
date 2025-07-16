#include <Wire.h>
#include "QMC5883P.h"

// uses NED orientation
// Dont touch any calibration.

// Register addresses
#define REG_DATA_X_LSB 0x01
#define REG_DATA_X_MSB 0x02
#define REG_DATA_Y_LSB 0x03
#define REG_DATA_Y_MSB 0x04
#define REG_DATA_Z_LSB 0x05
#define REG_DATA_Z_MSB 0x06
#define REG_CTRL1      0x0A
#define REG_CTRL2      0x0B

bool QMC5883P_init(void) {
   // CTRL1: OSR2=00, OSR1=00 (OSR=8), ODR=11 (200 Hz), MODE=11 (Continuous)
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_CTRL1);
  Wire.write((0<<6)|(0<<4)|(3<<2)|3);
  if (Wire.endTransmission() != 0) return false;

  // CTRL2: Soft reset off, self-test off, RNG=00 (2G full-scale; ±2 G), SET/RESET=01
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_CTRL2);
  Wire.write((0<<7)|(0<<6)|(0<<4)|1);
  if (Wire.endTransmission() != 0) return false;

  return true;
}

// Not yet tried mounted on drone
static float minX = 1200, maxX = 2100; // this is okay for y axis
static float minY = 1200, maxY = 1900; // this is okay now (x axis)
static float minZ = 4900, maxZ = 5600; // acceptable

bool QMC5883P_read(magData_t* magData) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_DATA_X_LSB);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(QMC5883P_ADDR, 6) != 6) return false;

  int16_t mx = Wire.read() | (Wire.read() << 8);
  int16_t my = Wire.read() | (Wire.read() << 8);
  int16_t mz = Wire.read() | (Wire.read() << 8);

  // Convert to float and apply axis sign flip if needed
  float mx_f = (float)mx;
  float my_f = -(float)my;  // flipping Y axis is often needed to correct handedness
  float mz_f = (float)mz;

  // Compute offsets
  float offsetX = (maxX + minX) / 2.0f;
  float offsetY = (maxY + minY) / 2.0f;
  float offsetZ = (maxZ + minZ) / 2.0f;

  // Compute scales (half the range)
  float scaleX = (maxX - minX) / 2.0f;
  float scaleY = (maxY - minY) / 2.0f;
  float scaleZ = (maxZ - minZ) / 2.0f;

  // Prevent division by zero
  if (scaleX == 0) scaleX = 1;
  if (scaleY == 0) scaleY = 1;
  if (scaleZ == 0) scaleZ = 1;

  // Normalize to [-1, 1] // swap mx & my (x @ 0 deg / north) to match mpu6050
  magData->mx = -(mx_f - offsetX) / scaleX;
  magData->my = (my_f - offsetY) / scaleY;
  magData->mz = (mz_f - offsetZ) / scaleZ;

  return true;
}

// Core tilt compensation function -> roll & pitch in deg
float compensateYaw_(float mx, float my, float mz, float roll, float pitch) {
  // convert the angles into rad
  float degToRad = PI / 180.0f;
  float roll_rad = roll * degToRad;
  float pitch_rad = pitch * degToRad;

  float xh = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
  float yh = mx * sinf(roll_rad) * sinf(pitch_rad) + my * cosf(roll_rad) - mz * sinf(roll_rad) * cosf(pitch_rad);
  float heading = atan2f(yh, xh) * 180.0f / PI;

  if (heading < 0) heading += 360.0f;
  return heading;
}

// Helper that writes compensated heading into the struct -> Angles in deg
void QMC5883P_updateYawWithTilt(magData_t* mag, float roll, float pitch) {
  mag->angleYaw = compensateYaw_(mag->mx, mag->my, mag->mz, roll, pitch);
}


//// TODO: create a function that returns the right calibration constants