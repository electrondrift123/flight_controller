#ifndef QMC5883P_H
#define QMC5883P_H

#include <Arduino.h>

#define QMC5883P_ADDR 0x2C

// Not yet tried mounted on drone
static float minX = 1200, maxX = 2100; // this is okay for y axis
static float minY = 1200, maxY = 1900; // this is okay now (x axis)
static float minZ = 4900, maxZ = 5600; // acceptable


typedef struct {
  float mx, my, mz;
  float angleYaw;
} magData_t;

// // Optional: Extract calibration separately
// typedef struct {
//   int16_t minX, maxX;
//   int16_t minY, maxY;
//   int16_t minZ, maxZ;
// } magCalib_t;

bool QMC5883P_init(void);
bool QMC5883P_read(magData_t* magData);  // still uses hardcoded min/max
void QMC5883P_updateYawWithTilt(magData_t* mag, float roll, float pitch);

#endif
