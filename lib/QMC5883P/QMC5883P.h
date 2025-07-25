#ifndef QMC5883P_H
#define QMC5883P_H

#include <Arduino.h>

#define QMC5883P_ADDR 0x2C

// Not yet tried mounted on drone
static float minX = 1523, maxX = 2233; 
static float minY = 1224, maxY = 1735; 
static float minZ = 5350, maxZ = 6200; 

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

void calibrate_compass(void);

#endif
