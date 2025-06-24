// It uses NED orientation
// x (north/forward), y (right), z (down)

#ifndef QMC5883P_H
#define QMC5883P_H

#include <Arduino.h>

#define QMC5883P_ADDR 0x2C

typedef struct {
  float mx, my, mz;
  float angleYaw;
} magData_t;

bool QMC5883P_init(void);
bool QMC5883P_read(magData_t* magData);

// Tilt compensation helper -> Angle in degrees
void QMC5883P_updateYawWithTilt(magData_t* mag, float roll, float pitch);

#endif