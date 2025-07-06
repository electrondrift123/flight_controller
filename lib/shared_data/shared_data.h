#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <Wire.h>

#include "QMC5883P.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Madgwick.h"

// Global shared sensor data
extern mpu6050Data_t mpuData;
extern magData_t     magData;
extern bmp280Data_t  bmpData;
extern MadgwickData_t madData;

// Euler angles output from filter
extern volatile float eulerAngles[3];

#endif // SHARED_DATA_H
