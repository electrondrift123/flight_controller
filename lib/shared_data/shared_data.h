#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <Wire.h>

#include "QMC5883P.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Madgwick.h"
#include "PID.h"

// Global shared sensor data
extern mpu6050Data_t mpuData;
extern magData_t     magData;
extern bmp280Data_t  bmpData;
extern MadgwickData_t madData;

// Angle Mode 
extern PIDControllerData_t pidRoll;
extern PIDControllerData_t pidPitch;
extern PIDControllerData_t pidYaw;
extern PIDControllerData_t pidThrottle; // for altitude hold

extern PIDControllerData_t pidRollRate;
extern PIDControllerData_t pidPitchRate;
extern PIDControllerData_t pidYawRate;

// PID params


//// TODO: rate Mode to be added

// Euler angles output from filter
extern volatile float eulerAngles[3];

extern volatile float inputList[8]; // [T, R, P, Y, RTL, EL, Kill, ALT_H]
extern volatile float telemetry[6]; //[Lat, lon, alt, heading, distance, batt]
// temporary telemtry: [roll, pitch, yaw or heading, alt, radio_state] 

// from sensor reading task
extern volatile float MadgwickSensorList[9];

extern volatile bool SAFE_WDT;

#endif // SHARED_DATA_H
