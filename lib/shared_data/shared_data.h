#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include <Wire.h>

#include "QMC5883P.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Madgwick.h"
#include "PID.h"
#include "LyGAPID.h"

// Constants
#define PI 3.1415f

// outer loop (desired angular rate in rad/s): 90 deg/sec
#define U_MAX_ROLL     PI / 2.0f
#define U_MAX_PITCH    PI / 2.0f
#define U_MAX_YAW      PI / 4.0f

// inner loop: it is pwm ticks in us for authority 
#define U_MAX_ROLL_RATE     500.0f
#define U_MAX_PITCH_RATE    500.0f
#define U_MAX_YAW_RATE      300.0f

// Adaptive PID config:
#define GAMMA_B         3e-3f
#define SIGMA           0.01f
#define B_SIGN          1.0f
#define CONTROLLER_MODE 0.0f

// Initial PID Gains:
#define P   1.0f
#define I   1.0f

#define KP  1.0f
#define KI  1.0f
#define KD  1.0f

// Global shared sensor data
extern mpu6050Data_t mpuData;
extern magData_t     magData;
extern bmp280Data_t  bmpData;
extern MadgwickData_t madData;

// // Angle Mode 
// extern PIDControllerData_t pidRoll;
// extern PIDControllerData_t pidPitch;
// extern PIDControllerData_t pidYaw;
extern PIDControllerData_t pidThrottle; // for altitude hold

// extern PIDControllerData_t pidRollRate;
// extern PIDControllerData_t pidPitchRate;
// extern PIDControllerData_t pidYawRate;

// Adaptive PID
extern LyGAPIDControllerData_t pidRoll;
extern LyGAPIDControllerData_t pidPitch;
extern LyGAPIDControllerData_t pidYaw;

extern LyGAPIDControllerData_t pidRollRate;
extern LyGAPIDControllerData_t pidPitchRate;
extern LyGAPIDControllerData_t pidYawRate;

// PID params

// Euler angles output from filter
extern volatile float eulerAngles[3];

extern volatile float inputList[8]; // [T, R, P, Y, RTL, EL, Kill, ALT_H]
extern volatile float telemetry[6]; //[Lat, lon, alt, heading, distance, batt]
// temporary telemtry: [roll, pitch, yaw or heading, alt, radio_state] 

// from sensor reading task
extern volatile float MadgwickSensorList[9];

extern volatile bool SAFE_WDT;

#endif // SHARED_DATA_H
