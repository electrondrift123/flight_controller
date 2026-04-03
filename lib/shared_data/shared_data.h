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
#include "Butterworth2ndLPF.h"
#include "EMA.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "velocity_control_z.h"

// Constants
#define PI 3.1415f

// outer loop (desired angular rate in rad/s): 180 deg/sec
#define U_MAX_ROLL          PI 
#define U_MAX_PITCH         PI 

// inner loop: it is pwm ticks in us for authority (PID 40% authority) 
#define U_MAX_ROLL_RATE     150.0f
#define U_MAX_PITCH_RATE    150.0f
#define U_MAX_YAW_RATE      100.0f

// Adaptive PID config: 
#define GAMMA_B         1.0f
#define SIGMA           0.001f
#define B_SIGN          1.0f
#define CONTROLLER_MODE 0.0f // 0 = adaptive, 1 = static

// Initial P-PID Gains:
#define P   4.0f

#define KP  45.0f
#define KI  10.0f
#define KD  0.008f

// MAX commands in float: 
#define THROTTLE_MAX 100.0f // throttle is now [-0.8, 0.8] m/s cmd velocty z 
#define THROTTLE_MIN -100.0f
#define YAW_MAX             PI * 2.0f   // 360 deg/s max cmd
#define PITCH_ROLL_MAX      PI / 6.0f   // 30 deg max cmd

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

// Butterworth 2nd order LPF
// extern Butterworth2ndLPF_t accelLPF; // Accelerometer LPF
// extern Butterworth2ndLPF_t gyroLPF;  // Gyroscope LPF
// extern Butterworth2ndLPF_t pidLPF;  // Gyroscope LPF
extern EMA_t axLPF, ayLPF, azLPF; // Accelerometer LPF
extern EMA_t pidLPF; // EMA for PID output
extern EMA_t dP;
extern EMA_t dkp_;
extern EMA_t dki_;
extern EMA_t dkd_;

// altitude hold (z-axis velocity control)
extern VelocityControlZData_t vc_z;
extern EMA_t vz_h_LPF;  // velocity estimate from altitude derivative

// for inputs (joysticks)
extern EMA_t T_LPF;
extern EMA_t Y_LPF;
extern EMA_t P_LPF;
extern EMA_t R_LPF;

// Failsafe for Radio 
extern TimerHandle_t linkWatchdogTimer;
extern const TickType_t LINK_TIMEOUT_MS;           // your desired timeout
extern volatile bool connection_ok;

// Euler angles output from filter
extern volatile float eulerAngles[3];

extern volatile float inputList[8]; // [T, Y, P, R, KILL, E-land, sigma, learning rate] // to receive
extern volatile float telemetry[5]; //now: [alt, P, kp, ki, kd] //[Lat, lon, alt, heading, distance, batt]
// temporary telemtry: [roll, pitch, yaw or heading, alt, radio_state] 

// from sensor reading task
extern volatile float MadgwickSensorList[9];

extern volatile bool SAFE_WDT;

#endif // SHARED_DATA_H
