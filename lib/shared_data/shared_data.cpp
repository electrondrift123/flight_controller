#include "shared_data.h"

// Allocate memory for global shared sensor data
mpu6050Data_t mpuData;
magData_t     magData;
bmp280Data_t  bmpData;
MadgwickData_t madData;

LyGAPIDControllerData_t pidRoll;
LyGAPIDControllerData_t pidPitch;
LyGAPIDControllerData_t pidYaw;
PIDControllerData_t pidThrottle; // for altitude hold

LyGAPIDControllerData_t pidRollRate;
LyGAPIDControllerData_t pidPitchRate;
LyGAPIDControllerData_t pidYawRate;

volatile float eulerAngles[3] = {0.0f, 0.0f, 0.0f};

volatile float inputList[8] = {1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float telemetry[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

volatile float MadgwickSensorList[9] = {0.00f, 0.00f, 0.00f,
                                        0.00f, 0.00f, 0.00f,
                                        0.00f, 0.00f, 0.00f};

volatile bool SAFE_WDT = true;