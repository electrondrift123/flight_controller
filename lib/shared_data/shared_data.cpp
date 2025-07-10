#include "shared_data.h"

// Allocate memory for global shared sensor data
mpu6050Data_t mpuData;
magData_t     magData;
bmp280Data_t  bmpData;
MadgwickData_t madData;

PIDControllerData_t pidRoll;
PIDControllerData_t pidPitch;
PIDControllerData_t pidYaw;
PIDControllerData_t pidThrottle; // for altitude hold

volatile float eulerAngles[3] = {0.0f, 0.0f, 0.0f};

volatile float inputList[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
volatile float loraList[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};