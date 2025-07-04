#include "shared_data.h"

// Allocate memory for global shared sensor data
mpu6050Data_t mpuData;
magData_t     magData;
bmp280Data_t  bmpData;
MadgwickData_t madData;

float eulerAngles[3] = {0.0f, 0.0f, 0.0f};
