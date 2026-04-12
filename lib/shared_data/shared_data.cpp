#include "shared_data.h"
#include "velocity_control_z.h"

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

// Butterworth2ndLPF_t accelLPF; // Accelerometer LPF
// Butterworth2ndLPF_t gyroLPF;  // Gyroscope LPF
// Butterworth2ndLPF_t pidLPF;  // Gyroscope LPF (init inside LyGAPID)
EMA_t axLPF, ayLPF, azLPF; // Accelerometer LPF
EMA_t pidLPF; // EMA for PID output
EMA_t dP;
EMA_t dkp_;
EMA_t dki_;
EMA_t dkd_;

// altitude hold (z-axis velocity control)
VelocityControlZData_t vc_z;
VelocityControlZData_t vz_in;
EMA_t alt_sp_LPF;  
EMA_t alt_LPF;
EMA_t raw_alt_LPF;
EMA_t vzLPF;

EMA_t d_filter;

// voltage monitor
EMA_t VbLPF;

// kalman filter state for altitude estimation
KalmanState_t kalmanState;

// for inputs (joysticks)
EMA_t T_LPF;
EMA_t Y_LPF;
EMA_t P_LPF;
EMA_t R_LPF;

// Failsafe for Radio
// === Global / static variables ===
TimerHandle_t linkWatchdogTimer = NULL;
const TickType_t LINK_TIMEOUT_MS = 200;           
volatile bool connection_ok = false;

volatile float eulerAngles[3] = {0.0f, 0.0f, 0.0f};

volatile float inputList[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.01f};
volatile float telemetry[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

volatile float MadgwickSensorList[9] = {0.00f, 0.00f, 0.00f,
                                        0.00f, 0.00f, 0.00f,
                                        0.00f, 0.00f, 0.00f};

volatile bool SAFE_WDT = true;