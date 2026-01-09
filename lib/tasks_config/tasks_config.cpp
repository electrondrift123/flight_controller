#include "tasks_config.h"

#include <STM32FreeRTOS.h>
#include <RF24.h>
// #include <RadioLib.h>

#include "QMC5883P.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Madgwick.h"

#include "pin_config.h"
#include "buzzer.h"
#include "shared_data.h"
#include "sync.h"

#include "main_rx.h"

#include "PID.h"
#include "LyGAPID.h"
#include "WDT.h"

#include "Butterworth2ndLPF.h"

// DEFINE PRIORITY LEVELS (status: working)
#define PRIORITY_PID_FUSION   1 // 500 Hz -> try prio: 2 (radio task starve)
#define PRIORITY_WDT          1 // 1 Hz
#define PRIORITY_SENSOR_READ  1 // 1 kHz
#define PRIORITY_RADIO        1 // Interrupt driven
#define PRIORITY_BLINK        1 // 1 Hz

// Complementary filter for altitude estimation
float complementaryAltitude(float baroAlt,
                             float accN, float accE, float accD, // already NED
                             float dt){
  // --- Step 1: Gravity compensation (Down axis) ---
  float acc_z_corrected = (accD - 1.0f) * 9.81f;  // m/s², zero when hovering

  // --- Step 2: Integrate velocity ---
  static float vel_z = 0.0f;
  vel_z += acc_z_corrected * dt;

  // --- Step 3: Predict altitude ---
  static float alt_est = 0.0f;
  float alt_pred = alt_est + vel_z * dt;

  // --- Drift clamp / correction ---
  float err = baroAlt - alt_pred;
  if (fabsf(err) > 2.0f) {       // 2m threshold, tune
    vel_z *= 0.5f;               // dampen velocity instead of killing it
    alt_pred += err * 0.5f;      // pull prediction halfway toward baro
  }

  // --- Step 4: Complementary fuse with baro ---
  float alpha = 0.60f;   // tune
  alt_est = alpha * alt_pred + (1.0f - alpha) * baroAlt;

  return alt_est;
}

void buzz_it(uint16_t num, uint16_t delay_time = 500) {
  for (uint16_t i = 0; i < num; i++) {
    buzz_on(); // Buzzer ON
    vTaskDelay(pdMS_TO_TICKS(delay_time)); // Use FreeRTOS delay
    buzz_off(); // Buzzer OFF
    vTaskDelay(pdMS_TO_TICKS(delay_time));
  }
}

static inline float constrainFloat(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

// check sensor function
bool sensorsReady(){
  return BMP280_read(&bmpData) && MPU6050_read(&mpuData) && QMC5883P_read(&magData);
}

/////// TASKS /////// 
void watchdogTask(void* parameters) {
  vTaskDelay(pdMS_TO_TICKS(1000));
  IWDG_Init(); // watchdogtimer initialization for failsafes
  Serial.println("WDT init success!");

  const TickType_t interval = pdMS_TO_TICKS(200);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (WDT_isSafe()) {
      IWDG->KR = 0xAAAA;  // Refresh watchdog
    } else { 
      // Don’t refresh → MCU resets
      if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1)) == pdTRUE){
        Serial.println("Watchdog timed out");
        xSemaphoreGive(serialMutex);
      }
      buzz_on();
    }

    // debug print
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   Serial.print("WDT_flag: "); Serial.println(WDT_isSafe());
    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, interval);
  }
}

void blinkTask(void *pvParameters) {
  TickType_t interval = pdMS_TO_TICKS(1000); // 500 ms interval
  for(;;){
    GPIOC->BSRR = (1 << (BUILTIN_LED_PIN)); // LED off
    vTaskDelay(interval);
    GPIOC->BSRR = (1 << (BUILTIN_LED_PIN + 16)); // LED on
    vTaskDelay(interval);
  }
}

void readSensorsTask(void* Parameters) {  // 1 kHz
  const TickType_t intervalTicks = pdMS_TO_TICKS(1);  // 1ms = 1 kHz
  TickType_t lastWakeTime = xTaskGetTickCount();

  float local_altitude; 
  float ax, ay, az, wx, wy, wz, mx, my, mz; // Madgwick

  float dt;

  for (;;) {
    dt = intervalTicks * portTICK_PERIOD_MS / 1000.0f; // dt in seconds

    // read the BMP280 sensor
    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      // read MPU6050, BMP280, Magnetometer sensor: 
      if (sensorsReady()) {
        WDT_setSafe(true);

        // Successfully read MPU6050 data (acc_xyz is in g-units)
        ax = mpuData.ax;
        ay = mpuData.ay;
        az = mpuData.az;
        wx = mpuData.wx;
        wy = mpuData.wy;
        wz = mpuData.wz;
        // Raw data from BMP280
        mx = magData.mx;
        my = magData.my;
        mz = magData.mz;

        Butterworth2ndLPF_Update(&accelLPF, ax, ay, az); // Apply LPF: Fc = 50 Hz
        Butterworth2ndLPF_Update(&gyroLPF, wx, wy, wz); // Apply LPF: Fc = 30 Hz

        // LPF outputs
        ax = accelLPF.output_x; // LPF output for Accel X
        ay = accelLPF.output_y; // LPF output for Accel Y
        az = accelLPF.output_z; // LPF output for Accel Z
        wx = gyroLPF.output_x;  // LPF output for Gyro X
        wy = gyroLPF.output_y;  // LPF output for Gyro Y
        wz = gyroLPF.output_z;  // LPF output for Gyro Z

        local_altitude = bmpData.altitude; // Store altitude locally
      }else {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
          Serial.println("Error reading sensors");
        }
        // local_altitude = 0.0f; // Set to zero if read fails
        buzz_on();
        vTaskDelay(pdMS_TO_TICKS(100));
        buzz_off();
        WDT_setSafe(false);
      }
      xSemaphoreGive(wireMutex);
    }

    // update the Madgwick's data:
    if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      MadgwickSensorList[0] = ax; // Accel X
      MadgwickSensorList[1] = ay; // Accel Y
      MadgwickSensorList[2] = az; // Accel Z
      MadgwickSensorList[3] = wx; // Gyro X
      MadgwickSensorList[4] = wy; // Gyro Y
      MadgwickSensorList[5] = wz; // Gyro Z
      MadgwickSensorList[6] = mx; // Mag X
      MadgwickSensorList[7] = my; // Mag Y
      MadgwickSensorList[8] = mz; // Mag Z
      xSemaphoreGive(madgwickMutex);
    }

      // --- call complementary filter ---
    float fusedAlt = complementaryAltitude(local_altitude, ax, ay, az, dt);

    // use EMA filter again for the altitude
    static float altSmooth = 0.0f;
    float alpha = 0.9f; // high = smoother
    altSmooth = alpha * altSmooth + (1.0f - alpha) * fusedAlt;


    // Update shared data: BMP280 altitude
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      telemetry[3] = altSmooth; // Update altitude in telemetry
      xSemaphoreGive(telemetryMutex);
    }

    // // debug printf
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   Serial.print("Euler: ");
    //   Serial.print(madData.roll); Serial.print(", ");
    //   Serial.print(madData.pitch); Serial.print(", ");
    //   Serial.println(madData.yaw);

    //   Serial.print("Altitude: "); Serial.println(altSmooth);
    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, intervalTicks); 
  }
}

// Madgwick filter task (sensor fusion) 1 kHz
// void MadgwickTask(void* Parameters) {
//   const TickType_t intervalTicks = pdMS_TO_TICKS(4);  // 4ms max
//   TickType_t prevTick = xTaskGetTickCount();
//   TickType_t lastWakeTime = xTaskGetTickCount();

//   // local varibles for sensors data
//   float ax, ay, az, wx, wy, wz, mx, my, mz;

//   for (;;) {
//     TickType_t nowTick = xTaskGetTickCount();
//     TickType_t deltaTick = nowTick - prevTick;

//     if (deltaTick >= intervalTicks) {
//       float dt = deltaTick * portTICK_PERIOD_MS / 1000.0f;  // dt in seconds
//       prevTick = nowTick;

//       // read the sensors data
//       if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//         ax = MadgwickSensorList[0]; // Accel X
//         ay = MadgwickSensorList[1]; // Accel Y
//         az = MadgwickSensorList[2]; // Accel Z
//         wx = MadgwickSensorList[3]; // Gyro X
//         wy = MadgwickSensorList[4]; // Gyro Y
//         wz = MadgwickSensorList[5]; // Gyro Z
//         mx = MadgwickSensorList[6]; // Mag X
//         my = MadgwickSensorList[7]; // Mag Y
//         mz = MadgwickSensorList[8]; // Mag Z
//         xSemaphoreGive(madgwickMutex);
//       }

//       MadgwickFilterUpdate(&madData,
//         wx, wy, wz,
//         ax, ay, az,
//         mx, my, mz,
//         dt);

//       MadgwickGetEuler(&madData);

//       if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//         // units: 
//         eulerAngles[0] = madData.roll;
//         eulerAngles[1] = madData.pitch;
//         eulerAngles[2] = madData.yaw;
//         xSemaphoreGive(eulerAnglesMutex);
//       }

//       //// Print debug
//       // if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
//       //   Serial.print("Euler: ");
//       //   Serial.print(madData.roll); Serial.print(", ");
//       //   Serial.print(madData.pitch); Serial.print(", ");
//       //   Serial.println(madData.yaw);

//       //   // It prints what I expected (It works fine except for magData):
//       //   // Acc
//       //   // Serial.print("Accel: "); Serial.print(mpuData.ax); Serial.print(", ");
//       //   // Serial.print(ay); Serial.print(", "); Serial.println(az);
//       //   // Gyro
//       //   // Serial.print("Gyro: "); Serial.print(mpuData.wx); Serial.print(", ");
//       //   // Serial.print(wy); Serial.print(", "); Serial.println(wz);
//       //   // Mag
//       //   // Serial.print("Mag: "); Serial.print(magData.mx); Serial.print(", ");
//       //   // Serial.print(my); Serial.print(", "); Serial.println(mz);
//       //   xSemaphoreGive(serialMutex);
//       // }
//     }
//     vTaskDelayUntil(&lastWakeTime, intervalTicks);
//   }
// }

void PIDtask(void* Parameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t interval = pdMS_TO_TICKS(2); // 500 Hz
  int PID_RATIO = 5; // 500 Hz / 100 Hz = 5
  float dt = 0.002f;
  float dt_out = (float)(PID_RATIO) * dt;


  float altitude;
  float roll, pitch, yaw; // local variables for Euler Angles
  float throttle, rollInput, pitchInput, yawInput; // user inputs

  float rollRate, pitchRate, yawRate; // from sensors

  static bool altitudeLockSet = false;

  // EMA filter variables & constants
  static float throttleFiltered = 0.0f;
  static float rollInputFiltered = 0.0f;
  static float pitchInputFiltered = 0.0f;
  static float yawInputFiltered = 0.0f;

  const float alpha = 0.8f;  // adjust as needed

  // flags
  bool ALT_H = false; // Altitude Hold flag
  bool RTL = false; // Return to Launch flag
  bool Emergency_Landing = false; // Emergency Landing flag
  bool KILL_MOTORS = false; // Kill Motors flag

  // counter
  static int altCounter = 0; // altitude counter
  static int outer_loop_counter = 0;
  static int print_counter = 0; // for debugging

  float roll_rate_setpoint = 0.00f;
  float pitch_rate_setpoint = 0.00f;
  float yaw_rate_setpoint = 0.00f;

  // FUSION Variables:
  float ax, ay, az, wx, wy, wz, mx, my, mz;

  // landed flag or take off
  // Static timer and delay constant
  static uint16_t takeoff_counter = 0;
  static const uint16_t TAKEOFF_DELAY_COUNTS = 100;  // 0.5s at 100Hz (adjust as needed: 75 = 0.75s, 100 = 1s)


  // motors init state:
  float motor_cmd[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};
  float cmd_bias = 1000.0f;

  // Hold min for arming
  TIM2->CCR1 = (uint16_t)motor_cmd[0];
  TIM2->CCR2 = (uint16_t)motor_cmd[1];
  TIM2->CCR3 = (uint16_t)motor_cmd[2];
  TIM2->CCR4 = (uint16_t)motor_cmd[3];

  vTaskDelay(pdMS_TO_TICKS(3000)); // delay for arming motor

  for (;;){
    outer_loop_counter++;
    print_counter++;

    ////////////// FUSION: MADGWICK ////////////
    // read the sensors data
    if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      ax = MadgwickSensorList[0]; // Accel X
      ay = MadgwickSensorList[1]; // Accel Y
      az = MadgwickSensorList[2]; // Accel Z
      wx = MadgwickSensorList[3]; // Gyro X
      wy = MadgwickSensorList[4]; // Gyro Y
      wz = MadgwickSensorList[5]; // Gyro Z
      mx = MadgwickSensorList[6]; // Mag X
      my = MadgwickSensorList[7]; // Mag Y
      mz = MadgwickSensorList[8]; // Mag Z
      xSemaphoreGive(madgwickMutex);
    }

    MadgwickFilterUpdate(&madData,
      wx, wy, wz,
      ax, ay, az,
      mx, my, mz,
      dt);

    MadgwickGetEuler(&madData);

    if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      // units: 
      eulerAngles[0] = madData.roll;
      eulerAngles[1] = madData.pitch;
      eulerAngles[2] = madData.yaw;
      xSemaphoreGive(eulerAnglesMutex);
    }
    //////////////////////////////////////////////

    // read sensors:
    roll =  eulerAngles[0] * DEG_TO_RAD;
    pitch = eulerAngles[1] * DEG_TO_RAD;
    yaw =   eulerAngles[2] * DEG_TO_RAD;

    // read Madgwick's data: units: rad
    // unit: rad/s
    rollRate =  wx; // Gyro X
    pitchRate = wy; // Gyro Y
    yawRate =   wz; // Gyro Z

    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE){
      altitude = telemetry[3]; // Get altitude from telemetry
      xSemaphoreGive(telemetryMutex); // release the mutex
    }

    // read user input from nRF24L01 (use mutex):
    if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE){
      // read user input here (radians for angles, rad/s for yaw rate)
      throttle = inputList[0]; // Throttle
      yawInput = inputList[1]; // Yaw Rate input
      pitchInput = inputList[2]; // Pitch input
      rollInput = inputList[3]; // Roll input

      KILL_MOTORS = (inputList[4] != 0.0f);

      Emergency_Landing = (inputList[5] != 0.0f);
      // KILL_MOTORS = (inputList[6] != 0.0f);
      // ALT_H = (inputList[7] != 0.0f); 
      xSemaphoreGive(nRF24Mutex); // release the mutex
    }

    // ====== SHUTDOWN MOTORS ======
    if (KILL_MOTORS) {
      resetLyGAPID(&pidRoll);
      resetLyGAPID(&pidPitch);

      resetLyGAPID(&pidRollRate);
      resetLyGAPID(&pidPitchRate);
      resetLyGAPID(&pidYawRate);

      // set motors to 1ms
      TIM2->CCR1 = 1000;
      TIM2->CCR2 = 1000;
      TIM2->CCR3 = 1000;
      TIM2->CCR4 = 1000;
    }

    // ====== EMERGENCY LANDING ======
    if (Emergency_Landing) {
      rollInput = 0.0f;
      pitchInput = 0.0f;
      yawInput = 0.0f;
      
      //// TODO: implement emergency landing logic 
      // Take the current throttle and slowly decrease it while considering altitude
      // throttle = 500.0f; // Set a safe throttle for landing
    }

    // ====== ALTITUDE HOLD ======
    if (ALT_H) {
      altCounter++;
      if (altCounter >= 10){
        static float targetAltitude = 0.0f; // the last altitude state w/o controller
        if (!altitudeLockSet) {
          targetAltitude = altitude;  // lock on current altitude
          altitudeLockSet = true;
        }
        throttleFiltered = computePID(&pidThrottle, targetAltitude, altitude, dt);
      }
    }else{
      // to remove the CONFLICT between EMA & PID
      altitudeLockSet = false;
      throttleFiltered = alpha * throttle  + (1 - alpha) * throttleFiltered;
    }

    // Apply EMA filter:
    rollInputFiltered  = alpha * rollInput  + (1 - alpha) * rollInputFiltered;
    pitchInputFiltered = alpha * pitchInput + (1 - alpha) * pitchInputFiltered;
    yawInputFiltered   = alpha * yawInput   + (1 - alpha) * yawInputFiltered;

    // clamp
    rollInputFiltered = constrainFloat(rollInputFiltered, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    pitchInputFiltered = constrainFloat(pitchInputFiltered, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
    yawInputFiltered = constrainFloat(yawInputFiltered, -YAW_MAX, YAW_MAX);

    // PID start:
    // computing desired rates: (P-controller):
    if (outer_loop_counter >= PID_RATIO){ // 100 Hz
      // Already clamped by the LyGAPID function
      roll_rate_setpoint = computeLyGAPID_out(&pidRoll, rollInputFiltered, roll, dt_out); // deg/s
      pitch_rate_setpoint = computeLyGAPID_out(&pidPitch, pitchInputFiltered, pitch, dt_out); // deg/s

      // ===== IS LANDED LOGIC (disable I-term) =====
      float avg_motor_output = (motor_cmd[0] + motor_cmd[1] + motor_cmd[2] + motor_cmd[3]) / 4.0f;
      bool landed = (throttleFiltered < 1200.0f) && (avg_motor_output < 1200.0f); 

      float pid_landed_flag;

      if (landed) {
          // Definitely on ground — immediate reset
          takeoff_counter = 0;
          pid_landed_flag = 1.0f;  // blocks integration + triggers reset in PID
      } else {
          // In flight
          if (takeoff_counter < TAKEOFF_DELAY_COUNTS) {
            takeoff_counter++;
            pid_landed_flag = 1.0f;  // still block integration during grace period
          } else {
            pid_landed_flag = 0.0f;  // now allow full integration
          }
        }

      // Apply the SAME flag to all rate PID structs
      pidRollRate.landed  = pid_landed_flag;
      pidPitchRate.landed = pid_landed_flag;
      pidYawRate.landed   = pid_landed_flag;  

      outer_loop_counter = 0; // reset the counter
    }

    yaw_rate_setpoint = yawInputFiltered; // deg/s (YAW RATE CMD)

    // Inner loop: (full PID): 250 Hz
    float roll_rate_correction = computeLyGAPID_in(&pidRollRate, roll_rate_setpoint, rollRate, dt);
    float pitch_rate_correction = computeLyGAPID_in(&pidPitchRate, pitch_rate_setpoint, pitchRate, dt);
    float yaw_rate_correction = computeLyGAPID_yaw(&pidYawRate, yaw_rate_setpoint, yawRate, dt);

    float R_mix = constrainFloat(roll_rate_correction, -U_MAX_ROLL_RATE, U_MAX_ROLL_RATE);
    float P_mix = constrainFloat(pitch_rate_correction, -U_MAX_PITCH_RATE, U_MAX_PITCH_RATE);
    float Y_mix = constrainFloat(yaw_rate_correction, -U_MAX_YAW_RATE, U_MAX_YAW_RATE);

    // PID end.
    throttleFiltered = constrainFloat(throttleFiltered, 0.0f, 1000.0f);
    // Motor Mixer Algorithm (Props-out):
    motor_cmd[0] = cmd_bias + throttleFiltered + R_mix + P_mix - Y_mix; // Front Left
    motor_cmd[1] = cmd_bias + throttleFiltered - R_mix + P_mix + Y_mix; // Front Right
    motor_cmd[2] = cmd_bias + throttleFiltered + R_mix - P_mix + Y_mix; // Back Left
    motor_cmd[3] = cmd_bias + throttleFiltered - R_mix - P_mix - Y_mix; // Back Right
    
    // Failsafe & Limit motor outputs to the range [1000, 2000]:
    // Disable PID correction when throttle is low and drone is likely landed:
    if (!ALT_H && throttleFiltered < 30.0f) {
      resetLyGAPID(&pidRoll);
      resetLyGAPID(&pidPitch);

      resetLyGAPID(&pidRollRate);
      resetLyGAPID(&pidPitchRate);
      resetLyGAPID(&pidYawRate);

      for (int i = 0; i < 4; i++) {
        motor_cmd[i] = 1000.0f;
      }
    } else {
      // Dynamic mixer:
      if (motor_cmd[0] > 2000.0f || motor_cmd[1] > 2000.0f ||
          motor_cmd[2] > 2000.0f || motor_cmd[3] > 2000.0f) {
        // Find the maximum command
        float maxCmd = motor_cmd[0];
        for (int i = 1; i < 4; i++) {
          if (motor_cmd[i] > maxCmd) {
            maxCmd = motor_cmd[i];
          }
        }
        // Calculate the excess amount
        float excess = maxCmd - 2000.0f;
        // Reduce all commands by the excess amount
        for (int i = 0; i < 4; i++) {
          motor_cmd[i] -= excess;
        }
      }

      // Hard Clamp
      for (int i = 0; i < 4; i++) {
        motor_cmd[i] = constrainFloat(motor_cmd[i], 1000.0f, 2000.0f);
      }
    }

    // Motor Output:
    TIM2->CCR1 = (uint16_t)motor_cmd[0];
    TIM2->CCR2 = (uint16_t)motor_cmd[1];
    TIM2->CCR3 = (uint16_t)motor_cmd[2];
    TIM2->CCR4 = (uint16_t)motor_cmd[3];

    // Debugging output: Temporary -> uncomment it in deployment!
    if (print_counter >= 50){ // 100ms}
      // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){

      // // MOTOR output (unitless: hardware based)
      // Serial.print(motor1_output); Serial.print(", ");
      // Serial.print(motor2_output); Serial.print(", ");
      // Serial.print(motor3_output); Serial.print(", ");
      // Serial.println(motor4_output); 

      // Serial.println(throttle);

      // Roll mixer output:
      // Serial.print("Roll: "); Serial.print(roll * (180.0f / 3.1415f));
      // Serial.print(", R_mix: "); Serial.print(R_mix); Serial.print("Y: ");
      // Serial.println(yawRate);

      // Serial.print(roll * (180.0f / 3.1415f)); Serial.print(", ");
      // Serial.print(pitch * (180.0f / 3.1415f)); Serial.print(", ");
      // Serial.println(yaw * (180.0f / 3.1415f)); 

      // Serial.print("PI gains: "); Serial.println(pidRoll.Kp); 

      // Serial.print("PID gains: "); Serial.print(pidRollRate.Kp); Serial.print(", ");
      // Serial.print(pidRollRate.Ki); Serial.print(", "); Serial.println(pidRollRate.Kd); 

      // xSemaphoreGive(serialMutex);
      // }
      print_counter = 0; // reset the counter
    }
    vTaskDelayUntil(&lastWakeTime, interval); // Delay until the next cycle
  }
}

void RXtask(void* Parameters){
  // reply: roll, pitch, yaw, alt, rad status, 2{P, kp, ki, kd}, {Kp, Ki, Kd}
  // 32 bytes nRF24 max can handle, sent:  32 bytes, int16 = 2 bytes
  // int16_t local_telemetry[16] = {0, 0, 0, 0, 0, 0,0,0,0, 0,0,0,0, 0,0,0}; 
  int16_t local_telemetry[5] = {0, 0, 0, 0, 0}; 
  int mode = 0;
  // float kp, ki, kd, kill;
  float Tcmd = 0.0f;
  float Ycmd = 0.0f;
  float Pcmd = 0.0f;
  float Rcmd = 0.0f;
  float killcmd = 0.0f;
  int16_t rx_load[5] = {0, 0, 0, 0, 0};

  int counter_print = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Read + clear status flags
    uint8_t flags = radio.clearStatusFlags();

    // Handle data ready
    if (flags & RF24_RX_DR) {
      while (radio.available()) {
        local_telemetry[4] = 1; // 1 = connected, 0 = disconnected

        if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          local_telemetry[0] = (int16_t) eulerAngles[0];
          local_telemetry[1] = (int16_t) eulerAngles[1];
          local_telemetry[2] = (int16_t) eulerAngles[2];
          xSemaphoreGive(eulerAnglesMutex);
        }
        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          local_telemetry[3] = (int16_t) telemetry[3]; // altitude
          xSemaphoreGive(telemetryMutex);
        }

        // // Extract PID gains
        // local_telemetry[5] = (int16_t)(pidRoll.Kp * 100);
        // local_telemetry[6] = (int16_t)(pidRollRate.Kp * 100);
        // local_telemetry[7] = (int16_t)(pidRollRate.Ki * 100);
        // local_telemetry[8] = (int16_t)(pidRollRate.Kd * 100);
        
        // local_telemetry[9] = (int16_t)(pidPitch.Kp * 100);
        // local_telemetry[10] = (int16_t)(pidPitchRate.Kp * 100);
        // local_telemetry[11] = (int16_t)(pidPitchRate.Ki * 100);
        // local_telemetry[12] = (int16_t)(pidPitchRate.Kd * 100);

        // local_telemetry[13] = (int16_t)(pidYawRate.Kp * 100);
        // local_telemetry[14] = (int16_t)(pidYawRate.Ki * 100);
        // local_telemetry[15] = (int16_t)(pidYawRate.Kd * 100);

        // Write ACK payload BEFORE read
        radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
        
        // Read incoming data
        radio.read(rx_load, sizeof(rx_load));
        
        radio.startListening();

        // if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        //   Serial.print("[nRF24 RX] Received: ");
        //   for (int i = 0; i < 4; i++) {
        //     Serial.print(rx_load[i]); Serial.print(", ");
        //   }
        //   Serial.println(rx_load[4]);
        //   xSemaphoreGive(serialMutex);
        // }

        // // 1. store 
        // mode = (int)rx_load[0];
        // kp = (float)rx_load[1] / 100.00f; // divide by 100 to get the actual value
        // ki = (float)rx_load[2] / 100.00f;
        // kd = (float)rx_load[3] / 100.00f;

        // int16_t -> float data (scaled back by 1/100)
        // angle command expected in deg with 2 dec precision:
        Tcmd = (float)rx_load[0] * 10.0f; // (expected: 0-100 in (%) for 0-1000 us tick)
        Ycmd = (float)rx_load[1] / 100.0f;
        Pcmd = (float)rx_load[2] / 100.0f;
        Rcmd = (float)rx_load[3] / 100.0f;
        killcmd = (float)rx_load[4];

        // convert attitude command from Deg to Rad
        Ycmd = Ycmd * DEG_TO_RAD;
        Pcmd = Pcmd * DEG_TO_RAD;
        Rcmd = Rcmd * DEG_TO_RAD;

        // clamp it to the safe range 
        Tcmd = constrainFloat(Tcmd, THROTTLE_MIN, THROTTLE_MAX); // Throttle command
        Ycmd = constrainFloat(Ycmd, -YAW_MAX, YAW_MAX); // Yaw command (max: -180 to 180 deg/s)
        Pcmd = constrainFloat(Pcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX); // Pitch command (max: -50 to 50 deg)
        Rcmd = constrainFloat(Rcmd, -PITCH_ROLL_MAX, PITCH_ROLL_MAX); // Roll command (max: -50 to 50 deg)
        killcmd = constrainFloat(killcmd, 0.0f, 1.0f); // Kill command (0 or 1)

        // // clamp the PID gains
        // kp = constrainFloat(kp, 1.0f, 40.0f);
        // ki = constrainFloat(ki, 0.0f, 20.0f);
        // kd = constrainFloat(kd, 0.0f, 10.0f);

        // 2. update the pid params
        if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(1)) == pdTRUE) {  
          inputList[0] = Tcmd; // update throttle
          inputList[1] = Ycmd; // update yaw
          inputList[2] = Pcmd; // update pitch
          inputList[3] = Rcmd; // update roll
          inputList[4] = killcmd; // update kill flag in inputList
          xSemaphoreGive(nRF24Mutex);
        }

      }
    }

    // Optional: catch abnormal TX_ACK behavior
    if (flags & 0x10) {
      radio.flush_tx();  // clear stuck packet
    }
  }
}

// void RXtask(void* Parameters){
//   int16_t local_telemetry[5] = {0, 0, 0, 0, 0}; 
//   int16_t rx_load[5];

//   static uint32_t lastPacketTime = 0;
//   const uint32_t CONNECTION_TIMEOUT_MS = 500;  // 0.5 second timeout

//   // Persistent commands — these keep last good values
//   static float Tcmd = 0.0f;
//   static float Ycmd = 0.0f;
//   static float Pcmd = 0.0f;
//   static float Rcmd = 0.0f;
//   static float killcmd = 0.0f;

//   for (;;) {
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//     uint8_t flags = radio.clearStatusFlags();
//     bool packetReceived = false;

//     if (flags & RF24_RX_DR) {
//       while (radio.available()) {
//         packetReceived = true;
//         local_telemetry[4] = 1; // 1 = connected, 0 = disconnected

//         // Update telemetry (roll, pitch, yaw, alt)
//         if (xSemaphoreTake(eulerAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//           local_telemetry[0] = (int16_t)eulerAngles[0];
//           local_telemetry[1] = (int16_t)eulerAngles[1];
//           local_telemetry[2] = (int16_t)eulerAngles[2];
//           xSemaphoreGive(eulerAnglesMutex);
//         }
//         if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
//           local_telemetry[3] = (int16_t)telemetry[3];
//           xSemaphoreGive(telemetryMutex);
//         }

//         radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
//         radio.read(rx_load, sizeof(rx_load));
//         radio.startListening();

//         // Decode fresh commands
//         Tcmd    = constrainFloat((float)rx_load[0] * 10.0f, THROTTLE_MIN, THROTTLE_MAX);
//         Ycmd    = constrainFloat((float)rx_load[1] / 100.0f * DEG_TO_RAD, -YAW_MAX, YAW_MAX);
//         Pcmd    = constrainFloat((float)rx_load[2] / 100.0f * DEG_TO_RAD, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//         Rcmd    = constrainFloat((float)rx_load[3] / 100.0f * DEG_TO_RAD, -PITCH_ROLL_MAX, PITCH_ROLL_MAX);
//         killcmd = constrainFloat((float)rx_load[4], 0.0f, 1.0f);

//         lastPacketTime = millis();  // Refresh timeout
//       }
//     }

//     if (flags & 0x10) {
//       radio.flush_tx();
//     }

//     // === FAILSAFE UPDATE: Runs every task cycle ===
//     if (xSemaphoreTake(nRF24Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//       bool isConnected = (millis() - lastPacketTime < CONNECTION_TIMEOUT_MS);

//       // if (!isConnected && (local_telemetry[3] > 2.0f)) {
//       //   // Slowly reduce throttle for controlled descent
//       //   static float descentThrottle = Tcmd;
//       //   descentThrottle *= 0.98f; // Gradually reduce
//       //   inputList[0] = descentThrottle;
//       // }

//       if (isConnected) {
//         // Use fresh commands
//         inputList[0] = Tcmd;        // Throttle
//         inputList[1] = Ycmd;        // Yaw
//         inputList[2] = Pcmd;        // Pitch
//         inputList[3] = Rcmd;        // Roll
//         inputList[4] = killcmd;     // Kill flag
//       } else {
//         // === CONNECTION LOST: FORCE SAFE STATE ===
//         inputList[0] = 0.0f;         // Throttle = 0 (MOTORS WILL SPIN DOWN)
//         inputList[1] = 0.0f;         // No yaw input
//         inputList[2] = 0.0f;         // No pitch
//         inputList[3] = 0.0f;         // No roll
//         inputList[4] = 1.0f;         // Optional: FORCE KILL (if your code checks this)
//         // inputList[5] = 1.0f;          // Emergency Landing flag 
//       }

//       xSemaphoreGive(nRF24Mutex);
//     }
//   }
// }

//// TODO: find ADC pin for this task
void batteryMonitorTask(void* Parameters){
  TickType_t interval = pdMS_TO_TICKS(4000);
  // TickType_t lastWakeTime = x
  float batteryVoltage; // Variable to hold battery voltage

  for (;;){
    // Read battery voltage

    // Update shared data
    if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
      telemetry[5] = batteryVoltage; // Update battery voltage in telemetry
      xSemaphoreGive(telemetryMutex);
    }
    
    vTaskDelay(interval);
  }
}

////////// freeRTOS INIT
BaseType_t result;
void freeRTOS_tasks_init(void){
  result = xTaskCreate(
    watchdogTask,
    "Watchdog Task",
    64,
    NULL,
    PRIORITY_WDT,
    NULL
  );
  if (result != pdPASS) {
    Serial.println("Failed to create the WDT task!");
    while(1);
  }

  result = xTaskCreate(
    blinkTask,           // Task function
    "blinkTask",         // Name of the task
    64,                // Stack size in words
    NULL,                // Task input parameter
    PRIORITY_BLINK,                   // Priority
    NULL                 // Task handle
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create blinkTask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    readSensorsTask,
    "read Sensors Task",
    128, // Stack size in words
    NULL,
    PRIORITY_SENSOR_READ,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create readSensorsTask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    PIDtask,
    "PID and Fusion task",
    256,
    NULL,
    PRIORITY_PID_FUSION,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create PIDtask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    RXtask,
    "nRF24 RX task",
    256,
    NULL,
    PRIORITY_RADIO,
    &radioTaskHandle
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create RXtask");
    while (1); // Infinite loop to indicate failure
  }

  // result = xTaskCreate(
  //   batteryMonitorTask,
  //   "Battery Monitor Task",
  //   256,
  //   NULL,
  //   1,
  //   NULL
  // );
  // if (result != pdPASS) {
  //   // Handle task creation failure
  //   Serial.println("Failed to create batteryMonitorTask");
  //   while (1); // Infinite loop to indicate failure
  // }
}



// Priority tasks are all set to 1 (working) - i will try to change for the better: status: trying...

//// TODO: 
// - PID, Fusion, Sensor read = 500 Hz, 500 Hz, 1 kHz (done)
// - PID highest priority without sacrificing stability of other tasks (...)
// - Cut the motor when the signal is lost for more than 0.5 second
// - fix the integral windup issue (...)
// - PID gains ranges