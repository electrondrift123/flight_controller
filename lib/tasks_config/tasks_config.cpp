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
#include "WDT.h"

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
      if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
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

void readSensorsTask(void* Parameters) {
  const TickType_t intervalTicks = pdMS_TO_TICKS(4);  // 4ms ≈ 250Hz
  TickType_t lastWakeTime = xTaskGetTickCount();

  float local_altitude; 
  float ax, ay, az, wx, wy, wz, mx, my, mz; // Madgwick

  float dt;

  for (;;) {
    dt = intervalTicks * portTICK_PERIOD_MS / 1000.0f; // dt in seconds

    // read the BMP280 sensor
    if (xSemaphoreTake(wireMutex, portMAX_DELAY)){
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
        mx = magData.mx;
        my = magData.my;
        mz = magData.mz;

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
    if (xSemaphoreTake(madgwickMutex, portMAX_DELAY)){
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
    if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)){
      telemetry[3] = altSmooth; // Update altitude in telemetry
      xSemaphoreGive(telemetryMutex);
    }

    // // debug printf
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
      // Serial.print("Euler: ");
      // Serial.print(madData.roll); Serial.print(", ");
      // Serial.print(madData.pitch); Serial.print(", ");
      // Serial.println(madData.yaw);

    //   Serial.print("Altitude: "); Serial.println(altSmooth);
    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, intervalTicks); 
  }
}

// Madgwick filter task (sensor fusion)
void MadgwickTask(void* Parameters) {
  const TickType_t intervalTicks = pdMS_TO_TICKS(5);  // 5ms ≈ 200Hz
  TickType_t prevTick = xTaskGetTickCount();
  TickType_t lastWakeTime = xTaskGetTickCount();

  // local varibles for sensors data
  float ax, ay, az, wx, wy, wz, mx, my, mz;

  for (;;) {
    TickType_t nowTick = xTaskGetTickCount();
    TickType_t deltaTick = nowTick - prevTick;

    if (deltaTick >= intervalTicks) {
      float dt = deltaTick * portTICK_PERIOD_MS / 1000.0f;  // dt in seconds
      prevTick = nowTick;

      // read the sensors data
      if (xSemaphoreTake(madgwickMutex, portMAX_DELAY)) {
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

      if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)) {
        // units: 
        eulerAngles[0] = madData.roll;
        eulerAngles[1] = madData.pitch;
        eulerAngles[2] = madData.yaw;
        xSemaphoreGive(eulerAnglesMutex);
      }

      //// Print debug
      // if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
      //   Serial.print("Euler: ");
      //   Serial.print(madData.roll); Serial.print(", ");
      //   Serial.print(madData.pitch); Serial.print(", ");
      //   Serial.println(madData.yaw);

      //   // It prints what I expected (It works fine except for magData):
      //   // Acc
      //   // Serial.print("Accel: "); Serial.print(mpuData.ax); Serial.print(", ");
      //   // Serial.print(ay); Serial.print(", "); Serial.println(az);
      //   // Gyro
      //   // Serial.print("Gyro: "); Serial.print(mpuData.wx); Serial.print(", ");
      //   // Serial.print(wy); Serial.print(", "); Serial.println(wz);
      //   // Mag
      //   // Serial.print("Mag: "); Serial.print(magData.mx); Serial.print(", ");
      //   // Serial.print(my); Serial.print(", "); Serial.println(mz);
      //   xSemaphoreGive(serialMutex);
      // }
    }
    vTaskDelayUntil(&lastWakeTime, intervalTicks);
  }
}

// Angle Mode
void PIDtask(void* Parameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t previousTime = lastWakeTime; // Initialize previous time
  TickType_t interval = pdMS_TO_TICKS(5); // 200 Hz

  float altitude;
  float roll, pitch, yaw; // local variables for Euler Angles
  float throttle, rollInput, pitchInput, yawInput; // user inputs

  float rollRate, pitchRate, yawRate; // from sensors

  static bool altitudeLockSet = false;

  // EMA filter variables & constants
  static float throttleFiltered = 1000.0f;
  static float rollInputFiltered = 0.0f;
  static float pitchInputFiltered = 0.0f;
  static float yawInputFiltered = 0.0f;

  const float alpha = 0.2f;  // adjust as needed

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

  vTaskDelay(pdMS_TO_TICKS(3000)); // delay for arming motor

  for (;;){
    TickType_t currentTime = xTaskGetTickCount();
    float dt = (currentTime - previousTime) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f; // clamp dt to a safe range
    previousTime = currentTime;

    outer_loop_counter++;
    print_counter++;

    // read sensors:
    if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)){
      // units: deg
      // convert: deg -> rad
      roll =  eulerAngles[0] * DEG_TO_RAD;
      pitch = eulerAngles[1] * DEG_TO_RAD;
      yaw =   eulerAngles[2] * DEG_TO_RAD;
      xSemaphoreGive(eulerAnglesMutex); // release the mutex
    }
    if (xSemaphoreTake(madgwickMutex, portMAX_DELAY)){
      // read Madgwick's data: units: rad
      // unit: rad/s
      rollRate =  MadgwickSensorList[3]; // Gyro X
      pitchRate = MadgwickSensorList[4]; // Gyro Y
      yawRate =   MadgwickSensorList[5]; // Gyro Z
      xSemaphoreGive(madgwickMutex); // release the mutex
    }
    if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)){
      altitude = telemetry[3]; // Get altitude from telemetry
      xSemaphoreGive(telemetryMutex); // release the mutex
    }

    // read user input from nRF24L01 (use mutex):
    if (xSemaphoreTake(nRF24Mutex, portMAX_DELAY)){
      // read user input here
      // throttle = inputList[0]; // Throttle
      // rollInput = inputList[1]; // Roll input
      // pitchInput = inputList[2]; // Pitch input
      // yawInput = inputList[3]; // Yaw input
      throttle = 1400.0f; // Throttle
      rollInput = 0.0f; // Roll input
      pitchInput = 0.0f; // Pitch input
      yawInput = 0.0f; // Yaw input

      // RTL = (inputList[4] != 0.0f);
      Emergency_Landing = (inputList[5] != 0.0f);
      // KILL_MOTORS = (inputList[6] != 0.0f);
      KILL_MOTORS = (inputList[4] != 0.0f);
      ALT_H = (inputList[7] != 0.0f); 
      xSemaphoreGive(nRF24Mutex); // release the mutex
    }

    // ====== SHUTDOWN MOTORS ======
    if (KILL_MOTORS) {
      resetPID(&pidRoll);
      resetPID(&pidPitch);
      resetPID(&pidYaw);

      resetPID(&pidRollRate);
      resetPID(&pidPitchRate);
      resetPID(&pidYawRate);

      // set motors to 1ms
      TIM2->CCR1 = 1000;
      TIM2->CCR2 = 1000;
      TIM2->CCR3 = 1000;
      TIM2->CCR4 = 1000;
    }

    // ====== RETURN TO LAUNCH ======
    if (RTL){
      rollInput = 0.0f;
      pitchInput = 0.0f;

      float homeYaw = 0.0f; // Replace with actual value when GPS is added
      yawInput = homeYaw;

      //// TODO: implement RTL logic
    }

    // ====== EMERGENCY LANDING ======
    if (Emergency_Landing) {
      rollInput = 0.0f;
      pitchInput = 0.0f;
      yawInput = 0.0f;
      
      //// TODO: implement emergency landing logic 
      // Take the current throttle and slowly decrease it while considering altitude
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

    // PID start:
    // Outer loop (30ms):
    // computing desired rates: (only Kp, P-controller):
    if (outer_loop_counter >= 6){ // 33.33 Hz
      roll_rate_setpoint = computePID(&pidRoll, rollInputFiltered, roll, 6.0f * dt); // deg/s
      pitch_rate_setpoint = computePID(&pidPitch, pitchInputFiltered, pitch, 6.0f * dt); // deg/s
      yaw_rate_setpoint = computePID(&pidYaw, yawInputFiltered, yaw, 6.0f * dt); // deg/s

      outer_loop_counter = 0; // reset the counter
    }

    // Inner loop: (full PID): 200 Hz
    float roll_rate_correction = computePID(&pidRollRate, roll_rate_setpoint, rollRate, dt);
    float pitch_rate_correction = computePID(&pidPitchRate, pitch_rate_setpoint, pitchRate, dt);
    float yaw_rate_correction = computePID(&pidYawRate, yaw_rate_setpoint, yawRate, dt);

    // PID end.
    throttleFiltered = constrainFloat(throttleFiltered, 1400.0f, 1800.0f);
    //// TODO: add gain for the rate corrections
    // Motor Mixer Algorithm (Props-out), (not yet tested):
    float motor1_output = throttleFiltered + roll_rate_correction + pitch_rate_correction + yaw_rate_correction; // Front Left
    float motor2_output = throttleFiltered - roll_rate_correction + pitch_rate_correction - yaw_rate_correction; // Front Right
    float motor3_output = throttleFiltered + roll_rate_correction - pitch_rate_correction - yaw_rate_correction; // Back Left
    float motor4_output = throttleFiltered - roll_rate_correction - pitch_rate_correction + yaw_rate_correction; // Back Right
    
    // Failsafe & Limit motor outputs to the range [1000, 2000]:
    // Disable PID correction when throttle is low and drone is likely landed:
    if (!ALT_H && throttleFiltered < 1100.0f) {
      resetPID(&pidRoll);
      resetPID(&pidPitch);
      resetPID(&pidYaw);

      resetPID(&pidRollRate);
      resetPID(&pidPitchRate);
      resetPID(&pidYawRate);
      motor1_output = motor2_output = motor3_output = motor4_output = 1000.0f;
    } else {
      motor1_output = constrainFloat(motor1_output, 1000.0f, 2000.0f);
      motor2_output = constrainFloat(motor2_output, 1000.0f, 2000.0f);
      motor3_output = constrainFloat(motor3_output, 1000.0f, 2000.0f);
      motor4_output = constrainFloat(motor4_output, 1000.0f, 2000.0f);
    }

    // Motor Output:
    TIM2->CCR1 = (uint16_t)motor1_output;
    TIM2->CCR2 = (uint16_t)motor2_output;
    TIM2->CCR3 = (uint16_t)motor3_output;
    TIM2->CCR4 = (uint16_t)motor4_output;

    // // Debugging output: Temporary -> uncomment it in deployment!
    if (print_counter >= 20){ // 100ms}
      // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
      // // PID tuning Starts (FC only)
      // // roll: [setpoint, actual, correction, current time in ms, P_val]
      // Serial.print("0"); Serial.print(", ");
      // Serial.print(roll); Serial.print(", "); 
      // Serial.print(roll_correction); Serial.print(", ");
      // Serial.print(currentTime * portTICK_PERIOD_MS); Serial.print(", ");
      // Serial.println("1.00");

      // Serial.println(roll_rate_correction);

      Serial.print(roll); Serial.print(", ");
      Serial.print(motor1_output); Serial.print(", ");
      Serial.print(motor2_output); Serial.print(", ");
      Serial.print(motor3_output); Serial.print(", ");
      Serial.println(motor4_output); 

      // xSemaphoreGive(serialMutex);
      // }
      print_counter = 0; // reset the counter
    }
    vTaskDelayUntil(&lastWakeTime, interval); // Delay until the next cycle
  }
}

// motor tester task
void MotorTest(void *Parameters){
  // Hold min for arming
  TIM2->CCR1 = 1000;
  TIM2->CCR2 = 1000;
  TIM2->CCR3 = 1000;
  TIM2->CCR4 = 1000;
  vTaskDelay(pdMS_TO_TICKS(3000));   // wait for the beeps to finish

  for(;;){
    for (int i = 0; i <= 200; i++){
      TIM2->CCR1 = 1300 + i;
      TIM2->CCR2 = 1300 + i;
      TIM2->CCR3 = 1300 + i;
      TIM2->CCR4 = 1300 + i;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 200; i >= 0; i--){
      TIM2->CCR1 = 1500 - i;
      TIM2->CCR2 = 1500 - i;
      TIM2->CCR3 = 1500 - i;
      TIM2->CCR4 = 1500 - i;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void RXtask(void* Parameters){
  int16_t local_telemetry[5] = {0, 0, 0, 0, 0};
  int mode = 1;
  float kp, ki, kd, kill;
  int16_t rx_load[5];

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Read + clear status flags
    uint8_t flags = radio.clearStatusFlags();

    // Handle data ready
    if (flags & RF24_RX_DR) {
      while (radio.available()) {
        local_telemetry[4] = 1; // 1 = connected, 0 = disconnected

        if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)) {
          local_telemetry[0] = (int16_t) eulerAngles[0];
          local_telemetry[1] = (int16_t) eulerAngles[1];
          local_telemetry[2] = (int16_t) eulerAngles[2];
          xSemaphoreGive(eulerAnglesMutex);
        }
        if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
          local_telemetry[3] = (int16_t) telemetry[3];
          xSemaphoreGive(telemetryMutex);
        }

        // Write ACK payload BEFORE read
        radio.writeAckPayload(PIPE_INDEX, local_telemetry, sizeof(local_telemetry));
        
        // Read incoming data
        radio.read(rx_load, sizeof(rx_load));
        
        radio.startListening();

        // if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
        //   Serial.print("[nRF24 RX] Received: ");
        //   for (int i = 0; i < 4; i++) {
        //     Serial.print(rx_load[i]); Serial.print(", ");
        //   }
        //   Serial.println(rx_load[4]);
        //   xSemaphoreGive(serialMutex);
        // }

        // 1. store 
        mode = (int)rx_load[0];
        kp = (float)rx_load[1] / 100.00f; // divide by 100 to get the actual value
        ki = (float)rx_load[2] / 100.00f;
        kd = (float)rx_load[3] / 100.00f;
        kill = (float)rx_load[4];

        // clamp the PID gains
        kp = constrainFloat(kp, 1.0f, 40.0f);
        ki = constrainFloat(ki, 0.0f, 20.0f);
        kd = constrainFloat(kd, 0.0f, 10.0f);

        // 2. update the pid params
        if (xSemaphoreTake(nRF24Mutex, portMAX_DELAY)){
          // mode = inputList[0];
          // kp = inputList[1];
          // ki = inputList[2];
          // kd = inputList[3];
          inputList[4] = kill; // update kill flag in inputList
          xSemaphoreGive(nRF24Mutex);
        }

        if (mode == 1){ // roll only
          pidRollRate.kp = kp;
          pidRollRate.ki = ki;
          pidRollRate.kd = kd;
        }
        if (mode == 2){ // pitch only
          pidPitchRate.kp = kp;
          pidPitchRate.ki = ki;
          pidPitchRate.kd = kd;
        }
        if (mode == 3){ // yaw only
          pidYawRate.kp = kp;
          pidYawRate.ki = ki;
          pidYawRate.kd = kd;
        }
      }
    }

    // Optional: catch abnormal TX_ACK behavior
    if (flags & 0x10) {
      radio.flush_tx();  // clear stuck packet
    }
  }
}


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
    1,
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
    1,                   // Priority
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
    1,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create readSensorsTask");
    while (1); // Infinite loop to indicate failure
  }

  // sensors data reading task
  result = xTaskCreate(
    MadgwickTask,
    "MadgwickTask", // Task name
    256,               // Stack size in words
    NULL,
    1,                 // Task priority
    NULL               // Task handle
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create MadgwickTask");
    while (1); // Infinite loop to indicate failure
  }

  result = xTaskCreate(
    PIDtask,
    "PID task",
    256,
    NULL,
    1,
    NULL
  );
  if (result != pdPASS) {
    // Handle task creation failure
    Serial.println("Failed to create PIDtask");
    while (1); // Infinite loop to indicate failure
  }

  // result = xTaskCreate(
  //   MotorTest,
  //   "motor testing",
  //   128,
  //   NULL,
  //   1,
  //   NULL
  // );
  // if (result != pdPASS) {
  //   // Handle task creation failure
  //   Serial.println("Failed to create motor testing task");
  //   while (1); // Infinite loop to indicate failure
  // }

  result = xTaskCreate(
    RXtask,
    "nRF24 RX task",
    256,
    NULL,
    1,
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



// Priority tasks are all set to 1 