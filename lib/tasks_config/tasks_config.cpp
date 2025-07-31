#include "tasks_config.h"

#include <STM32FreeRTOS.h>
#include <RF24.h>
#include <RadioLib.h>

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

void disableMotors(void) {
  TIM2->CCR1 = TIM2->CCR2 = TIM2->CCR3 = TIM2->CCR4 = 0;
  TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
}

void enableMotors(void) {
  TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
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
  const TickType_t intervalTicks = pdMS_TO_TICKS(4);  // 5ms ≈ 200Hz
  TickType_t lastWakeTime = xTaskGetTickCount();

  float local_altitude; 
  float ax, ay, az, wx, wy, wz, mx, my, mz; // Madgwick

  for (;;) {
    // read the BMP280 sensor
    if (xSemaphoreTake(wireMutex, portMAX_DELAY)){
      // read MPU6050, BMP280, Magnetometer sensor: 
      if (sensorsReady()) {
        WDT_setSafe(true);

        // Successfully read MPU6050 data
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
        local_altitude = 0.0f; // Set to zero if read fails
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

    // Update shared data: BMP280 altitude
    if (xSemaphoreTake(loraMutex, portMAX_DELAY)){
      loraList[2] = local_altitude; // Update altitude in loraList
      xSemaphoreGive(loraMutex);
    }

    //// debug printf
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   Serial.print("Euler: ");
    //   Serial.print(madData.roll); Serial.print(", ");
    //   Serial.print(madData.pitch); Serial.print(", ");
    //   Serial.println(madData.yaw);

    //   // Serial.print("Altitude: "); Serial.println(local_altitude);
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

  static bool motorsEnabled = true;
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

  for (;;){
    TickType_t currentTime = xTaskGetTickCount();
    float dt = (currentTime - previousTime) * portTICK_PERIOD_MS / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f; // clamp dt to a safe range
    previousTime = currentTime;

    // read sensors:
    if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)){
      roll = eulerAngles[0];
      pitch = eulerAngles[1];
      yaw = eulerAngles[2];
      xSemaphoreGive(eulerAnglesMutex); // release the mutex
    }
    if (xSemaphoreTake(loraMutex, portMAX_DELAY)){
      altitude = loraList[2]; // Get altitude from loraList
      xSemaphoreGive(loraMutex); // release the mutex
    }

    // read user input from nRF24L01 (use mutex):
    if (xSemaphoreTake(nRF24Mutex, portMAX_DELAY)){
      // read user input here
      throttle = inputList[0]; // Throttle
      rollInput = inputList[1]; // Roll input
      pitchInput = inputList[2]; // Pitch input
      yawInput = inputList[3]; // Yaw input

      RTL = (inputList[4] != 0.0f);
      Emergency_Landing = (inputList[5] != 0.0f);
      KILL_MOTORS = (inputList[6] != 0.0f);
      ALT_H = (inputList[7] != 0.0f); 
      xSemaphoreGive(nRF24Mutex); // release the mutex
    }

    // ====== SHUTDOWN MOTORS ======
    if (KILL_MOTORS) {
      if (motorsEnabled) {
        resetPID(&pidRoll);
        resetPID(&pidPitch);
        resetPID(&pidYaw);

        // Only disable if currently enabled
        disableMotors();  // your wrapper
        motorsEnabled = false;
      }

      // stay in off state
      vTaskDelayUntil(&lastWakeTime, interval);
      continue;

    } else {
      if (!motorsEnabled) {
        // Only re-enable if previously disabled
        enableMotors();  // your wrapper
        motorsEnabled = true;
      }
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
      static float targetAltitude = 0.0f;
      if (!altitudeLockSet) {
        targetAltitude = altitude;  // lock on current altitude
        altitudeLockSet = true;
      }
      throttleFiltered = computePID(&pidThrottle, targetAltitude, altitude, dt);
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
    float roll_correction = computePID(&pidRoll, rollInputFiltered, roll, dt);
    float pitch_correction = computePID(&pidPitch, pitchInputFiltered, pitch, dt);
    float yaw_correction = computePID(&pidYaw, yawInputFiltered, yaw, dt);
    // float throttle_correction = computePID(&pidThrottle, throttleFiltered, throttle, dt);

    // Motor Mixer Algorithm (Props-out), (not yet tested):
    float motor1_output = throttleFiltered + roll_correction + pitch_correction + yaw_correction; // Front Left
    float motor2_output = throttleFiltered - roll_correction + pitch_correction - yaw_correction; // Front Right
    float motor3_output = throttleFiltered + roll_correction - pitch_correction - yaw_correction; // Back Left
    float motor4_output = throttleFiltered - roll_correction - pitch_correction + yaw_correction; // Back Right
    
    // Failsafe & Limit motor outputs to the range [1000, 2000]:
    // Disable PID correction when throttle is low and drone is likely landed:
    if (!ALT_H && throttleFiltered < 1050.0f) {
      resetPID(&pidRoll);
      resetPID(&pidPitch);
      resetPID(&pidYaw);
      motor1_output = motor2_output = motor3_output = motor4_output = 1000.0f;
    } else {
      motor1_output = constrainFloat(motor1_output, 1000.0f, 2000.0f);
      motor2_output = constrainFloat(motor2_output, 1000.0f, 2000.0f);
      motor3_output = constrainFloat(motor3_output, 1000.0f, 2000.0f);
      motor4_output = constrainFloat(motor4_output, 1000.0f, 2000.0f);
    }

    // Motor Output:
    TIM2->CCR1 = motor1_output; // TIM2_CH1
    TIM2->CCR2 = motor2_output; // TIM2_CH2
    TIM2->CCR3 = motor3_output; // TIM2_CH3
    TIM2->CCR4 = motor4_output; // TIM2_CH4

    // Debugging output: Temporary -> uncomment it in deployment!
    // if (xSemaphoreTake(serialMutex, portMAX_DELAY)){
    //   Serial.print("M1: "); Serial.print(motor1_output); Serial.print(" | ");
    //   Serial.print("M2: "); Serial.print(motor2_output); Serial.print(" | ");
    //   Serial.print("M3: "); Serial.print(motor3_output); Serial.print(" | ");
    //   Serial.print("M4: "); Serial.println(motor4_output);

    //   //// raw angles: 
    //   // Serial.print("Angles: "); Serial.print(roll); Serial.print(" | ");
    //   // Serial.print(pitch); Serial.print(" | "); Serial.println(yaw);

    //   //// Corrected Angles
    //   // Serial.print("Corrected: "); Serial.print(roll_correction); Serial.print(", ");
    //   // Serial.print(pitch_correction); Serial.print(", "); Serial.println(yaw_correction);

    //   xSemaphoreGive(serialMutex);
    // }

    vTaskDelayUntil(&lastWakeTime, interval); // Delay until the next cycle
  }
}

// nRF24 RX task
//// TODO: test it!!!!
void RXtask(void* Parameters){
  int16_t telemetry[5] = {0, 0, 0, 0, 0}; // Telemetry data to send back
  // temporary for testing: {roll, pitch, yaw, altitude}
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for IRQ

    uint8_t flags = radio.clearStatusFlags();

    if (flags & RF24_RX_DR) {
      while (radio.available()) {
        uint8_t len = radio.getDynamicPayloadSize();
        char buffer[33] = {0};  // one extra for null-terminator
        radio.read(&buffer, len);

        if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
          Serial.print("[nRF24 RX] Received: ");
          Serial.println(buffer);
          xSemaphoreGive(serialMutex);
        }

        // use mutex and read the data for telemetry
        if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)) {
          telemetry[0] = (int16_t) eulerAngles[0]; // roll
          telemetry[1] = (int16_t) eulerAngles[1]; // pitch
          telemetry[2] = (int16_t) eulerAngles[2]; // yaw
          xSemaphoreGive(eulerAnglesMutex);
        }
        if (xSemaphoreTake(loraMutex, portMAX_DELAY)){
          telemetry[3] = (int16_t) loraList[2]; // altitude
          xSemaphoreGive(loraMutex);
        }
        
        // Send as ACK payload
        radio.writeAckPayload(PIPE_INDEX, telemetry, sizeof(telemetry));

        //// TODO: process the received data:
      }
    }

    if (flags & RF24_TX_DF) {
      radio.flush_tx(); // not used on RX-only but good practice
    }
  }
}

// // //// TODO: implement
// // void loraTXtask(void* Parameters){
// //   TickType_t interval = pdMS_TO_TICKS(1000);

// //   for (;;){

// //     vTaskDelay(interval);
// //   }
// // }

//// TODO: find ADC pin for this task
void batteryMonitorTask(void* Parameters){
  TickType_t interval = pdMS_TO_TICKS(4000);
  // TickType_t lastWakeTime = x
  float batteryVoltage; // Variable to hold battery voltage

  for (;;){
    // Read battery voltage

    // Update shared data
    if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {
      loraList[5] = batteryVoltage; // Update battery voltage in loraList
      xSemaphoreGive(loraMutex);
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

  // // xTaskCreate(
  // //   loraTXtask,
  // //   "LoRa TX Task",
  // //   128,
  // //   NULL,
  // //   1,
  // //   NULL
  // // );

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