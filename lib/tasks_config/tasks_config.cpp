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

#include "PID.h"

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


void blinkTask(void *pvParameters) {
  TickType_t interval = pdMS_TO_TICKS(1000); // 500 ms interval
  for(;;){
    GPIOC->BSRR = (1 << (BUILTIN_LED_PIN)); // LED ON
    vTaskDelay(interval);
    GPIOC->BSRR = (1 << (BUILTIN_LED_PIN + 16)); // LED OFF
    vTaskDelay(interval);
  }
}

// check sensor function
bool sensorsReady(){
  return BMP280_read(&bmpData) && MPU6050_read(&mpuData) && QMC5883P_read(&magData);
}

void MadgwickTask(void* Parameters) {
  const TickType_t intervalTicks = pdMS_TO_TICKS(5);  // 5ms ≈ 200Hz
  TickType_t prevTick = xTaskGetTickCount();

  for (;;) {
    TickType_t nowTick = xTaskGetTickCount();
    TickType_t deltaTick = nowTick - prevTick;

    if (deltaTick >= intervalTicks) {
      float dt = deltaTick * portTICK_PERIOD_MS / 1000.0f;  // dt in seconds
      prevTick = nowTick;

      if (sensorsReady()) {
        if (xSemaphoreTake(wireMutex, portMAX_DELAY)) {

          MadgwickFilterUpdate(&madData,
            mpuData.wx, mpuData.wy, mpuData.wz,
            mpuData.ax, mpuData.ay, mpuData.az,
            magData.mx, magData.my, magData.mz,
            dt);

          MadgwickGetEuler(&madData);

          if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)) {
            eulerAngles[0] = madData.roll;
            eulerAngles[1] = madData.pitch;
            eulerAngles[2] = madData.yaw;
            xSemaphoreGive(eulerAnglesMutex);
          }

          if (xSemaphoreTake(serialMutex, portMAX_DELAY)) {
            Serial.print("Euler: ");
            Serial.print(madData.roll); Serial.print(", ");
            Serial.print(madData.pitch); Serial.print(", ");
            Serial.println(madData.yaw);
            xSemaphoreGive(serialMutex);
          }

          xSemaphoreGive(wireMutex);
        }
      }
    }
    // a slight delay
    vTaskDelay(5);
  }
}

void PIDtask(void* Parameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t previousTime = lastWakeTime; // Initialize previous time
  TickType_t interval = pdMS_TO_TICKS(50); // 200 Hz

  float roll, pitch, yaw; // local variables for Euler Angles
  float throttle, rollInput, pitchInput, yawInput; // user inputs

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
    // read user input from nRF24L01 (use mutex):
    if (xSemaphoreTake(nRF24Mutex, portMAX_DELAY)){
      // read user input here
      throttle = inputList[0]; // Throttle
      rollInput = inputList[1]; // Roll input
      pitchInput = inputList[2]; // Pitch input
      yawInput = inputList[3]; // Yaw input
      xSemaphoreGive(nRF24Mutex); // release the mutex
    }

    // PID start:
    float roll_correction = computePID(&pidRoll, rollInput, roll, dt);
    float pitch_correction = computePID(&pidPitch, pitchInput, pitch, dt);
    float yaw_correction = computePID(&pidYaw, yawInput, yaw, dt);

    // Motor Mixer Algorithm (Props-out), (not yet tested):
    float motor1_output = throttle + roll_correction + pitch_correction + yaw_correction; // Front Left
    float motor2_output = throttle - roll_correction + pitch_correction - yaw_correction; // Front Right
    float motor3_output = throttle + roll_correction - pitch_correction - yaw_correction; // Back Left
    float motor4_output = throttle - roll_correction - pitch_correction + yaw_correction; // Back Right
    
    // Failsafe & Limit motor outputs to the range [1000, 2000]:
    if (throttle < 1050.0f) {
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

    vTaskDelayUntil(&lastWakeTime, interval); // Delay until the next cycle
  }
}

void RXtask(void* Parameters){
  TickType_t interval = pdMS_TO_TICKS(1000);

  for (;;){

    vTaskDelay(interval);
  }
}

void loraTXtask(void* Parameters){
  TickType_t interval = pdMS_TO_TICKS(1000);

  for (;;){

    vTaskDelay(interval);
  }
}

void kalmanTask(void* Parameters){
  TickType_t interval = pdMS_TO_TICKS(1000);

  for (;;){

    vTaskDelay(interval);
  }
}


void freeRTOS_tasks_init(void){
  xTaskCreate(
    blinkTask,           // Task function
    "blinkTask",         // Name of the task
    128,                // Stack size in words
    NULL,                // Task input parameter
    1,                   // Priority
    NULL                 // Task handle
  );

  // sensors data reading task
  xTaskCreate(
    MadgwickTask,
    "MadgwickTask", // Task name
    256,               // Stack size in words
    NULL,
    1,                 // Task priority
    NULL               // Task handle
  );

  xTaskCreate(
    PIDtask,
    "PID task",
    256,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    kalmanTask,
    "Kalman filter task for altitude",
    256,
    NULL,
    1, 
    NULL
  );

  xTaskCreate(
    RXtask,
    "nRF24 RX task",
    128,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    loraTXtask,
    "LoRa TX Task",
    128,
    NULL,
    1,
    NULL
  );
}

