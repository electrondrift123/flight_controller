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
  TickType_t interval = pdMS_TO_TICKS(50); // 200 Hz
  float roll, pitch, yaw; // local variables for Euler Angles
  float dt = 0.005f;

  for (;;){
    // read sensors:
    if (xSemaphoreTake(eulerAnglesMutex, portMAX_DELAY)){
      roll = eulerAngles[0];
      pitch = eulerAngles[1];
      yaw = eulerAngles[2];
      xSemaphoreGive(eulerAnglesMutex); // release the mutex
    }
    // read user input from nRF24L01 (use mutex):

    // PID start:

    // Motor Mixer Algorithm:
    
    // Motor Output:

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
    64,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    loraTXtask,
    "LoRa TX Task",
    64,
    NULL,
    1,
    NULL
  );
}

