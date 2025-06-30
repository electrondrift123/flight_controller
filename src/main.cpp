#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <LoRaLib.h>

// Custom Libraries
#include "QMC5883P.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Madgwick.h"

#define LED PC13

//// Mutexes for Serial & i2c
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t wireMutex;
SemaphoreHandle_t spiMutex;

// create the instance of the sensors data
mpu6050Data_t mpuData;
magData_t magData;
bmp280Data_t bmpData;

// create the instance of the Madgwick filter
MadgwickData_t madData;

// Global variables & their mutexes:
SemaphoreHandle_t eulerAnglesMutex;
float eulerAngles[3]; // roll, pitch, yaw

void blinkTask(void *pvParameters) {
  TickType_t interval = pdMS_TO_TICKS(1000); // 500 ms interval
  for(;;){
    digitalWrite(LED, HIGH); // Turn LED on
    vTaskDelay(interval); // Wait for 500 ms
    digitalWrite(LED, LOW); // Turn LED off
    vTaskDelay(interval); // Wait for another 500 ms
  }
}

// check sensor function
bool sensorsReady(){
  return BMP280_read(&bmpData) && MPU6050_read(&mpuData) && QMC5883P_read(&magData);
}

void MadgwickTask(void *Parameters) {
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
          QMC5883P_read(&magData);
          MPU6050_read(&mpuData);

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz
  delay(250);

  // the sensors initialization 
  //// TODO: add buzzers
  if (!BMP280_init()) {
    Serial.println("BMP280 init Failed!");
    while (1); // halt or retry
  }
  Serial.println("BMP280 initialized successfully!");

  if (!MPU6050_init()) {
    Serial.println("MPU6050 init Failed!");
    while (1); // halt or retry
  }
  Serial.println("MPU6050 initialized successfully!");

  if (!QMC5883P_init()) {
    Serial.println("QMC5883P init Failed!");
    while (1); // halt or retry
  }
  Serial.println("QMC5883P initialized successfully!");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // turn led off

  // Mutexes
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    Serial.println("Failed to create serial mutex!");
    while (1); // halt or retry
  }
  wireMutex = xSemaphoreCreateMutex();
  if (wireMutex == NULL) {
    Serial.println("Failed to create Wire mutex!");
    while (1); // halt or retry
  }
  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL){
    Serial.println("Failed to create SPI mutex!");
    while (1); // halt or retry
  }

  eulerAnglesMutex = xSemaphoreCreateMutex();
  if (eulerAnglesMutex == NULL) {
    Serial.println("Failed to create Euler Angles mutex!");
    while (1); // halt or retry
  }
 

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

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 ms
}

