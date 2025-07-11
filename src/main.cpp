#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <RadioLib.h>
#include <RF24.h>

// Custom Libraries
#include "pin_config.h"
#include "buzzer.h"
#include "tasks_config.h"
#include "sync.h"
#include "sensors.h"
// #include "shared_data.h"
#include "PID.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz
  delay(250);

  buzzer_init(); // Initialize the buzzer
  sensors_init(); // the sensors initialization 
  used_gpio_init(); // Initialize GPIOs
  mutexes_init(); // Initialize mutexes
 
  // === PID CONTROLLER INITIALIZATION ===
  // angle mode ======  NOT YET TUNED & TESTED!
  initPID(&pidRoll,  1.0f, 0.0f, 0.0f, -500.0f, 500.0f, 100.0f);
  initPID(&pidPitch, 1.0f, 0.0f, 0.0f, -500.0f, 500.0f, 100.0f);
  initPID(&pidYaw,   1.0f, 0.0f, 0.0f, -500.0f, 500.0f, 100.0f);
  initPID(&pidThrottle,   1.0f, 0.0f, 0.0f, -500.0f, 500.0f, 100.0f);

  freeRTOS_tasks_init(); // Initialize FreeRTOS tasks
  vTaskStartScheduler();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5000 ms
}


//// TODO:
// 1. lora task
// 2. main radio task
// 3. more failsafes
