#include <Arduino.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>
#include <RadioLib.h>
#include <RF24.h>

// Custom Libraries
#include "pin_config.h"
#include "tasks_config.h"
#include "sync.h"
#include "sensors.h"
// #include "shared_data.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // Set I2C clock to 400 kHz
  delay(250);

  sensors_init(); // the sensors initialization 
  used_gpio_init(); // Initialize GPIOs
  mutexes_init(); // Initialize mutexes
 
  freeRTOS_tasks_init(); // Initialize FreeRTOS tasks
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5000 ms
}

