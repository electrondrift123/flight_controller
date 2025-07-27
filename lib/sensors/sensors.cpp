#include <Arduino.h>
#include "sensors.h"
#include "BMP280.h"
#include "MPU6050.h"
#include "QMC5883P.h"
#include "shared_data.h"
#include "buzzer.h"

#include "WDT.h"

void buzzer_init_failed(void){
  int interval = 100; // 500 ms
  Serial.println("Buzzer init failed!");
  for (int i = 0; i < 10; i++) {
    buzz_on(); // Buzzer ON
    delay(interval); // Wait for 500ms
    buzz_off(); // Buzzer OFF
    delay(interval); // Wait for 500ms
  }
}

void sensors_init(void) {
  if (!BMP280_init()) {
    Serial.println("BMP280 init Failed!");
    buzzer_init_failed(); // Call buzzer init failed function
    while (1); // halt or retry
  }
  Serial.println("BMP280 initialized successfully!");

  if (!MPU6050_init()) {
    Serial.println("MPU6050 init Failed!");
    buzzer_init_failed(); // Call buzzer init failed function
    while (1); // halt or retry
  }
  Serial.println("MPU6050 initialized successfully!");

  if (!QMC5883P_init()) {
    Serial.println("QMC5883P init Failed!");
    buzzer_init_failed(); // Call buzzer init failed function
    while (1); // halt or retry
  }
  Serial.println("QMC5883P initialized successfully!");
}
