#include <Arduino.h>
#include "sensors.h"
#include "BMP280.h"
#include "MPU6050.h"
#include "QMC5883P.h"
#include "shared_data.h"

void sensors_init(void) {
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
}
