#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>
// #include <RadioLib.h>
#include <RF24.h>

// Custom Libraries
#include "pin_config.h"
#include "buzzer.h"
#include "tasks_config.h"
#include "sync.h"
#include "sensors.h"
#include "shared_data.h"
#include "main_rx.h" // Include main_rx for nRF24 radio handling
#include "PID.h"
#include "WDT.h"

void setup() {
  Serial.begin(115200);
  // while (!Serial);
  delay(250);
  Serial.println("Starting setup...");

  Wire.begin();
  // Wire.setClock(400000); // Set I2C clock to 400 kHz - if acticve, it causes issues with BMP280
  delay(250);
  Serial.println("i2c ready!");

  SPI.begin();
  delay(250);
  Serial.println("SPI ready!");

  sensors_init(); // the sensors initialization 
  used_gpio_init(); // Initialize GPIOs (buzzer, motors, blink)
  mutexes_init(); // Initialize mutexes
  delay(300); // delay before radio init
  if (!main_rx_init()){
    Serial.println("Error initializing main_rx");
    // buzz_on();
    while (1){
      buzz_on();
      delay(500);
      buzz_off();
      delay(500);
    }
  } // Initialize nRF24 radio
 
  // === PID CONTROLLER INITIALIZATION ===
  // angle mode ======  NOT YET TUNED & TESTED!
  // outer loop: P-controller for roll, pitch, yaw angles (unit: rad)
  initPID(&pidRoll,  10.0f, 1.0f, 0.0f, -U_MAX_ROLL, U_MAX_ROLL);
  initPID(&pidPitch, 10.0f, 1.0f, 0.0f, -U_MAX_PITCH, U_MAX_PITCH);
  initPID(&pidYaw,   10.1f, 1.0f, 0.0f, -U_MAX_YAW, U_MAX_YAW);
  // inner loop: PID for rates (unit: rad/sec)
  initPID(&pidRollRate,  1.0f, 1.0f, 0.0f, -U_MAX_ROLL_RATE, U_MAX_ROLL_RATE);
  initPID(&pidPitchRate, 1.0f, 1.0f, 0.0f, -U_MAX_PITCH_RATE, U_MAX_PITCH_RATE);
  initPID(&pidYawRate,   0.0f, 0.0f, 0.0f, -U_MAX_YAW_RATE, U_MAX_YAW_RATE); // zero for testing in a rod

  // interrupts();
  freeRTOS_tasks_init(); // Initialize FreeRTOS tasks
  Serial.println("FreeRTOS tasks initialized!");

  buzz_on();
  delay(200);
  buzz_off();
  vTaskStartScheduler();
}

void loop() {
  // empty
  // Do not put vTaskDelay() or any other blocking code here!!!
}

//// TODO:
// 1. debug the PID logic: map the torque comman into pwm ticks!

// Note: 
// - The output ticks for every motor is 1ms


// NOTE:
// 1. input from human: deg, then convert it into rad


