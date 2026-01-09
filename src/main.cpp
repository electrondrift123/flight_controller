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
#include "LyGAPID.h"
#include "WDT.h"
#include "Butterworth2ndLPF.h"

void enableFPU() {
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  // Enable CP10 and CP11 (full access to FPU)
  __DSB();
  __ISB();
}

void setup() {
  enableFPU(); // Enable FPU before any floating-point operations
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
  initLyGAPID(&pidRoll,  P, 0.0f, 0.0f, B_SIGN, GAMMA_B, SIGMA, U_MAX_ROLL, CONTROLLER_MODE);
  initLyGAPID(&pidPitch, P, 0.0f, 0.0f, B_SIGN, GAMMA_B, SIGMA, U_MAX_PITCH, CONTROLLER_MODE);

  // inner loop: PID for rates (unit: rad/sec)
  initLyGAPID(&pidRollRate,  KP, KI, KD, B_SIGN, GAMMA_B, SIGMA, U_MAX_ROLL_RATE, CONTROLLER_MODE);
  initLyGAPID(&pidPitchRate, KP, KI, KD, B_SIGN, GAMMA_B, SIGMA, U_MAX_PITCH_RATE, CONTROLLER_MODE);
  initLyGAPID(&pidYawRate,   KP, KI, KD, B_SIGN, GAMMA_B, SIGMA, U_MAX_YAW_RATE, CONTROLLER_MODE); 

  // Butterworth 2nd order LPF initialization
  Butterworth2ndLPF_Init(&accelLPF, 50.0f, 1000.0f); // Cutoff frequency: 50 Hz, Sample rate: 1000 Hz
  Butterworth2ndLPF_Init(&gyroLPF, 30.0f, 1000.0f); // Cutoff frequency: 30 Hz, Sample rate: 1000 Hz

  // interrupts()
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


