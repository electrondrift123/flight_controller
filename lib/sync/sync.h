#ifndef SYNC_H
#define SYNC_H

#include <Arduino.h>
#include <Arduino.h>
#include <STM32FreeRTOS.h>

// Declare (not define) the mutexes
extern SemaphoreHandle_t serialMutex;
extern SemaphoreHandle_t wireMutex;
extern SemaphoreHandle_t spiMutex;
extern SemaphoreHandle_t eulerAnglesMutex;

extern SemaphoreHandle_t nRF24Mutex;
extern SemaphoreHandle_t loraMutex;

// Task Handles
extern TaskHandle_t radioTaskHandle;

// from sensor reading task
extern SemaphoreHandle_t madgwickMutex;

// for watchdog timer - failsafe
extern SemaphoreHandle_t watchdogMutex;

// Function to initialize all mutexes
void mutexes_init(void);

#endif // SYNC_H
