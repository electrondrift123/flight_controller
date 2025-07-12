#include "sync.h"


// Define (allocate) the mutexes here
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t wireMutex;
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t eulerAnglesMutex;

SemaphoreHandle_t nRF24Mutex;
SemaphoreHandle_t loraMutex;

// Task Handles:
TaskHandle_t radioTaskHandle = NULL;

void mutexes_init(void){
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

  nRF24Mutex = xSemaphoreCreateMutex();
  if (nRF24Mutex == NULL) {
    Serial.println("Failed to create nRF24 mutex!");
    while (1); // halt or retry
  }
  loraMutex = xSemaphoreCreateMutex();
  if (loraMutex == NULL) {
    Serial.println("Failed to create LoRa mutex!");
    while (1); // halt or retry
  }

  eulerAnglesMutex = xSemaphoreCreateMutex();
  if (eulerAnglesMutex == NULL) {
    Serial.println("Failed to create Euler Angles mutex!");
    while (1); // halt or retry
  }
}