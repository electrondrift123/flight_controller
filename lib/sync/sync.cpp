#include "sync.h"


// Define (allocate) the mutexes here
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t wireMutex;
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t eulerAnglesMutex;

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

  eulerAnglesMutex = xSemaphoreCreateMutex();
  if (eulerAnglesMutex == NULL) {
    Serial.println("Failed to create Euler Angles mutex!");
    while (1); // halt or retry
  }
}