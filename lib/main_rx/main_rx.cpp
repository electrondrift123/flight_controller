#include "main_rx.h"

#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"

#include "pin_config.h"
#include "sync.h"

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };


// ------------------- ISR -------------------
void nrfInterruptHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(radioTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool main_rx_init(void){
    // Setup radio
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  if (!radio.begin()) {
    Serial.println("nRF24 not detected - but SPI works!");
    Serial.println("Trying reset...");
    
    // Manual reset procedure
    digitalWrite(NRF_CE_PIN, LOW);
    digitalWrite(NRF_CSN_PIN, HIGH);
    delay(100);
    radio.begin();
  }
  SPI.endTransaction();

  radio.flush_rx();
  radio.flush_tx();

  Serial.println("radio initialized!");

  // --- RADIO SETTINGS TO MATCH TX ---
  radio.setPALevel(RF24_PA_MAX);        // Use LOW or MAX (depending on power)
  // radio.setDataRate(RF24_1MBPS);        // MATCH: must match TX
  radio.setDataRate(RF24_250KBPS);        // MATCH: must match TX
  radio.setChannel(108);                // MATCH: must match TX
  radio.setCRCLength(RF24_CRC_16);      // Optional, but best to match TX
  radio.enableDynamicPayloads();        // Enable variable-length payloads
  radio.enableAckPayload();             // Allow ACK payloads
  // radio.setRetries(3, 5);               // Optional, RX ignores but safe for consistency

  // Only trigger interrupt on data ready (RX)
  // radio.setStatusFlags(RF24_RX_DR);

  // Address pipe must match TX's writing pipe
  radio.openReadingPipe(PIPE_INDEX, address[0]);
  radio.startListening();

  // IRQ pin
  pinMode(NRF_IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), nrfInterruptHandler, FALLING);

  if (!radio.isChipConnected()) {
    Serial.println("nRF24 disconnected!");
    return false;
  } else {
    Serial.println("nRF24 connected!");
    return true;
  }
}