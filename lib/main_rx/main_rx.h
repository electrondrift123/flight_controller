#ifndef MAIN_RX_H
#define MAIN_RX_H

#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"

extern RF24 radio;
extern uint8_t address[][6];

void nrfInterruptHandler(void);
bool main_rx_init(void);

#endif