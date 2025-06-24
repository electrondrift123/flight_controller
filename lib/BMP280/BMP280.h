#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>

#define BMP280_ADDRESS 0x76

typedef struct{
  float pressure, temperature, altitude;
}bmp280Data_t;

bool BMP280_init(void);
bool BMP280_read(bmp280Data_t *data);

#endif