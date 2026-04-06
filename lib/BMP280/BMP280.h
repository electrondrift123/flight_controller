#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>

#define BMP280_ADDRESS 0x76

typedef struct {
    float pressure;      // Pressure in hPa
    float temperature;   // Temperature in °C
    float altitude;      // Relative altitude in cm (from startup reference)
} bmp280Data_t;

// Initialize the BMP280 sensor
// Returns true if successful, false if failed
bool BMP280_init(void);

// Read sensor data (pressure, temperature, relative altitude)
// Returns true if successful, false if failed
bool BMP280_read(bmp280Data_t *data);

// Get the current raw altitude in cm (without startup offset)
float BMP280_get_raw_altitude(void);

// Reset the altitude reference to current position
void BMP280_reset_altitude_reference(void);

#endif