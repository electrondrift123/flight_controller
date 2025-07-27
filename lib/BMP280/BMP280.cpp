#include <Wire.h>
#include <math.h>
#include "BMP280.h"

float baseAltitude = 0;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

int32_t t_fine;

// smoothens the altitude reading: X = (1 - alpha) * X + alpha * X_prev, (LPF: EMA)
float alpha = 0.90f;
float smoothedAltitude = 0.0f;

void updateAltitude(float rawAltitude) {
  smoothedAltitude = (1 - alpha) * rawAltitude + alpha * smoothedAltitude;
}

bool i2c_available(uint8_t reg, uint8_t len) {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMP280_ADDRESS, len) != len) return false;
  return true;
}

void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t read8(uint8_t reg) {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDRESS, 1);
  return Wire.read();
}

uint16_t read16_LE(uint8_t reg) {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDRESS, 2);
  uint16_t lo = Wire.read();
  uint16_t hi = Wire.read();
  return (hi << 8) | lo;
}

int16_t readS16_LE(uint8_t reg) {
  return (int16_t)read16_LE(reg);
}

void readCalibrationData() {
  dig_T1 = read16_LE(0x88);
  dig_T2 = readS16_LE(0x8A);
  dig_T3 = readS16_LE(0x8C);
  dig_P1 = read16_LE(0x8E);
  dig_P2 = readS16_LE(0x90);
  dig_P3 = readS16_LE(0x92);
  dig_P4 = readS16_LE(0x94);
  dig_P5 = readS16_LE(0x96);
  dig_P6 = readS16_LE(0x98);
  dig_P7 = readS16_LE(0x9A);
  dig_P8 = readS16_LE(0x9C);
  dig_P9 = readS16_LE(0x9E);
}

int32_t readRawTemp() {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xFA);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDRESS, 3);
  int32_t adc = Wire.read() << 12 | Wire.read() << 4 | Wire.read() >> 4;
  return adc;
}

int32_t readRawPressure() {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDRESS, 3);
  int32_t adc = Wire.read() << 12 | Wire.read() << 4 | Wire.read() >> 4;
  return adc;
}

float compensateTemperature(int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * dig_T2) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                  dig_T3) >> 14;
  t_fine = var1 + var2;
  return ((t_fine * 5 + 128) >> 8) / 100.0;
}

float compensatePressure(int32_t adc_P) {
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
         ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) return 0;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 25600.0;  // hPa
}

float calculateAltitude(float pressure, float seaLevel_hPa = 1013.25) {
  return 44330.0 * (1.0 - pow(pressure / seaLevel_hPa, 0.1903));
}

bool BMP280_init(void) {
  // Check if calibration registers are readable
  if (!i2c_available(0x88, 24)) return false;  // calibration block
  if (!i2c_available(0xF7, 6)) return false;   // pressure/temp block

  write8(0xF4, 0x27); // ctrl_meas
  write8(0xF5, 0xA0); // config

  readCalibrationData();

  // Initial reading to get base altitude
  int32_t rawTemp = readRawTemp();
  if (rawTemp == 0) return false; // unlikely, but sanity check
  float temp = compensateTemperature(rawTemp);
  float pressure = compensatePressure(readRawPressure());

  if (isnan(temp) || isnan(pressure) || pressure < 300 || pressure > 1100) return false;

  baseAltitude = calculateAltitude(pressure);
  return true;
}


bool BMP280_read(bmp280Data_t *data) {
  if (!i2c_available(0xF7, 6)) return false;

  int32_t rawTemp = readRawTemp();
  if (rawTemp == 0) return false;

  float temp = compensateTemperature(rawTemp);
  float pressure_ = compensatePressure(readRawPressure());
  if (isnan(temp) || isnan(pressure_)) return false;

  float altitude = calculateAltitude(pressure_);
  float relative = altitude - baseAltitude;

  updateAltitude(relative); // Update smoothed altitude

  // data->altitude = relative;
  data->altitude = smoothedAltitude; // Use smoothed altitude
  data->pressure = pressure_;
  data->temperature = temp;

  return true;
}







