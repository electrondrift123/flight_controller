#ifndef KALMAN_ALTITUDE_H
#define KALMAN_ALTITUDE_H

#include <Arduino.h>

typedef struct {
    float x[2];      // [0]: altitude (m), [1]: vertical velocity (m/s)
    float P[2][2];   // covariance matrix

    float Q_pos;     // process noise position
    float Q_vel;     // process noise velocity
    float R_baro;    // baro measurement noise (m²)
} KalmanState_t;

void init_kalmanAltitude(KalmanState_t *state, 
                         float initial_altitude,
                         float Q_pos, 
                         float Q_vel, 
                         float R_baro);

float kalmanAltitudeUpdate(KalmanState_t *state, float baroAlt, float accD, float dt);

#endif