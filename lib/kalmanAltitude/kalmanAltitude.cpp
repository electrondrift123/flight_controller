#include "kalmanAltitude.h"

void init_kalmanAltitude(KalmanState_t *state, 
                         float initial_altitude,
                         float Q_pos, 
                         float Q_vel, 
                         float R_baro) {
    state->x[0] = initial_altitude;
    state->x[1] = 0.0f;                    // start with zero vertical velocity

    // Initial uncertainty (tune if needed)
    state->P[0][0] = 1.0f;   // high uncertainty in position initially
    state->P[0][1] = 0.0f;
    state->P[1][0] = 0.0f;
    state->P[1][1] = 1.0f;   // high uncertainty in velocity

    state->Q_pos = Q_pos;
    state->Q_vel = Q_vel;
    state->R_baro = R_baro;
}

float kalmanAltitudeUpdate(KalmanState_t *state, float baroAlt, float accD, float dt) {
    if (dt <= 0.0f) return state->x[0];

    float acc_z = (accD - 1.0f) * 9.81f;

    // === Predict (Time Update) ===
    state->x[0] += state->x[1] * dt + 0.5f * acc_z * dt * dt;
    state->x[1] += acc_z * dt;

    // Covariance predict (simplified but commonly used for this application)
    float dt2 = dt * dt;
    state->P[0][0] += dt * (dt * state->P[1][1] + 2.0f * state->P[0][1]) + state->Q_pos;
    state->P[0][1] += dt * state->P[1][1];
    state->P[1][1] += state->Q_vel;
    // Keep symmetry
    state->P[1][0] = state->P[0][1];

    // === Update (Measurement Update) ===
    float y = baroAlt - state->x[0];           // innovation
    float S = state->P[0][0] + state->R_baro;
    float K0 = state->P[0][0] / S;
    float K1 = state->P[0][1] / S;

    state->x[0] += K0 * y;
    state->x[1] += K1 * y;

    // Update covariance
    float P00 = state->P[0][0];
    float P01 = state->P[0][1];
    state->P[0][0] = (1.0f - K0) * P00;
    state->P[0][1] = (1.0f - K0) * P01;
    state->P[1][0] = state->P[0][1];
    state->P[1][1] -= K1 * P01;

    return state->x[0];
}