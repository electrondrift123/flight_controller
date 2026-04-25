#include "kalmanAltitude.h"

// void init_kalmanAltitude(KalmanState_t *state, 
//                          float initial_altitude,
//                          float Q_pos, 
//                          float Q_vel, 
//                          float R_baro) {
//     state->x[0] = initial_altitude;
//     state->x[1] = 0.0f;                    // start with zero vertical velocity

//     // Initial uncertainty (tune if needed)
//     state->P[0][0] = 1.0f;   // high uncertainty in position initially
//     state->P[0][1] = 0.0f;
//     state->P[1][0] = 0.0f;
//     state->P[1][1] = 1.0f;   // high uncertainty in velocity

//     state->Q_pos = Q_pos;
//     state->Q_vel = Q_vel;
//     state->R_baro = R_baro;
// }

// float kalmanAltitudeUpdate(KalmanState_t *state, float baroAlt, float accD, float dt) {
//     if (dt <= 0.0f) return state->x[0];

//     float acc_z = (accD - 1.0f) * 9.81f; // poitive vz upward

//     // === Predict (Time Update) ===
//     state->x[0] += state->x[1] * dt + 0.5f * acc_z * dt * dt;
//     state->x[1] += acc_z * dt;

//     // Covariance predict (simplified but commonly used for this application)
//     float dt2 = dt * dt;
//     state->P[0][0] += dt * (dt * state->P[1][1] + 2.0f * state->P[0][1]) + state->Q_pos;
//     state->P[0][1] += dt * state->P[1][1];
//     state->P[1][1] += state->Q_vel;
//     // Keep symmetry
//     state->P[1][0] = state->P[0][1];

//     // // === Update (Measurement Update) ===
//     // float y = baroAlt - state->x[0];           // innovation
//     // float S = state->P[0][0] + state->R_baro;
//     // float K0 = state->P[0][0] / S;
//     // float K1 = state->P[0][1] / S;

//     // state->x[0] += K0 * y;
//     // state->x[1] += K1 * y;

//     // // Update covariance
//     // float P00 = state->P[0][0];
//     // float P01 = state->P[0][1];
//     // state->P[0][0] = (1.0f - K0) * P00;
//     // state->P[0][1] = (1.0f - K0) * P01;
//     // state->P[1][0] = state->P[0][1];
//     // state->P[1][1] -= K1 * P01;

//     // === Correct Update (Measurement Update) ===
//     float y = baroAlt - state->x[0];
//     float S = state->P[0][0] + state->R_baro;
//     float K0 = state->P[0][0] / S;
//     float K1 = state->P[0][1] / S;

//     state->x[0] += K0 * y;
//     state->x[1] += K1 * y;

//     // Use temporary variables to prevent using updated values mid-calculation
//     float P00_temp = state->P[0][0];
//     float P01_temp = state->P[0][1];

//     state->P[0][0] -= K0 * P00_temp;
//     state->P[0][1] -= K0 * P01_temp;
//     state->P[1][0] = state->P[0][1];
//     state->P[1][1] -= K1 * P01_temp; // Correct: uses the same P01 used for K1

//     return state->x[0];
// }

// void reset_kalmanAltitude(KalmanState_t *state) {
//     state->x[0] = 0.0f; // Reset altitude to zero
//     state->x[1] = 0.0f; // Reset vertical velocity to zero

//     // Reset covariance to initial values (tune if needed)
//     state->P[0][0] = 1.0f;   // high uncertainty in position
//     state->P[0][1] = 0.0f;
//     state->P[1][0] = 0.0f;
//     state->P[1][1] = 1.0f;   // high uncertainty in velocity
// }


void kalman_init(KalmanState_t* state) {
    state->S[0] = 0.0f;  // position
    state->S[1] = 0.0f;  // velocity
    state->S[2] = -0.30f;  // accel bias
    memset(state->P, 0, sizeof(state->P));
}

void kalman_reset(KalmanState_t* state) {
    state->S[0] = 0.0f;
    state->S[1] = 0.0f;
    state->S[2] = -0.30f;
    memset(state->P, 0, sizeof(state->P));
}

void kalman_update(KalmanState_t* state, float alt_raw, float az, float dt) {
    if (dt <= 0.0f) return;

    // === INITIAL / RESET ===
    if (state->P[0][0] < 1e-6f && state->P[1][1] < 1e-6f && state->P[2][2] < 1e-6f) {
        state->P[0][0] = 1.0f;
        state->P[1][1] = 1.0f;
        state->P[2][2] = 0.1f;
        state->P[0][1] = state->P[1][0] = 0.0f;
        state->P[0][2] = state->P[2][0] = 0.0f;
        state->P[1][2] = state->P[2][1] = 0.0f;
    }

    // === 1. STATE PREDICTION ===
    float az_corrected = az - state->S[2];
    
    float S_pred[3];
    S_pred[0] = state->S[0] + state->S[1] * dt + 0.5f * dt * dt * az_corrected;
    S_pred[1] = state->S[1] + dt * az_corrected;
    S_pred[2] = state->S[2];

    // === 2. SIMPLIFIED COVARIANCE PREDICTION (numerically stable) ===
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    
    // Process noise
    float q_accel = 0.4f;      // Acceleration process noise (uncertainty in az), was 0.1
    float bias_rw_std = 0.02f; // was 0.020
    float q_bias = bias_rw_std * bias_rw_std * dt;
    
    // Predict covariance (analytical solution for constant dt)
    // This avoids matrix multiplication numerical issues
    float P00 = state->P[0][0] + 2.0f*dt*state->P[0][1] + dt2*state->P[1][1] 
                - dt2*state->P[0][2] - dt3*state->P[1][2] + 0.25f*dt4*q_accel;
    float P01 = state->P[0][1] + dt*state->P[1][1] - dt*state->P[0][2] 
                - 0.5f*dt2*state->P[1][2] + 0.5f*dt3*q_accel;
    float P02 = state->P[0][2] + dt*state->P[1][2];
    float P11 = state->P[1][1] - 2.0f*dt*state->P[1][2] + dt2*state->P[2][2] + dt2*q_accel;
    float P12 = state->P[1][2] - dt*state->P[2][2];
    float P22 = state->P[2][2] + q_bias;
    
    float P_pred[3][3] = {
        {P00, P01, P02},
        {P01, P11, P12},
        {P02, P12, P22}
    };
    
    // === 3. KALMAN GAIN ===
    float dev = 0.4f; // expected deviation from baro
    float R = dev * dev;  // 0.5m std dev squared
    float S = P_pred[0][0] + R;
    
    float K[3];
    K[0] = P_pred[0][0] / S;
    K[1] = P_pred[0][1] / S;
    K[2] = P_pred[0][2] / S;
    
    // === 4. STATE UPDATE ===
    float y = alt_raw - S_pred[0];
    
    state->S[0] = S_pred[0] + K[0] * y;
    state->S[1] = S_pred[1] + K[1] * y;
    state->S[2] = S_pred[2] + K[2] * y;
    
    // === 5. COVARIANCE UPDATE (simplified, stable) ===
    float I_KH_00 = 1.0f - K[0];
    float I_KH_01 = 0.0f;
    float I_KH_02 = 0.0f;
    float I_KH_10 = -K[1];
    float I_KH_11 = 1.0f;
    float I_KH_12 = 0.0f;
    float I_KH_20 = -K[2];
    float I_KH_21 = 0.0f;
    float I_KH_22 = 1.0f;
    
    state->P[0][0] = I_KH_00 * P_pred[0][0] + I_KH_01 * P_pred[1][0] + I_KH_02 * P_pred[2][0] + K[0]*R*K[0];
    state->P[0][1] = I_KH_00 * P_pred[0][1] + I_KH_01 * P_pred[1][1] + I_KH_02 * P_pred[2][1] + K[0]*R*K[1];
    state->P[0][2] = I_KH_00 * P_pred[0][2] + I_KH_01 * P_pred[1][2] + I_KH_02 * P_pred[2][2] + K[0]*R*K[2];
    
    state->P[1][0] = state->P[0][1];
    state->P[1][1] = I_KH_10 * P_pred[0][1] + I_KH_11 * P_pred[1][1] + I_KH_12 * P_pred[2][1] + K[1]*R*K[1];
    state->P[1][2] = I_KH_10 * P_pred[0][2] + I_KH_11 * P_pred[1][2] + I_KH_12 * P_pred[2][2] + K[1]*R*K[2];
    
    state->P[2][0] = state->P[0][2];
    state->P[2][1] = state->P[1][2];
    state->P[2][2] = I_KH_20 * P_pred[0][2] + I_KH_21 * P_pred[1][2] + I_KH_22 * P_pred[2][2] + K[2]*R*K[2];
    
    // === 6. SANITY CHECK ===
    if (fabsf(state->S[1]) > 5.0f || fabsf(state->S[2]) > 2.0f) {
        // Reset if diverged
        state->S[0] = alt_raw;
        state->S[1] = 0.0f;
        state->S[2] = 0.0f;
        state->P[0][0] = 1.0f;
        state->P[1][1] = 1.0f;
        state->P[2][2] = 0.1f;
    }
}

