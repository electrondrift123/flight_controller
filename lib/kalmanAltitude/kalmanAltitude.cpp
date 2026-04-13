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
    state->S[0] = 0.0f;
    state->S[1] = 0.0f;
    memset(state->P, 0, sizeof(state->P));  // probably not good
}

void kalman_reset(KalmanState_t* state) {
    state->S[0] = 0.0f;
    state->S[1] = 0.0f;
    memset(state->P, 0, sizeof(state->P));
}

void kalman_update(KalmanState_t* state, float alt_raw, float az, float dt) {
    if (dt <= 0.0f) {
        return;  // prevent division-by-zero or crazy behavior
    }

    // === HANDLE INITIAL / RESET STATE (prevents wild velocity swings at startup) ===
    // If P is still zero (from init/reset), give the filter realistic starting uncertainty.
    // Large velocity uncertainty lets it settle very quickly without crazy oscillations.
    if (state->P[0][0] < 1e-6f && state->P[1][1] < 1e-6f) {
        state->P[0][0] = 100.0f * 100.0f;   // ±100 m on altitude (we don't care much about alt)
        state->P[1][1] = 40.0f * 40.0f;     // ±20 m/s on velocity — allows fast settling
        state->P[0][1] = state->P[1][0] = 0.0f;
    }

    // === 1. STATE PREDICTION (position + velocity driven by measured acceleration) ===
    float S_pred[2];
    S_pred[0] = state->S[0] + state->S[1] * dt + 0.5f * dt * dt * az;
    S_pred[1] = state->S[1] + dt * az;

    // === 2. COVARIANCE PREDICTION ===
    // Process noise is now much smaller and tunable — this is the key to responsive + low-noise velocity
    float G[2] = {0.5f * dt * dt, dt};
    float accel_noise_std = 0.2f;           // <<< TUNE THIS (0.3–1.0 range is typical)
                                            // Lower value = trust IMU accel more → faster response, smoother velocity
                                            // Higher value = trust baro more → slower response, may add noise
    float q = accel_noise_std * accel_noise_std;

    float Q[2][2] = {
        {G[0] * G[0] * q, G[0] * G[1] * q},
        {G[1] * G[0] * q, G[1] * G[1] * q}
    };

    float F[2][2] = {{1.0f, dt}, {0.0f, 1.0f}};

    // F * P
    float FP[2][2];
    FP[0][0] = F[0][0] * state->P[0][0] + F[0][1] * state->P[1][0];
    FP[0][1] = F[0][0] * state->P[0][1] + F[0][1] * state->P[1][1];
    FP[1][0] = F[1][0] * state->P[0][0] + F[1][1] * state->P[1][0];
    FP[1][1] = F[1][0] * state->P[0][1] + F[1][1] * state->P[1][1];

    // P_pred = F * P * F^T + Q
    float P_pred[2][2];
    P_pred[0][0] = FP[0][0] * F[0][0] + FP[0][1] * F[0][1] + Q[0][0];
    P_pred[0][1] = FP[0][0] * F[1][0] + FP[0][1] * F[1][1] + Q[0][1];
    P_pred[1][0] = FP[1][0] * F[0][0] + FP[1][1] * F[0][1] + Q[1][0];
    P_pred[1][1] = FP[1][0] * F[1][0] + FP[1][1] * F[1][1] + Q[1][1];

    // === 3. KALMAN GAIN ===
    // We deliberately trust the altitude measurement LESS (higher R) because you said you don't care about altitude.
    // This makes velocity almost purely driven by the IMU acceleration input → more responsive and much less noisy.
    float dev = 1.2f; // 120 cm deviation
    float R = dev * dev;   // <<< TUNE THIS (0.3² … 1.0²). Higher = less alt influence = smoother velocity

    float S = P_pred[0][0] + R;   // innovation covariance

    float K[2];
    K[0] = P_pred[0][0] / S;
    K[1] = P_pred[1][0] / S;

    // === 4. STATE UPDATE ===
    float y = alt_raw - S_pred[0];

    state->S[0] = S_pred[0] + K[0] * y;
    state->S[1] = S_pred[1] + K[1] * y;

    // === 5. COVARIANCE UPDATE (Joseph form — numerically stable, prevents negative variances or asymmetry) ===
    // This is the most important stability fix over your original code.
    float I_minus_KH[2][2] = {
        {1.0f - K[0], 0.0f},
        {-K[1],       1.0f}
    };

    // temp = (I - KH) * P_pred
    float temp[2][2];
    temp[0][0] = I_minus_KH[0][0] * P_pred[0][0] + I_minus_KH[0][1] * P_pred[1][0];
    temp[0][1] = I_minus_KH[0][0] * P_pred[0][1] + I_minus_KH[0][1] * P_pred[1][1];
    temp[1][0] = I_minus_KH[1][0] * P_pred[0][0] + I_minus_KH[1][1] * P_pred[1][0];
    temp[1][1] = I_minus_KH[1][0] * P_pred[0][1] + I_minus_KH[1][1] * P_pred[1][1];

    // P = temp * (I - KH)^T + K R K^T
    state->P[0][0] = temp[0][0] * I_minus_KH[0][0] + temp[0][1] * I_minus_KH[1][0] + K[0] * R * K[0];
    state->P[0][1] = temp[0][0] * I_minus_KH[0][1] + temp[0][1] * I_minus_KH[1][1] + K[0] * R * K[1];
    state->P[1][0] = state->P[0][1];  // enforce symmetry (extra safety)
    state->P[1][1] = temp[1][0] * I_minus_KH[0][1] + temp[1][1] * I_minus_KH[1][1] + K[1] * R * K[1];
}