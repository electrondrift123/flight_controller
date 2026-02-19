#ifndef MAHONY_H
#define MAHONY_H

#include <Arduino.h>
#include <math.h>

typedef struct {
    // Quaternion (q0 = scalar/w, q1,q2,q3 = vector)
    float q0, q1, q2, q3;

    // Euler angles in degrees (NED convention)
    float roll;   // φ - rotation around X (forward)
    float pitch;  // θ - rotation around Y (right)
    float yaw;    // ψ - rotation around Z (down)

    // Tuning parameters
    float twoKp;   // 2 * proportional gain
    float twoKi;   // 2 * integral gain
    float integralFBx, integralFBy, integralFBz;  // integral error terms

    // Sample period (seconds)
    float deltat;
} MahonyFilter_t;

// Initialize filter
void Mahony_Init(MahonyFilter_t* f, float sampleRateHz, float Kp, float Ki);

// Full 9DOF update (accel + gyro + mag)
void Mahony_Update9DOF(MahonyFilter_t* f,
                       float gx, float gy, float gz,     // rad/s
                       float ax, float ay, float az,     // g or m/s² – will normalize
                       float mx, float my, float mz);    // uT or normalized

// Optional: 6DOF update (no magnetometer)
void Mahony_Update6DOF(MahonyFilter_t* f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az);

// Compute Euler angles from current quaternion (NED)
void Mahony_GetEuler(MahonyFilter_t* f);

#endif // MAHONY_H