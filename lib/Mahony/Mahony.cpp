#include "Mahony.h"

// Some useful constants
#define twoKpDef    (2.0f * 15.0f)     // default 2Kp — tune this!
#define twoKiDef    (2.0f * 0.05f)     // default 2Ki — tune this!

// Quick normalization helper
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - halfx * y * y);
    return y;
}

// Initialize
void Mahony_Init(MahonyFilter_t* f, float sampleRateHz, float Kp, float Ki) {
    f->q0 = 1.0f;
    f->q1 = 0.0f;
    f->q2 = 0.0f;
    f->q3 = 0.0f;

    f->integralFBx = 0.0f;
    f->integralFBy = 0.0f;
    f->integralFBz = 0.0f;

    f->twoKp = (Kp > 0.0f) ? 2.0f * Kp : twoKpDef;
    f->twoKi = (Ki > 0.0f) ? 2.0f * Ki : twoKiDef;

    f->deltat = 1.0f / sampleRateHz;
}

// 9DOF Mahony update (accel + gyro + mag)
void Mahony_Update9DOF(MahonyFilter_t* f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz) {

    float recipNorm;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        Mahony_Update6DOF(f, gx, gy, gz, ax, ay, az);
        return;
    }

    // Normalize accelerometer
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalize magnetometer
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Estimated direction of gravity and magnetic field
    halfvx = f->q1 * f->q3 - f->q0 * f->q2;
    halfvy = f->q0 * f->q1 + f->q2 * f->q3;
    halfvz = f->q0 * f->q0 - 0.5f + f->q3 * f->q3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - f->q2 * f->q2 - f->q3 * f->q3) +
                 my * (f->q1 * f->q2 - f->q0 * f->q3) +
                 mz * (f->q1 * f->q3 + f->q0 * f->q2));
    hy = 2.0f * (mx * (f->q1 * f->q2 + f->q0 * f->q3) +
                 my * (0.5f - f->q1 * f->q1 - f->q3 * f->q3) +
                 mz * (f->q2 * f->q3 - f->q0 * f->q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (f->q1 * f->q3 - f->q0 * f->q2) +
                 my * (f->q2 * f->q3 + f->q0 * f->q1) +
                 mz * (0.5f - f->q1 * f->q1 - f->q2 * f->q2));

    halfwx = bx * (0.5f - f->q2 * f->q2 - f->q3 * f->q3) + bz * halfvx;
    halfwy = bx * (f->q1 * f->q2 - f->q0 * f->q3) + bz * halfvy;
    halfwz = bx * (f->q1 * f->q3 + f->q0 * f->q2) + bz * (0.5f - f->q1 * f->q1 - f->q2 * f->q2);

    // Error is cross product between estimated direction and measured direction of fields
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (f->twoKi > 0.0f) {
        f->integralFBx += f->twoKi * halfex * f->deltat;
        f->integralFBy += f->twoKi * halfey * f->deltat;
        f->integralFBz += f->twoKi * halfez * f->deltat;
        gx += f->integralFBx;
        gy += f->integralFBy;
        gz += f->integralFBz;
    } else {
        f->integralFBx = 0.0f;
        f->integralFBy = 0.0f;
        f->integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += f->twoKp * halfex;
    gy += f->twoKp * halfey;
    gz += f->twoKp * halfez;

    // Integrate rate of change of quaternion
    qDot1 = 0.5f * (-f->q1 * gx - f->q2 * gy - f->q3 * gz);
    qDot2 = 0.5f * ( f->q0 * gx + f->q2 * gz - f->q3 * gy);
    qDot3 = 0.5f * ( f->q0 * gy - f->q1 * gz + f->q3 * gx);
    qDot4 = 0.5f * ( f->q0 * gz + f->q1 * gy - f->q2 * gx);

    f->q0 += qDot1 * f->deltat;
    f->q1 += qDot2 * f->deltat;
    f->q2 += qDot3 * f->deltat;
    f->q3 += qDot4 * f->deltat;

    // Normalise quaternion
    recipNorm = invSqrt(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= recipNorm;
    f->q1 *= recipNorm;
    f->q2 *= recipNorm;
    f->q3 *= recipNorm;
}

// 6DOF version (no mag) – simpler
void Mahony_Update6DOF(MahonyFilter_t* f,
                       float gx, float gy, float gz,
                       float ax, float ay, float az) {

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qDot1, qDot2, qDot3, qDot4;

    // Normalize accel
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = f->q1 * f->q3 - f->q0 * f->q2;
    halfvy = f->q0 * f->q1 + f->q2 * f->q3;
    halfvz = f->q0 * f->q0 - 0.5f + f->q3 * f->q3;

    // Error = cross product
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Integral feedback
    if (f->twoKi > 0.0f) {
        f->integralFBx += f->twoKi * halfex * f->deltat;
        f->integralFBy += f->twoKi * halfey * f->deltat;
        f->integralFBz += f->twoKi * halfez * f->deltat;
        gx += f->integralFBx;
        gy += f->integralFBy;
        gz += f->integralFBz;
    } else {
        f->integralFBx = f->integralFBy = f->integralFBz = 0.0f;
    }

    // Proportional feedback
    gx += f->twoKp * halfex;
    gy += f->twoKp * halfey;
    gz += f->twoKp * halfez;

    // Integrate quaternion rate
    qDot1 = 0.5f * (-f->q1 * gx - f->q2 * gy - f->q3 * gz);
    qDot2 = 0.5f * ( f->q0 * gx + f->q2 * gz - f->q3 * gy);
    qDot3 = 0.5f * ( f->q0 * gy - f->q1 * gz + f->q3 * gx);
    qDot4 = 0.5f * ( f->q0 * gz + f->q1 * gy - f->q2 * gx);

    f->q0 += qDot1 * f->deltat;
    f->q1 += qDot2 * f->deltat;
    f->q2 += qDot3 * f->deltat;
    f->q3 += qDot4 * f->deltat;

    // Normalize
    recipNorm = invSqrt(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
    f->q0 *= recipNorm;
    f->q1 *= recipNorm;
    f->q2 *= recipNorm;
    f->q3 *= recipNorm;
}

// Compute roll, pitch, yaw (NED convention)
void Mahony_GetEuler(MahonyFilter_t* f) {
    float q0 = f->q0;
    float q1 = f->q1;
    float q2 = f->q2;
    float q3 = f->q3;

    // Roll (x-axis)
    f->roll = atan2f(2.0f * (q0*q1 + q2*q3),
                              1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.2958f;

    // Pitch (y-axis)
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f)
        f->pitch = copysignf(90.0f, sinp);
    else
        f->pitch = asinf(sinp) * 57.2958f;

    // Yaw (z-axis)
    f->yaw = atan2f(2.0f * (q0*q3 + q1*q2),
                             1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.2958f;
}