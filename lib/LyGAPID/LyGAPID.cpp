#include "LyGAPID.h"

void initLyGAPID(LyGAPIDControllerData_t* lygapid, float Kp, float Ki, float Kd, float b_sign,
             float gamma_base, float sigma, float output_limit, float mode){
    lygapid->Kp = Kp;
    lygapid->Ki = Ki;
    lygapid->Kd = Kd;

    lygapid->b_sign = b_sign;

    lygapid->gamma_base = gamma_base;
    lygapid->sigma = sigma;

    lygapid->output_limit = output_limit;

    lygapid->mode = mode;

    lygapid->sp = 0.0f;
    lygapid->integral = 0.0f;
    lygapid->prev_error = 0.0f;
    lygapid->prev_derivative = 0.0f;

    lygapid->i_limit = lygapid->output_limit * 0.40f; // 40% of output limit
}

float computeLyGAPID_out(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;

    float u = lygapid->Kp * error;

    // anti-windup
    if (u > lygapid->output_limit){
        u = lygapid->output_limit;
    }
    else if (u < -lygapid->output_limit){
        u = -lygapid->output_limit;
    }

    lygapid->prev_error = error;

    if (lygapid->mode <= 0.0f){
        // Adaptation
        float gamma_p = lygapid->gamma_base / (1.0f + lygapid->Kp);

        float dKp = gamma_p * lygapid->b_sign * error * error - lygapid->sigma * gamma_p * lygapid->Kp;

        lygapid->Kp += dKp * dt;

        if (lygapid->Kp > KP_OUT_MAX) lygapid->Kp = KP_OUT_MAX; // clamp
        else if (lygapid->Kp < 1.0f) lygapid->Kp = 1.0f;
    }

    return u;
}

float computeLyGAPID_in(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;

    float derivative = (error - lygapid->prev_error) / dt;
    float alpha = 0.2; // adjustable (LPF constant)
    derivative = alpha * derivative + (1 - alpha) * lygapid->prev_derivative;

    float u_ = lygapid->Kp * error + lygapid->Ki * lygapid->integral + lygapid->Kd * derivative;

    bool saturated = (u_ >= lygapid->output_limit || u_ <= -lygapid->output_limit);
    bool reducing_error = (error * u_ < 0);

    // integral update (Anti-windup)
    if (!saturated || reducing_error){
        lygapid->integral += error * dt;
    }

    // integral clamping
    if (lygapid->integral > lygapid->i_limit){
        lygapid->integral = lygapid->i_limit;
    }
    else if (lygapid->integral < -lygapid->i_limit){
        lygapid->integral = -lygapid->i_limit;
    }

    // final output
    float u = lygapid->Kp * error + lygapid->Ki * lygapid->integral + lygapid->Kd * derivative;

    // output clamping
    if (u > lygapid->output_limit){
        u = lygapid->output_limit;
    }
    else if (u < -lygapid->output_limit){
        u = -lygapid->output_limit;
    }

    lygapid->prev_error = error;
    lygapid->prev_derivative = derivative;

    if (lygapid->mode <= 0.0f){
        // Adaptation
        float gamma_p = lygapid->gamma_base / (1.0f + lygapid->Kp);
        float gamma_i = lygapid->gamma_base / (1.0f + lygapid->Ki);
        float gamma_d = lygapid->gamma_base / (1.0f + lygapid->Kd);

        float dKp = gamma_p * lygapid->b_sign * error * error - lygapid->sigma * gamma_p * lygapid->Kp;
        float dKi = gamma_i * lygapid->b_sign * lygapid->integral * error - lygapid->sigma * gamma_i * lygapid->Ki;
        float dKd = gamma_d * lygapid->b_sign * derivative * error - lygapid->sigma * gamma_d * lygapid->Kd;

        lygapid->Kp += dKp * dt;
        lygapid->Ki += dKi * dt;
        lygapid->Kd += dKd * dt;

        if (lygapid->Kp > KP_MAX) lygapid->Kp = KP_MAX; // clamp
        else if (lygapid->Kp < 1.0f) lygapid->Kp = 1.0f;

        if (lygapid->Ki > KI_MAX) lygapid->Ki = KI_MAX; // clamp
        else if (lygapid->Ki < 0.0f) lygapid->Ki = 0.0f;

        if (lygapid->Kd > KD_MAX) lygapid->Kd = KD_MAX; // clamp
        else if (lygapid->Kd < 0.0f) lygapid->Kd = 0.0f;
    }

    return u;
}

void resetLyGAPID(LyGAPIDControllerData_t* lygapid){
    lygapid->integral = 0.0f;
    lygapid->prev_error = 0.0f;
    lygapid->prev_derivative = 0.0f;
}