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
}

float computeLyGAPID_out(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;
    lygapid->integral += error * dt;

    float u = lygapid->Kp * error + lygapid->Ki * lygapid->integral;

    // anti-windup
    if (u > lygapid->output_limit){
        lygapid->integral = lygapid->output_limit - lygapid->Kp * error;
        u = lygapid->output_limit;
    }
    else if (u < -lygapid->output_limit){
        lygapid->integral = -lygapid->output_limit - lygapid->Kp * error;
        u = -lygapid->output_limit;
    }

    lygapid->prev_error = error;

    if (lygapid->mode <= 0.0f){
        // Adaptation
        float gamma_p = lygapid->gamma_base / (1.0f + lygapid->Kp);
        float gamma_i = lygapid->gamma_base / (1.0f + lygapid->Ki);

        float dKp = gamma_p * lygapid->b_sign * error * error - lygapid->sigma * gamma_p * lygapid->Kp;
        float dKi = gamma_i * lygapid->b_sign * lygapid->integral * error - lygapid->sigma * gamma_i * lygapid->Ki;

        lygapid->Kp += dKp * dt;
        lygapid->Ki += dKi * dt;
    }

    return u;
}

float computeLyGAPID_in(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt){
    float error = setpoint - actual;
    lygapid->integral += error * dt;

    float derivative = (error - lygapid->prev_error) / dt;
    float alpha = 0.2; // adjustable (LPF constant)
    derivative = alpha * derivative + (1 - alpha) * lygapid->prev_derivative;

    float u = lygapid->Kp * error + lygapid->Ki * lygapid->integral + lygapid->Kd * derivative;

    // anti-windup
    if (u > lygapid->output_limit){
        lygapid->integral = lygapid->output_limit - lygapid->Kp * error - lygapid->Kd * derivative;
        u = lygapid->output_limit;
    }
    else if (u < -lygapid->output_limit){
        lygapid->integral = -lygapid->output_limit - lygapid->Kp * error - lygapid->Kd * derivative;
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
    }

    return u;
}

void resetLyGAPID(LyGAPIDControllerData_t* lygapid){
    lygapid->integral = 0.0f;
    lygapid->prev_error = 0.0f;
    lygapid->prev_derivative = 0.0f;
}