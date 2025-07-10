#include "PID.h"

void initPID(PIDControllerData_t* pid, float kp, float ki, float kd,
             float output_min_limit, float output_max_limit,
             float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_min_limit = output_min_limit;
    pid->output_max_limit = output_max_limit;
    pid->integral_limit = integral_limit;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->setpoint = 0.0f; // Initialize setpoint to zero
}

float computePID(PIDControllerData_t* pid, float setpoint, float actual, float dt) {
    float error = setpoint - actual;

    // Integral term with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;

    // PID Output
    float output = (pid->kp * error) +
                   (pid->ki * pid->integral) +
                   (pid->kd * derivative);

    // Output limiting
    if (output > pid->output_max_limit)
        output = pid->output_max_limit;
    else if (output < pid->output_min_limit)
        output = pid->output_min_limit;

    // Save error for next cycle
    pid->prev_error = error;

    return output;
}

void resetPID(PIDControllerData_t* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
