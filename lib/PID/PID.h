#ifndef PID_H
#define PID_H

#include <Arduino.h>

typedef struct {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
    float setpoint;  // Desired value
    float integral;  // Integral term
    float last_error;  // Last error value
    unsigned long last_time;  // Last time the PID was updated
} PIDControllerData_t;

void PIDController_Init(PIDControllerData_t *pid, float Kp, float Ki, float Kd, float setpoint);
void PIDCompute(PIDControllerData_t *pid, float current_value, float *output);

#endif