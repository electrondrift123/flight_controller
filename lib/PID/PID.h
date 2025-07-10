#ifndef PID_H
#define PID_H

#include <Arduino.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float output_max_limit;
    float output_min_limit;
    float integral_limit;

    float setpoint;
} PIDControllerData_t;

void initPID(PIDControllerData_t* pid, float kp, float ki, float kd,
             float output_min_limit, float output_max_limit,
             float integral_limit);

float computePID(PIDControllerData_t* pid, float setpoint, float actual, float dt);

void resetPID(PIDControllerData_t* pid);

#endif // PID_H
