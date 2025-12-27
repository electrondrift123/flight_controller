#ifndef LYGAPID
#define LYGAPID

#include <Arduino.h>

#define KP_OUT_MAX 10.0f

#define KP_MAX 10.0f
#define KI_MAX 10.0f
#define KD_MAX 10.0f


typedef struct {
    float Kp, Ki, Kd;

    float integral, prev_error, prev_derivative, i_limit;

    float gamma_base;
    float sigma;

    float b_sign;

    float output_limit;
    float sp;

    float mode; // 0 = adaptive, 1 = static

} LyGAPIDControllerData_t;

void initLyGAPID(LyGAPIDControllerData_t* lygapid, float kp, float ki, float kd, float b_sign,
             float gamma_base, float sigma, float output_limit, float mode);

             
float computeLyGAPID_out(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt);

float computeLyGAPID_in(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt);

void resetLyGAPID(LyGAPIDControllerData_t* lygapid);


#endif // LyGAPID