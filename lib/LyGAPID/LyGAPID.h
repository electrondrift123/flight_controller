#ifndef LYGAPID
#define LYGAPID

#include <Arduino.h>

#define KP_OUT_MAX 6.0f
#define KP_OUT_MIN 2.0f

#define KP_MAX 45.0f
#define KP_MIN 30.0f

#define KI_MAX 35.0f
#define KI_MIN 8.0f

#define KD_MAX 0.003f
#define KD_MIN 0.0f


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

float computeLyGAPID_yaw(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt);

void resetLyGAPID(LyGAPIDControllerData_t* lygapid);


#endif // LyGAPID