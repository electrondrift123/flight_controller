#ifndef LYGAPID
#define LYGAPID

#include <Arduino.h>

// empirical known good static P-PID gains: 3, 50, 15, 0.005

#define KP_OUT_MAX 4.0f // was 6.0f
#define KP_OUT_MIN 3.0f

#define KP_MAX 60.0f // was 60.0f
#define KP_MIN 50.0f

#define KI_MAX 30.0f
#define KI_MIN 15.0f // maybe try 10 later

#define KD_MAX 0.025f 
#define KD_MIN 0.005f

typedef struct {
    float Kp, Ki, Kd;

    float integral, prev_error, prev_derivative, i_limit;

    float gamma_base;
    float sigma;

    float b_sign;

    float output_limit;
    float sp;

    float mode; // 0 = adaptive, 1 = static
    float landed; // 1 = true

} LyGAPIDControllerData_t;

void initLyGAPID(LyGAPIDControllerData_t* lygapid, float kp, float ki, float kd, float b_sign,
             float gamma_base, float sigma, float output_limit, float mode);

             
float computeLyGAPID_out(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt);

float computeLyGAPID_in(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt);

float computeLyGAPID_yaw(LyGAPIDControllerData_t* lygapid, float setpoint, float actual, float dt);

void resetLyGAPID(LyGAPIDControllerData_t* lygapid);


#endif // LyGAPID