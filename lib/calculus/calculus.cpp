#include "calculus.h"
#include "EMA.h"
#include "shared_data.h"

void init_derivative(Derivative_t *der){
    der->prev_state = 0.0f;
    der->output = 0.0f;

    emaInit(&d_filter, 1.0f, 1.0f, 100.0f); // PT1, fc = 10 Hz, fs = 100 Hz (dt=0.010s) for derivative smoothing
}
void init_integral(Integral_t *integ){
    integ->integral = 0.0f;
    integ->output = 0.0f;
    integ->output_limit = 100.0f; // default limit, can be updated later
}

float compute_derivative(Derivative_t *der, float current_state, float dt){
    float d = (current_state - der->prev_state) / dt;
    emaUpdate(&d_filter, d); // Smooth the derivative
    d = d_filter.output; // Get the smoothed derivative
    der->prev_state = current_state;
    der->output = d;
    return d;
}
float compute_integral(Integral_t *integ, float integral, float dt){
    integ->integral += integral * dt;
    // Anti-windup: clamp the integral term
    if (integ->integral > integ->output_limit)  integ->integral = integ->output_limit;
    if (integ->integral < -integ->output_limit) integ->integral = -integ->output_limit;
    integ->output = integ->integral;
    return integ->integral;
}
