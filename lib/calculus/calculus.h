#ifndef CALCULUS_H
#define CALCULUS_H

#include <Arduino.h>

typedef struct {
    float prev_state;
    float output;
} Derivative_t;

typedef struct {
    float integral;
    float output;
    float output_limit;
} Integral_t;

void init_derivative(Derivative_t *der);
void init_integral(Integral_t *integ);

float compute_derivative(Derivative_t *der, float current_state, float dt);
float compute_integral(Integral_t *integ, float integral, float dt);

#endif 