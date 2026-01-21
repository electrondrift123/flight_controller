#ifndef EMA_H
#define EMA_H

#include <Arduino.h>

typedef struct{
    float order; // [1,2]
    float fc; // cutoff frequency
    float fs; // sampling time
    float tau; // tie constant
    float alpha; // [0,1]
    float x_prev;
    float x_prev2; // for 2nd order
    float output;
}EMA_t;

void emaInit(EMA_t* filter, float order, float fc, float fs);
void emaUpdate(EMA_t* filter, float x);

#endif 