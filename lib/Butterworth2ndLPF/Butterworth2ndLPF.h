#ifndef BUTTERWORTH2NDLPF_H
#define BUTTERWORTH2NDLPF_H

#include <Arduino.h>

typedef struct {
    float b0, b1, b2; // Coefficients for the filter
    float a1, a2;     // Coefficients for the feedback
    float x1, x2;     // Previous input values
    float y1, y2;     // Previous output values

    float output;
} Butterworth2ndLPF_t;

void Butterworth2ndLPF_Init(Butterworth2ndLPF_t* filter, float cutoffFrequency, float sampleRate);

void Butterworth2ndLPF_Reset(Butterworth2ndLPF_t* filter);

void Butterworth2ndLPF_Update(Butterworth2ndLPF_t* filter, float x);

#endif // BUTTERWORTH2NDLPF_H