#ifndef BUTTERWORTH2NDLPF_H
#define BUTTERWORTH2NDLPF_H

#include <Arduino.h>

typedef struct {
    float b0, b1, b2; // Coefficients for the filter
    float a1, a2;     // Coefficients for the feedback
    float x1_x, x2_x, x1_y, x2_y, x1_z, x2_z;     // Previous input values
    float y1_x, y2_x, y1_y, y2_y, y1_z, y2_z;     // Previous output values

    float output_x, output_y, output_z;
} Butterworth2ndLPF_t;

void Butterworth2ndLPF_Init(Butterworth2ndLPF_t* filter, float cutoffFrequency, float sampleRate);

void Butterworth2ndLPF_Reset(Butterworth2ndLPF_t* filter);

void Butterworth2ndLPF_Update(Butterworth2ndLPF_t* filter, float x, float y, float z);

#endif // BUTTERWORTH2NDLPF_H