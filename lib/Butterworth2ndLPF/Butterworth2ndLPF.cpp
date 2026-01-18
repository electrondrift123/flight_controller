#include "Butterworth2ndLPF.h"

void Butterworth2ndLPF_Init(Butterworth2ndLPF_t* filter, float cutoffFrequency, float sampleRate) {
    float K = tanf(PI * cutoffFrequency / sampleRate);
    float A = 4*(1 + sqrtf(2)*K + K*K);
    float B = 8*(K*K - 1);
    float C = 4*(1 - sqrtf(2)*K + K*K);
    float d = A / 4.0f;

    filter->b0 = (K*K) / d;
    filter->b1 = 2 * filter->b0;
    filter->b2 = filter->b0;
    filter->a1 = B / A;
    filter->a2 = C / A;

    filter->x1 = 0.0f;
    filter->x2 = 0.0f;

    filter->y1 = 0.0f;
    filter->y2 = 0.0f;

    filter->output = 0.0f;
}

void Butterworth2ndLPF_Reset(Butterworth2ndLPF_t* filter){
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;

    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

void Butterworth2ndLPF_Update(Butterworth2ndLPF_t* filter, float x) {
    // Apply the filter equation
    filter->output = filter->b0 * x + filter->b1 * filter->x1 + filter->b2 * filter->x2
                      - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    // Update the previous values
    filter->x1 = x;
    filter->x2 = filter->x1;

    filter->y2 = filter->y1;
    filter->y1 = filter->output;
}