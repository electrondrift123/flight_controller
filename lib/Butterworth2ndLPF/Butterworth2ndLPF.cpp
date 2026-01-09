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

    filter->x1_x = 0.0f;
    filter->x2_x = 0.0f;
    filter->x1_y = 0.0f;
    filter->x2_y = 0.0f;
    filter->x1_z = 0.0f;
    filter->x2_z = 0.0f;

    filter->y1_x = 0.0f;
    filter->y2_x = 0.0f;
    filter->y1_y = 0.0f;
    filter->y2_y = 0.0f;
    filter->y1_z = 0.0f;
    filter->y2_z = 0.0f;
    filter->output_x = 0.0f;
    filter->output_y = 0.0f;
    filter->output_z = 0.0f;
}

void Butterworth2ndLPF_Reset(Butterworth2ndLPF_t* filter){
    filter->x1_x = 0.0f;
    filter->x2_x = 0.0f;
    filter->x1_y = 0.0f;
    filter->x2_y = 0.0f;
    filter->x1_z = 0.0f;
    filter->x2_z = 0.0f;

    filter->y1_x = 0.0f;
    filter->y2_x = 0.0f;
    filter->y1_y = 0.0f;
    filter->y2_y = 0.0f;
    filter->y1_z = 0.0f;
    filter->y2_z = 0.0f;
}

void Butterworth2ndLPF_Update(Butterworth2ndLPF_t* filter, float x, float y, float z) {
    // Apply the filter equation
    filter->output_x = filter->b0 * x + filter->b1 * filter->x1_x + filter->b2 * filter->x2_x
                      - filter->a1 * filter->y1_x - filter->a2 * filter->y2_x;
    filter->output_y = filter->b0 * y + filter->b1 * filter->x1_y + filter->b2 * filter->x2_y
                      - filter->a1 * filter->y1_y - filter->a2 * filter->y2_y;
    filter->output_z = filter->b0 * z + filter->b1 * filter->x1_z + filter->b2 * filter->x2_z
                      - filter->a1 * filter->y1_z - filter->a2 * filter->y2_z;

    // Update the previous values
    filter->x1_x = x;
    filter->x2_x = filter->x1_x;
    filter->x1_y = y;
    filter->x2_y = filter->x1_y;
    filter->x1_z = z;
    filter->x2_z = filter->x1_z;

    filter->y2_x = filter->y1_x;
    filter->y1_x = filter->output_x;
    filter->y2_y = filter->y1_y;
    filter->y1_y = filter->output_y;
    filter->y2_z = filter->y1_z;
    filter->y1_z = filter->output_z;
}