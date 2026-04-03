#ifndef VELOCITY_CONTROL_Z_H
#define VELOCITY_CONTROL_Z_H

#include <Arduino.h>

typedef struct {
    float kp;
    float h_prev; // for altitude derivative
    float vz_az;
    float output_limit;
    float alpha;
    float velocity_estimate; // for future velocity estimation
} VelocityControlZData_t;

void initVelocityControlZ(VelocityControlZData_t* vc_z, float kp, float output_limit);
float computeVelocityControlZ(VelocityControlZData_t* vc_z, float v_cmd, float az, float h, float dt);


#endif  // VELOCITY_CONTROL_Z_H