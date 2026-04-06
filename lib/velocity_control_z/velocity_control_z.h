#ifndef VELOCITY_CONTROL_Z_H
#define VELOCITY_CONTROL_Z_H

#include <Arduino.h>

typedef struct {
    float kp;
    float ki;
    float output_limit;
    float altitude_sp;
    float integral;
    float is_flying; // 0 = grounded, 1 = flying (for setpoint management)
} VelocityControlZData_t;

void initVelocityControlZ(VelocityControlZData_t* vz_in, float kp, float ki, float output_limit);
float computeVelocityControlZ(VelocityControlZData_t* vz_in, float v_cmd, float velocity_z, float dt);

void initAltitudeControl(VelocityControlZData_t* vc_z, float kp, float ki, float output_limit);
float computeAltitudeControl(VelocityControlZData_t* vc_z, float v_cmd, float altitude, float dt);

#endif  // VELOCITY_CONTROL_Z_H