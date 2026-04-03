#include "velocity_control_z.h"
#include "EMA.h"
#include "shared_data.h"

void initVelocityControlZ(VelocityControlZData_t* vc_z, float kp, float output_limit){
    vc_z->kp = kp;
    vc_z->output_limit = output_limit;
    vc_z->velocity_estimate = 0.0f;
    vc_z->h_prev = 0.0f;
    vc_z->vz_az = 0.0f;

    float dt = 0.002f;   // 2 ms control loop (500 Hz)
    float tau = 0.5f;
    float alpha = dt / (tau + dt);  // ~0.004 for dt=0.002
    vc_z->alpha = alpha;

    emaInit(&vz_h_LPF, 1.0f, 30.0f, 500.0f);
}

float computeVelocityControlZ(VelocityControlZData_t* vc_z, float v_cmd, float az, float h, float dt){
    // Convert az to true acceleration
    float true_acceleration = (az - 1.0f) * 9.81f;
    vc_z->vz_az += true_acceleration * dt;
    
    // Apply small decay to prevent drift (optional but helpful)
    vc_z->vz_az *= 0.9995f;

    // Velocity from baro altitude
    float vz_h = (h - vc_z->h_prev) / dt;
    vc_z->h_prev = h;
    emaUpdate(&vz_h_LPF, vz_h);
    vz_h = vz_h_LPF.output;

    // Complementary filter: 95% baro (stable), 5% accel (responsive)
    float baro_weight = 0.95f;
    vc_z->velocity_estimate = baro_weight * vz_h + (1.0f - baro_weight) * vc_z->vz_az;

    // P-controller
    float error = v_cmd - vc_z->velocity_estimate;
    float output = vc_z->kp * error * dt;

    // Clamp output    
    if (output > vc_z->output_limit) output = vc_z->output_limit;
    else if (output < -vc_z->output_limit) output = -vc_z->output_limit;
    
    return output;
}