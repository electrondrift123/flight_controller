#include "velocity_control_z.h"
#include "EMA.h"
#include "shared_data.h"


void initAltitudeControl(VelocityControlZData_t* vc_z, float kp, float ki, float output_limit){
    vc_z->kp = kp;
    vc_z->ki = ki;
    vc_z->output_limit = output_limit;
    vc_z->altitude_sp = 0.0f;
    vc_z-> integral = 0.0f;
    vc_z->is_flying = 0.0f; // start with grounded state

    emaInit(&alt_sp_LPF, 1.0f, 10.0f, 100.0f); // PT1, fc = 10 Hz, fs = 100 Hz (dt=0.010s) for altitude setpoint smoothing
}

float computeAltitudeControl(VelocityControlZData_t* vc_z, float v_cmd, float altitude, float dt){
    
    if (vc_z->is_flying == 0.0f) { // If currently grounded, initialize setpoint to current altitude
        vc_z->altitude_sp = altitude; // Keep setpoint glued to current height on ground
        vc_z->integral = 0.0f;
        return 0.0f; 
    }

    // scale back the stick input
    v_cmd = v_cmd / 100.0f;
    
    // dt = 0.010s
    // input stick: [-80,80] units = [-0.8, 0.8] m/s cmd
    vc_z->altitude_sp += v_cmd * dt;
    emaUpdate(&alt_sp_LPF, vc_z->altitude_sp);
    vc_z->altitude_sp = alt_sp_LPF.output;

    float error = vc_z->altitude_sp - altitude; // positive error means we need to go up

    vc_z->integral += error * dt;
    float integral_limit = 250.0f;
    if (vc_z->integral > integral_limit) vc_z->integral = integral_limit;
    else if (vc_z->integral < -integral_limit) vc_z->integral = -integral_limit;
    
    float u = vc_z->kp * error + vc_z->ki * vc_z->integral; // P-control

    if (u > vc_z->output_limit) u = vc_z->output_limit;
    else if (u < -vc_z->output_limit) u = -vc_z->output_limit;

    return u;
}
