#ifndef PID
#define PID

#include <stdint.h>

typedef struct {
    // Gain constants
    float k_p;
    float k_i;
    float k_d;

    // Derivative "D" low-pass filter time constant
    // Sample time (s)
    float d_tau;
    float sample_time;

    // Limits
    float out_min;
    float out_max;
    float clamp_max;
    float clamp_min;

    // General controller variables
    float proportional;
    float integrator;
    float differentiator;
    float previous_error;
    float previous_measurement;

    // Output
    float output;

} pid_controller;


void pid_controller_init(pid_controller *pid);
float pid_controller_update(pid_controller *pid, float setpoint, float measurement);

#endif