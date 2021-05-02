#include "pid.h"

/*
Here's my wonky explanation for some of this stuff - Ahmed

GENERAL FLOW DIAGRAM FOR PID CONTROL

---> (+) sum ---> PID ------> SYSTEM --------> OUTPUT
      (-) |                              |
          |                              |
          |                              |
          |                              |
          '----------- SENSOR <----------'


"ZOOMED IN" TO THE PID BLOCK:

                     ,---- Kp ----------------------------------,
                    |                                            |
... -- (+) sum -----|----- Ki ----- 1/s ----------------------- sum -----> SYSTEM --- ...
        (-) |       |                                            |
            |        '---- Kd -----  s  ----- 1/(s*tau + 1) ----'
           ...

Some notes:
[1] What goes into the SYSTEM block is P + I + D
[2] The derivative branch has a low-pass filter because of high frequency noise
[3] The integrator needs to be "clamped" on both sides to prevent "integrator windup", a solution cleverly
    dubbed "integrator anti-windup" (not shown)
[4] The differentiator can sometimes have a "kick" when there's setpoint changes, so we'll actually place it 
    after the sensor and before the summation block. This applies the derivative directly to the measurement
    rather than to a new input which is abnormally high due to an impulse change (doesn't really affect 
    proportional or integrating branches).
*/



void pid_controller_init(pid_controller *pid) {
    pid->proportional = 0;
    pid->integrator = 0;
    pid->differentiator = 0;

    pid->previous_error = 0;
    pid->previous_measurement = 0;
    pid->output = 0;
}

float pid_controller_update(pid_controller *pid, float setpoint, float measurement) {
    /* Error
    Compute the  difference between our actual and desired values, i.e. the error between what we have and what we want.
    */
    float error = setpoint - measurement;

    /* P => Proportional Gain
    Compute the "proportional" part of the controller using the equation
        p[n] = Kp * e[n]
    */
    float P = pid->k_p * error;
    pid->proportional = P;

    /* I => Integrator Gain
    Compute the "integrator" part of the controller using the equation
        i[n] = i[n-1] + [Ki * Ts * (e[n] + e[n-1])]/2
    Also, clamp the integrator using the integrator clamping parameters.
    */
    pid->integrator = pid->integrator + (pid->k_i * pid->sample_time * (error + pid->previous_error))/2.0f;

    if (pid->integrator > pid->clamp_max)
        pid->integrator = pid->clamp_max;
    else if (pid->integrator < pid->clamp_min)
        pid->integrator = pid->clamp_min;

    float I = pid->integrator;

    /* D => Derivative Gain
    Compute the "derivative" part of the controller using the equation
        d[n] = [2 * Kd * (m[n] - m[n-1])]/(2*tau + Ts) + (2*tau - Ts * d[n-1])/(2*tau + Ts)
    */
    pid->differentiator = -(2.0f * pid->k_d * (measurement - pid->previous_measurement) 
        + (2.0f * pid->d_tau - pid->sample_time) * pid->differentiator)
        / (2.0f * pid->d_tau + pid->sample_time);

    float D = pid->differentiator;

    /* Output Computation
    Compute the output using the equation
        u[n] = p[n] + i[n] + d[n]
    Also, clamp the output using the output_max and output_min parameters, store the current error and measurement values 
    for future computations, and output the final controller computation..
    */
    pid->output = P + I + D;

    if (pid->output > pid->out_max)
        pid->output = pid->out_max;
    else if (pid->output < pid->out_min)
        pid->output = pid->out_min;

    pid->previous_error = error;
    pid->previous_measurement = measurement;

    return pid->output;
}