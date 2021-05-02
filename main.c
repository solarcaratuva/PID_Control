/*
Finnicky simulation, we'll have to figure how to find the actual values experimentally
for Kp, Ki, Kd - Ahmed
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "pid.h"

// Runtime of simulation
#define SIMULATION_TIME                 5.0f
#define SIMULATION_STEPS                0.01f

// Simulated first-order dynamical system for the purpose of testing.
float Update(pid_controller *pid) {
    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (pid->sample_time * pid->output + output) / (1.0f + alpha * pid->sample_time);

    return output;
}

int main() {
    pid_controller controller;

    // Initialize controller - adjust these gain constants to "shape" the oscillations
    controller.k_p = 2.0f;
    controller.k_i = 1.75f;
    controller.k_d = 0.50f;

    controller.d_tau = 0.02f;

    controller.out_min = -10.0f;
    controller.out_max = 10.0f;
    controller.clamp_min = -5.0f;
    controller.clamp_max = 5.0f;

    controller.sample_time = SIMULATION_STEPS;

    pid_controller_init(&controller);

    // Define the goal/desired value
    float setpoint = 1.0f;
    float measurement;
    float t;

    printf("\r\nTime (s)\tSystem Output\tController Output\t\tP\t   I\t\tD\r\n");

    for (t = 0.0f; t <= SIMULATION_TIME; t += controller.sample_time) {
        measurement = Update(&controller);

        pid_controller_update(&controller, setpoint, measurement);

        printf("%f\t%f\t%f\t\t = \t%f + %f + %f\r\n", t, measurement, controller.output, 
            controller.proportional, controller.integrator, controller.differentiator);
    }

    printf("\r\nINPUT: 0.0\tOUTPUT: %f\tITERATIONS: %f\r\n", measurement, SIMULATION_TIME/SIMULATION_STEPS);

    return 0;
}