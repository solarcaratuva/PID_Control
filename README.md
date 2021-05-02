# PID_Control
A custom PID control library for the UVA Solar Car Team's motors (Rivanna 2).

- `pid.c` and `pid.h` include the basic descriptions of a PID controller and the implementations for an init and update function.
- `main.c` includes a first-order dynamical system to use in simulation testing - there's way more interesting simulations we can look at, but this is one with a simulated runtime of 5 seconds with sampling times every 10 milliseconds, meaning 500 iterations. Feel free to adjust this locally.

Note there's three added discrepancies between a theoretical PID controller and a "real-world" one:
1. the integrator can have an issue where it saturates its output, called "integrator wind-up". This can be solved by limiting the inputs and outputs of the integrator, or "clamping" them, a solution cleverly called "integrator anti-wind-up".  
2. the differentiator has an issue of adding a large amount of high frequency noise, so there's an addded low-pass filter in  the derivative "D" branch to attenuate the HF noise.  
3. the differentiator is prone to massive "kick" whenever there's an impulse change in the input (e.g. changing goal cruise speed from 30mph to 40mph) as the rate of change is now large. To solve this, we can place the differentiator right after the sensor's  measurement; this gets rid of the kick without affecting any other part of the system.  

**Issues**
- running the default simulation will show that the controller never fully converges to the setpoint (which is 1.00), maybe make this faster or more intense?
- not sure how to find the Kp, Ki, and Kd values experimentally, will likely take testing (integration problem)
