/*
Control loop calculations to compute the motor control input based on the desired RPM
*/

#pragma once

namespace Controls
{
    
    constexpr int u_scale = 10000; // max value of the control outputs (plant inputs)
    constexpr int u_deadband = 10; // deadband for the plant input, to account for jitter in the radio signal
    // Plant inupts (u)
    extern int u[5]; // Plant input for all motors. Motors 1-4 (drive) use u_scale, motor 5 (wpn) uses pulse width in microseconds (1000 - 2000)

    void calc_u(); // calculate plant inputs based on reference signals and incoming sensor data
}