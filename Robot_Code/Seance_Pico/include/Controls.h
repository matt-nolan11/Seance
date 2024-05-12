/*
Control loop calculations to compute the motor control input based on the desired RPM
*/

#pragma once

namespace Controls
{
    constexpr int u_scale = 10000; // max value of the control outputs (plant inputs)
    constexpr int u_deadband = 10; // deadband for the plant input, to account for jitter in the radio signal
    // Plant inupts (u)
    extern int drv1_u; // Plant input for drive motor 1, using u_scale
    extern int drv2_u; // Plant input for drive motor 2, using u_scale
    extern int drv3_u; // Plant input for drive motor 3, using u_scale
    extern int drv4_u; // Plant input for drive motor 4, using u_scale
}