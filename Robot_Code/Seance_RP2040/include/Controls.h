/*
Control loop calculations to compute the motor control input based on the desired RPM
*/

#pragma once

namespace Controls
{
    
    constexpr int U_SCALE = 1000; // max value of the control outputs (plant inputs)
    constexpr int U_DEADBAND = 10; // deadband for the plant input, to account for jitter in the radio signal
    constexpr int US_DEADBAND = 10; // deadband for microseconds input
    


    void calc_u(); // calculate plant inputs based on reference signals and incoming sensor data
}