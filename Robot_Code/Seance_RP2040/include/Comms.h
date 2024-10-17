
#pragma once

#include <Arduino.h>
#include <CRSFforArduino.hpp>

namespace Comms
{
    // RC channel input mapping
    constexpr int xdot_channel = 0;
    constexpr int ydot_channel = 1;
    constexpr int thetadot_channel = 2;
    constexpr int cv_theta_channel = 3;
    constexpr int wpn_channel = 5; // skip channel 4 (AUX1) as it is reserved strictly for arming/disarming

    // Input value scaling
    constexpr float maxXDot = 10.0 * 12;        // maximum xDot (inches per second)
    constexpr float maxYDot = 10.0 * 12;        // maximum yDot (inches per second)
    constexpr float maxThetaDot = 3 * (2 * PI); // maximum thetaDot (radians per second)

    // Global Input Variables
    extern float xdot;       // desired x velocity (inches per second)
    extern float ydot;       // desired y velocity (inches per second)
    extern float thetadot;   // desired angular velocity (radians per second)
    extern float cv_theta;   // yaw angle as measured by the computer vision (cv) system (radians)
    extern int wpn_throttle; // weapon throttle value (microseconds)

    // This function is automatically called every time new RC channel data is received
    void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData);

    // Initializes CRSF communication with the receiver
    void init();

    // Update function that reads new RC channel data and writes new telemetry data
    // Should be called as often as possible!
    void read();
}