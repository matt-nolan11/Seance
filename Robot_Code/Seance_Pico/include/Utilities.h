/*
General-use functions
*/

#pragma once

namespace Utilities
{
    // Map function that works with floating point values
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
