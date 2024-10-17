/*
General-use functions
*/

#pragma once

namespace Utilities
{

    /// @brief Map function that works with floating point values
    /// @param x Input value
    /// @param in_min Input range minimum
    /// @param in_max Input range maximum
    /// @param out_min Output range minimum
    /// @param out_max Output range maximum
    /// @return Input value mapped from the input range to the output range
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

    /// @brief Map a plant input to the bidirectional (3D Mode) DShot scale
    /// @param _u Plant input (-u_scale to u_scale)
    /// @return Corresponding value on the bidirectional DShot scale (see https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs)
    int map_bidir(int _u);

    /// @brief Map a positive plant input to the normal DShot scale
    /// @param _u Plant input (0 to u_scale)
    /// @return Corresponding value on the normal DShot scale (0 to stop, 48 to 2047 to run)
    int map_1d(int _u);

    /// @brief Map a RC microsecond value to the bidirectional (3D Mode) DShot scale
    /// @param _uS Microseconds input (1000 to 2000)
    /// @return Corresponding value on the bidirectional DShot scale (see https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs)
    int map_uS_bidir(int _uS);

    /// @brief Map a RC microsecond value to the normal DShot scale
    /// @param _uS Microseconds input (1000 to 2000)
    /// @return Corresponding value on the normal DShot scale (0 to stop, 48 to 2047 to run)
    int map_uS_1d(int _uS);

}
