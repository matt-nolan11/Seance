#include <Utilities.h>
#include <Controls.h>
#include <Arduino.h>

namespace Utilities
{
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /*
    DShot Bidirectional Motion (3D Mode)
    If you enable 3D mode, the throttle ranges split in two.
    Direction 1) 48 is the slowest, 1047 is the fastest
    Direction 2) 1049 is the slowest, 2047 is the fastest
    1048 does NOT stop the motor! Use command 0 for that.
    */

    int map_bidir(int u)
    {
        // clamp u to be within the specified range
        u = constrain(u, -Controls::U_SCALE, Controls::U_SCALE);

        int dshot_output;
        if (abs(u) < Controls::U_DEADBAND)
        {
            dshot_output = 0; // motor stop command
        }
        else if (u > 0) // positive plant input
        {
            dshot_output = map(u, Controls::U_DEADBAND, Controls::U_SCALE, 48, 1047);
        }
        else // if u is less than 0
        {
            dshot_output = map(u, -Controls::U_DEADBAND, -Controls::U_SCALE, 1049, 2047);
        }

        return dshot_output;
    }

    int map_1d(unsigned int u)
    {
        // clamp u to be within the specified range
        u = constrain(u, 0, Controls::U_SCALE);

        int dshot_output;
        if (u < Controls::U_DEADBAND)
        {
            dshot_output = 0; // motor stop command
        }
        else
        {
            dshot_output = map(u, Controls::U_DEADBAND, Controls::U_SCALE, 48, 2047);
        }

        return dshot_output;
    }

    int map_uS_bidir(int uS)
    {
        // clamp uS to be within the specified range
        uS = constrain(uS, 1000, 2000);

        int dshot_output;
        if (abs(uS - 1500) < Controls::US_DEADBAND) // microseconds center value is 1500 in bidirectional mode
        {
            dshot_output = 0; // motor stop command
        }
        else if (uS > 1500) // microseconds value > 1500, indicating positive motor motion
        {
            dshot_output = map(uS, 1500 + Controls::US_DEADBAND, 2000, 48, 1047);
        }
        else // microseconds value < 1500, indicating negative motor motion
        {
            dshot_output = map(uS, 1500 - Controls::US_DEADBAND, 1000, 1049, 2047);
        }

        return dshot_output;
    }

    int map_uS_1d(int uS)
    {
        // clamp uS to be within the specified range
        uS = constrain(uS, 1000, 2000);

        int dshot_output;
        if (uS - 1000 < Controls::US_DEADBAND)
        {
            dshot_output = 0; // motor stop command
        }
        else
        {
            dshot_output = map(uS, 1000 + Controls::US_DEADBAND, 2000, 48, 2047);
        }

        return dshot_output;
    }
}