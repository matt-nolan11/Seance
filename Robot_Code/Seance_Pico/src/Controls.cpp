#include <Controls.h>
#include <Comms.h>
#include <ESC.h>

namespace Controls
{
    int u[5]; // plant inputs (control outputs) for all motors

    // calculate plant inputs based on reference signals and incoming sensor data
    void calc_u()
    {
        u[5] = Comms::wpn_throttle; // forward weapon throttle value directly to the final control input
    }

}