#include <Controls.h>
#include <Comms.h>
#include <ESC.h>

namespace Controls
{
    int drv1_u;
    int drv2_u;
    int drv3_u;
    int drv4_u;

    float test = ESC::drv1Telem.rpm;
    int test2 = Comms::xdot;
}