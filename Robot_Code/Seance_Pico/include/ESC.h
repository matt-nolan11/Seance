/*
Handles the robot's communication with its 5 ESCs (4 drive, 1 weapon), using
bidirectional dshot to get RPM and voltage/current telemetry
*/

#include <dshot/esc.h> // https://github.com/josephduchesne/pico-dshot-bidir

#pragma once

// DSHOT Control Signal Configs
#define DRIVE_MOTOR_POLES 14  // number of poles on the drive motors
#define WEAPON_MOTOR_POLES 14 // number of poles on the weapon motor
#define DSHOT_TYPE DShot::Type::Bidir
#define DSHOT_SPEED DShot::Speed::DS600

namespace ESC
{

    class esc
    {
    public:
        const char* name = "";
        DShot::ESC dshot; // dshot ESC object to read and write directly to the ESC
        DShot::Telemetry telem = {0}; // decoded telemetry data
        uint64_t raw_telem; // raw telemetry data
        esc(const char* name, uint dshot_gpio, PIO pio = pio0, uint poles = 14)
            : dshot(dshot_gpio, pio, DSHOT_TYPE, DSHOT_SPEED, poles), name(name) {};       
    };

    // Initialize DSHOT communications with the ESCs
    void init();

    // Maps drive plant input u (-u_scale to u_scale) to the 3D Mode DShot scale
    int map_3d(int u);

    // Map weapon throttle (1000 to 2000 us) to the 1-direction DShot scale
    int map_wpn(int us);

    // Write DSHOT values to the ESCs, and read the telemetry replies
    void write_read();

}
