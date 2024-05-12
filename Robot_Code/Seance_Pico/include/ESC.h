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
    // ESC signal pin config
    constexpr int drv1Pin = 26;
    constexpr int drv2Pin = 27;
    constexpr int drv3Pin = 28;
    constexpr int drv4Pin = 29;
    constexpr int wpnPin = 2;

    // pico-dshot Telemetry objects
    extern DShot::Telemetry drv1Telem; // Telemetry struct for drive motor 1
    extern DShot::Telemetry drv2Telem; // Telemetry struct for drive motor 2
    extern DShot::Telemetry drv3Telem; // Telemetry struct for drive motor 3
    extern DShot::Telemetry drv4Telem; // Telemetry struct for drive motor 4
    extern DShot::Telemetry wpnTelem;  // Telemetry struct for weapon motor

    // Initialize DSHOT communications with the ESCs
    void init();

    // Maps drive plant input u (-u_scale to u_scale) to the 3D Mode DShot scale
    int map_3d(int u);

    // Map weapon throttle (1000 to 2000 us) to the 1-direction DShot scale
    int map_wpn(int us);

    // Write DSHOT values to the ESCs, and read the telemetry replies
    void write_read();

}
