#include <ESC.h>
#include <Comms.h>
#include <Controls.h>

namespace ESC
{
    // pico-dshot ESC objects
    DShot::ESC drv1(drv1Pin, pio0, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    DShot::ESC drv2(drv2Pin, pio0, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    DShot::ESC drv3(drv3Pin, pio0, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    DShot::ESC drv4(drv4Pin, pio1, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    DShot::ESC wpn(wpnPin, pio1, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);

    // pico-dshot Telemetry objects
    DShot::Telemetry drv1Telem = {0}; // Telemetry struct for drive motor 1
    DShot::Telemetry drv2Telem = {0}; // Telemetry struct for drive motor 2
    DShot::Telemetry drv3Telem = {0}; // Telemetry struct for drive motor 3
    DShot::Telemetry drv4Telem = {0}; // Telemetry struct for drive motor 4
    DShot::Telemetry wpnTelem = {0};  // Telemetry struct for weapon motor

    // variables to store raw telemetry information
    uint64_t drv1_raw_telem;
    uint64_t drv2_raw_telem;
    uint64_t drv3_raw_telem;
    uint64_t drv4_raw_telem;
    uint64_t wpn_raw_telem;

    void init()
    {
        drv1.init();
        drv2.init();
        drv3.init();
        drv4.init();
        wpn.init();

        int timestamp = millis();
        while (millis() - timestamp <= 1000) // repeat for 1 second
        {
            drv1.setStop(); // command motor stop
            drv2.setStop();
            drv3.setStop();
            drv4.setStop();
            wpn.setStop();
            delay(1);
            drv1.getRawTelemetry(drv1_raw_telem);
            drv2.getRawTelemetry(drv2_raw_telem);
            drv3.getRawTelemetry(drv3_raw_telem);
            drv4.getRawTelemetry(drv4_raw_telem);
            wpn.getRawTelemetry(wpn_raw_telem);
        }

        timestamp = millis();
        while (millis() - timestamp <= 200) // repeat for 0.2 seconds
        {
            drv1.setCommand(13); // enable extended telemetry
            drv2.setCommand(13);
            drv3.setCommand(13);
            drv4.setCommand(13);
            wpn.setCommand(13);
            delay(1);
            drv1.getRawTelemetry(drv1_raw_telem);
            drv2.getRawTelemetry(drv2_raw_telem);
            drv3.getRawTelemetry(drv3_raw_telem);
            drv4.getRawTelemetry(drv4_raw_telem);
            wpn.getRawTelemetry(wpn_raw_telem);
        }
    }

    /*
    DShot Bidirectional Motion (3D Mode)
    If you enable 3D mode, the throttle ranges split in two.
    Direction 1) 48 is the slowest, 1047 is the fastest
    Direction 2) 1049 is the slowest, 2047 is the fastest
    1048 does NOT stop the motor! Use command 0 for that.
    */
    int map_3d(int u) // map drive plant input u (-10000 to 10000) to the 3D Mode DShot scale
    {
        // clamp u to be within the specified range
        if (abs(u) > Controls::u_scale) // if u is outside the defined range (should never happen, but better safe than sorry)
        {
            u = (1 - (u < 0) * 2) * Controls::u_scale; // set u to + or - u_scale, depending on the sign of u
        }

        int dshot_output;
        if (abs(u) <= Controls::u_deadband)
        {
            dshot_output = 0; // motor stop command
        }

        else if (u > 0) // positive plant input
        {
            dshot_output = map(u, Controls::u_deadband, Controls::u_scale, 48, 1047);
        }

        else // if u is less than 0
        {
            dshot_output = map(u, -Controls::u_deadband, -Controls::u_scale, 1049, 2047);
        }

        return dshot_output;
    }

    int map_wpn(int us) // map weapon throttle (us) to the 1-direction DShot scale
    {
        return map(us, 1000, 2000, 48, 2047);
    }

    void write_read()
    {
        // send dshot commands
        drv1.setCommand(map_3d(Controls::drv1_u));
        drv2.setCommand(map_3d(Controls::drv2_u));
        drv3.setCommand(map_3d(Controls::drv3_u));
        drv4.setCommand(map_3d(Controls::drv4_u));
        wpn.setCommand(map_wpn(Comms::wpn_throttle));

        delay(1); // wait for the PIOs to complete

        // grab raw telemetry
        drv1.getRawTelemetry(drv1_raw_telem);
        drv2.getRawTelemetry(drv2_raw_telem);
        drv3.getRawTelemetry(drv3_raw_telem);
        drv4.getRawTelemetry(drv4_raw_telem);
        wpn.getRawTelemetry(wpn_raw_telem);

        // decode the raw telemetry
        drv1.decodeTelemetry(drv1_raw_telem, drv1Telem);
        drv2.decodeTelemetry(drv2_raw_telem, drv2Telem);
        drv3.decodeTelemetry(drv3_raw_telem, drv3Telem);
        drv4.decodeTelemetry(drv4_raw_telem, drv4Telem);
        wpn.decodeTelemetry(wpn_raw_telem, wpnTelem);
    }

}