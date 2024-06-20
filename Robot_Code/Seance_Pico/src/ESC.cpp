#include <ESC.h>
#include <Comms.h>
#include <Controls.h>

namespace ESC
{

    constexpr int num_motors = 5; // number of motors on the robot
    // ESC signal pin config
    constexpr int FR_Pin = 26; // signal pin for front right motor
    constexpr int FL_Pin = 27; // front left
    constexpr int BL_Pin = 28; // back left
    constexpr int BR_Pin = 29; // back right
    constexpr int wpn_Pin = 2; // weapon
    // // pico-dshot ESC objects
    // DShot::ESC drv1(drv1Pin, pio0, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    // DShot::ESC drv2(drv2Pin, pio0, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    // DShot::ESC drv3(drv3Pin, pio0, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    // DShot::ESC drv4(drv4Pin, pio1, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);
    // DShot::ESC wpn(wpnPin, pio1, DSHOT_TYPE, DSHOT_SPEED, DRIVE_MOTOR_POLES);

    // // pico-dshot Telemetry objects
    // DShot::Telemetry drv1Telem = {0}; // Telemetry struct for drive motor 1
    // DShot::Telemetry drv2Telem = {0}; // Telemetry struct for drive motor 2
    // DShot::Telemetry drv3Telem = {0}; // Telemetry struct for drive motor 3
    // DShot::Telemetry drv4Telem = {0}; // Telemetry struct for drive motor 4
    // DShot::Telemetry wpnTelem = {0};  // Telemetry struct for weapon motor

    // // variables to store raw telemetry information
    // uint64_t drv1_raw_telem;
    // uint64_t drv2_raw_telem;
    // uint64_t drv3_raw_telem;
    // uint64_t drv4_raw_telem;
    // uint64_t wpn_raw_telem;

    // struct motor(int pin) {
    //     DShot::ESC esc;
    //     DShot::Telemetry telem = {0};
    //     uint64_t raw_telem;
    // };

    // struct drive {
    //     motor FR; // front right
    //     motor FL; // front left
    //     motor BL; // back left
    //     motor BR; // back right
    // };

    esc motors[] = {
        esc("drv1", FR_Pin, pio0, DRIVE_MOTOR_POLES),
        esc("drv2", FL_Pin, pio0, DRIVE_MOTOR_POLES),
        esc("drv3", BL_Pin, pio0, DRIVE_MOTOR_POLES),
        esc("drv4", BR_Pin, pio1, DRIVE_MOTOR_POLES),
        esc("wpn", wpn_Pin, pio1, DRIVE_MOTOR_POLES)};

    void init()
    {
        for (int i = 0; i < num_motors; i++)
        {
            motors[i].dshot.init();
        }

        int timestamp = millis();
        while (millis() - timestamp <= 1000) // repeat for 1 second
        {
            for (int i = 0; i < num_motors; i++)
            {
                motors[i].dshot.setStop();
            }
            delay(1); // wais 1ms to receive telemetry info
            for (int i = 0; i < num_motors; i++)
            {
                motors[i].dshot.getRawTelemetry(motors[i].raw_telem);
            }
        }

        timestamp = millis();
        while (millis() - timestamp <= 200) // repeat for 0.2 seconds
        {
            for (int i = 0; i < num_motors; i++)
            {
                motors[i].dshot.setCommand(13); // enable extended telemetry
            }
            delay(1); // wais 1ms to receive telemetry info
            for (int i = 0; i < num_motors; i++)
            {
                motors[i].dshot.getRawTelemetry(motors[i].raw_telem);
            }
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
        for (int i = 0; i < num_motors; i++)
        {
            if (motors[i].name == "wpn")
            {
                motors[i].dshot.setCommand(map_wpn(Controls::u[i]));
            }
            else // drive motors
            {
                motors[i].dshot.setCommand(map_3d(Controls::u[i]));
            }
        }

        delay(1); // wait for the PIOs to complete writing

        for (int i=0; i<num_motors; i++) {
            motors[i].dshot.getRawTelemetry(motors[i].raw_telem); // grab the raw telemetry
            motors[i].dshot.decodeTelemetry(motors[i].raw_telem, motors[i].telem); // decode the raw telemetry
        }

    }
}