#include <Motors.h>
#include <Comms.h>
#include <Controls.h>

namespace Motors
{
    // DSHOT Control Signal Configs
    constexpr int DRIVE_MOTOR_POLES = 14;  // number of poles on the drive motors
    constexpr int WEAPON_MOTOR_POLES = 14; // number of poles on the weapon motor
    constexpr DShot::Type DSHOT_TYPE = DShot::Type::Bidir;
    constexpr DShot::Speed DSHOT_SPEED = DShot::Speed::DS600;

    // ESC signal pin config
    constexpr int FR_ESC_PIN = 26; // signal pin for front right motor
    constexpr int FL_ESC_PIN = 27; // front left
    constexpr int BL_ESC_PIN = 28; // back left
    constexpr int BR_ESC_PIN = 29; // back right
    constexpr int WPN_ESC_PIN = 2; // weapon

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

    motor::motor(const char *_name, unsigned int _dshot_gpio, unsigned int _poles = 14, PIO _pio = pio0) : name(_name),
                                                                                                           dshot_gpio(_dshot_gpio),
                                                                                                           poles(_poles),
                                                                                                           pio(_pio),
                                                                                                           dshot(dshot_gpio, pio, DSHOT_TYPE, DSHOT_SPEED, poles) {}

    void motor::dshot_begin()
    {
        dshot.init();
        
        timestamp = millis();
        while (millis() - timestamp <= 1000)
        {
            dshot.setStop();
            delay(1); // wait 1ms to receive telemetry info
            dshot.getRawTelemetry(raw_telem);
        }

        timestamp = millis();
        while (millis() - timestamp <= 200)
        {
            dshot.setCommand(13); // enable extended telemetry
            delay(1);
            dshot.getRawTelemetry(raw_telem);
        }
    }

    motor drive1("drive1", FR_ESC_PIN, DRIVE_MOTOR_POLES, pio0);
    motor drive2("drive2", FL_ESC_PIN, DRIVE_MOTOR_POLES, pio0);
    motor drive3("drive3", BL_ESC_PIN, DRIVE_MOTOR_POLES, pio0);
    motor drive4("drive4", BR_ESC_PIN, DRIVE_MOTOR_POLES, pio0);

    motor weapon("weapon", WPN_ESC_PIN, WEAPON_MOTOR_POLES, pio1);

    std::vector<motor *> motors{&drive1, &drive2, &drive3, &drive4, &weapon};

    void init()
    {
        for (auto &m : motors)
        {
            m->dshot_begin();
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

        for (int i = 0; i < num_motors; i++)
        {
            motors[i].dshot.getRawTelemetry(motors[i].raw_telem);                  // grab the raw telemetry
            motors[i].dshot.decodeTelemetry(motors[i].raw_telem, motors[i].telem); // decode the raw telemetry
        }
    }
}