/*
Handles the robot's communication with its 5 ESCs (4 drive, 1 weapon), using
bidirectional dshot to get RPM and voltage/current telemetry
*/

#include <dshot/esc.h> // https://github.com/josephduchesne/pico-dshot-bidir
#include <QuickPID.h>  // https://github.com/Dlloydev/QuickPID

#pragma once

class Motor
{

public:
    /// @brief Create a motor object and fill in the required configuration constants
    /// @param _name Name of the motor's mechanism (just for debugging)
    /// @param _dshot_gpio Microcontroller pin that the ESC is attached to
    /// @param _gear_ratio How many times the output spins for every motor rotation
    /// @param _poles Number of poles (i.e. magnets) in the rotor (defaults to 14)
    /// @param _pio PIO block that should be used (pio0 or pio1, max 4 motors per block, defaults to pio0)
    Motor(const char *_name,
          unsigned int _dshot_gpio,
          float _gear_ratio,
          unsigned int _poles,
          PIO _pio);

    /// @brief Initializes bidirectional dshot communication with the ESC
    void dshotBegin();

    /// @brief Sets the PWM duty cycle of the motor using the bidirectional plant input scale
    /// @brief Also reads the ESC's telemetry data into the telem object
    /// @param input_u Motor duty cycle on the semi-arbitrary plant input scale (-U_SCALE to U_SCALE)
    void setDuty_U(int input_u);

    /// @brief Sets the PWM duty cycle of the motor using the microsecond scale
    /// @brief Also reads the ESC's telemetry data into the telem object
    /// @param input_uS Motor duty cycle on the RC microseconds scale (1000 to 2000 microseconds)
    void setDuty_uS(int input_uS);

    /// @brief Sends the internally stored dshot_value to the ESC
    void sendDshot();

    /// @brief Reads the bidirectional DShot telemetry from the ESC (should be called 1ms after every set_duty command!)
    void readTelem();

    /// @brief Getter method for the number of motors
    static int getNumMotors();

    /// @brief Getter method for the current vector of Motor instances
    static std::vector<Motor *> getInstances();

private:
    static int num_motors;                 // class-level variable to keep track of the number of motors in the program
    static std::vector<Motor *> instances; // class-level vector that contains references to all the created motor instances, for easy iteration

    // Motor configuration constants, initialized by the class constructor
    const char *name;
    const unsigned int dshot_gpio;
    const float gear_ratio;
    const unsigned int poles;
    const PIO pio;

    // ESC control variables
    DShot::ESC dshot;             // dshot ESC object to read and write directly to the ESC
    DShot::Telemetry telem = {0}; // decoded telemetry data
    uint64_t raw_telem;           // raw telemetry data
    int dshot_value;              // value on the DShot scale that is being sent to the motor

    // PID control variables
    QuickPID rpm_controller; // QuickPID object for controlling the rpm based on bidirectional DShot feedback
    float rpm;               // rpm as read by the bidirectional DShot telemetry (needs a separate variable for the pid controller to use)
    float rpm_setpoint;      // desired rpm value
    float u;                 // motor duty cycle (-U_SCALE to U_SCALE, only used for bidirectional motors)
    float uS;                // motor throttle value (1000 to 2000 microseconds, only used for non-bidirectional motors)

    unsigned long timestamp; // can use this for instance-specific timing
};

namespace ESC
{
    /// @brief Initialize DSHOT communications with the ESCs
    void init();

    /// @brief Writes the calculated dshot_values to the ESCs
    void write_read();

    extern Motor drive1;
    extern Motor drive2;
    extern Motor drive3;
    extern Motor drive4;
    extern Motor weapon;
}
