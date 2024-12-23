#include <Motors.h>
#include <Comms.h>
#include <Controls.h>
#include <Utilities.h>

// DSHOT Control Signal Configs
constexpr int DRIVE_MOTOR_POLES = 14;                     // number of poles on the drive motors
constexpr int WEAPON_MOTOR_POLES = 14;                    // number of poles on the weapon motor
constexpr DShot::Type DSHOT_TYPE = DShot::Type::Bidir;    // DShot communication type (normal or bidirectional)
constexpr DShot::Speed DSHOT_SPEED = DShot::Speed::DS600; // DShot communication speed

constexpr float DRIVE_GEAR_RATIO = 22. * 2 / 1;
constexpr float WEAPON_GEAR_RATIO = 32. / 28;

// ESC signal pin config
constexpr int FR_ESC_PIN = 26; // signal pin for front right motor
constexpr int FL_ESC_PIN = 27; // front left
constexpr int BL_ESC_PIN = 28; // back left
constexpr int BR_ESC_PIN = 29; // back right
constexpr int WPN_ESC_PIN = 2; // weapon

// Class-level variable / method definitions
int Motor::num_motors;                 // class-level variable to keep track of the number of motors in the program
std::vector<Motor *> Motor::instances; // class-level vector that contains references to all the created motor instances

int Motor::getNumMotors()
{
    return num_motors;
}

std::vector<Motor *> Motor::getInstances()
{
    return instances;
}

// Instance-level method definitions

Motor::Motor(const char *_name,
             unsigned int _dshot_gpio,
             float _gear_ratio,
             unsigned int _poles = 14,
             PIO _pio = pio0) : name(_name),
                                dshot_gpio(_dshot_gpio),
                                gear_ratio(_gear_ratio),
                                poles(_poles),
                                pio(_pio),
                                dshot(dshot_gpio, pio, DSHOT_TYPE, DSHOT_SPEED, poles),
                                rpm_controller(&rpm, &u, &rpm_setpoint)
{
    num_motors += 1;
    instances.push_back(this);
    rpm_controller.SetMode(QuickPID::Control::manual); // pid speed controller is initially turned off
}

void Motor::dshotBegin()
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
    while (millis() - timestamp <= 500)
    {
        dshot.setCommand(13); // enable extended telemetry
        delay(1);
        dshot.getRawTelemetry(raw_telem);
    }

    dshot.setCommand(3); // DSHOT_CMD_BEEP3
    delay(300);
}

void Motor::setDuty_U(int input_u)
{
    u = constrain(input_u, -Controls::U_SCALE, Controls::U_SCALE);
    dshot_value = Utilities::map_bidir(u);
}

void Motor::setDuty_uS(int input_uS)
{
    uS = constrain(input_uS, 1000, 2000);
    dshot_value = Utilities::map_uS_bidir(uS);
}

void Motor::sendDshot()
{
    dshot.setCommand(dshot_value);
}

void Motor::readTelem()
{
    dshot.getRawTelemetry(raw_telem);        // grab raw ESC telemetry
    dshot.decodeTelemetry(raw_telem, telem); // decode raw telemetry and populate the telem struct with it
}

namespace ESC
{

    Motor drive1("drive1", FR_ESC_PIN, 1, DRIVE_MOTOR_POLES, pio0);
    Motor drive2("drive2", FL_ESC_PIN, 1, DRIVE_MOTOR_POLES, pio0);
    Motor drive3("drive3", BL_ESC_PIN, 1, DRIVE_MOTOR_POLES, pio0);
    Motor drive4("drive4", BR_ESC_PIN, 1, DRIVE_MOTOR_POLES, pio0);
    Motor weapon("weapon", WPN_ESC_PIN, 1, WEAPON_MOTOR_POLES, pio1);

    std::vector<Motor *> drivetrain = {&drive1, &drive2, &drive3, &drive4};

    void init()
    {
        for (auto &m : Motor::getInstances())
        {
            m->dshotBegin();
        }
    }

    void write_read()
    {
        for (auto &m : Motor::getInstances())
        {
            m->sendDshot();
        }
        delay(1);
        for (auto &m : Motor::getInstances())
        {
            m->readTelem();
        }
    }
}