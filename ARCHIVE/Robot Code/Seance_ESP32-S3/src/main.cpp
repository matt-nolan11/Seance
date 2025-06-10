#include <CRSFforArduino.hpp>
#include <ESP32Servo.h>
#include <ESP32_DSHOT.h>

#define printChannels false // whether or not we want to print the received RC channels, for debugging

// Global class declaration as a null pointer.
CRSFforArduino *crsf = nullptr;

// A flag to hold the fail-safe status.
bool isFailsafeActive = false;

// RC Channels labels.
int rcChannelCount = 16;
const char *rcChannelNames[] = {
    "xdot",
    "ydot",
    "thetadot",
    "cv_theta",
    "Aux1",
    "Wpn",
    "Aux2",
    "Aux3",
    "Aux4",
    "Aux5",
    "Aux6",
    "Aux7",
    "Aux8",
    "Aux9",
    "Aux10",
    "Aux11"};

#define xdot_channel 0
#define ydot_channel 1
#define thetadot_channel 2
#define cv_theta_channel 3
#define wpn_channel 5 // skip channel 4 (AUX1) as it is reserved strictly for arming/disarming

// Input value scaling
#define maxXDot 10.0 * 12        // maximum xDot (inches per second)
#define maxYDot 10.0 * 12        // maximum yDot (inches per second)
#define maxThetaDot 3 * (2 * PI) // maximum thetaDot (radians per second)

// Global Input Variables
float xdot;     // x velocity (inches per second)
float ydot;     // y velocity (inches per second)
float thetadot; // angular velocity (radians per second)
float cv_theta; // yaw angle as measured by the computer vision (cv) system (radians)
int wpnThr;     // weapon throttle value (microseconds)

// DSHOT Control Signal Configs
#define DRIVE_MOTOR_POLES 14  // number of poles on the drive motors
#define WEAPON_MOTOR_POLES 14 // number of poles on the weapon motor
#define DSHOT_MODE DSHOT::DSHOT600_BIDIR
// ESC signal pin config
#define drv1Pin 6
#define drv2Pin 0
#define drv3Pin 0
#define drv4Pin 0
#define wpnPin 0
// DShotRMT objects
DSHOT drv1;
// DSHOT drv2;
// DSHOT drv3;
// DSHOT drv4;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) // map function that works with floating point values
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData)
{
    if (rcData->failsafe)
    {
        if (!isFailsafeActive)
        {
            isFailsafeActive = true;

            /* Centre all RC Channels, except for Channel 5 (Aux1). */
            for (int i = 0; i < rcChannelCount; i++)
            {
                if (i != 4 && i != wpn_channel)
                {
                    rcData->value[i] = CRSF_RC_CHANNEL_CENTER;
                }
            }

            /* Set Channel 5 (Aux1) and Channel 6 (Weapon) to their minimum values. */
            rcData->value[4] = CRSF_RC_CHANNEL_MIN;
            rcData->value[wpn_channel] = CRSF_RC_CHANNEL_MIN;
        }
        Serial.println("Failsafe active!");
    }
    else
    {
        /* Set the failsafe status to false. */
        if (isFailsafeActive)
        {
            isFailsafeActive = false;
        }
    }

    // RC channels implementation.
    // Grab channel values
    xdot = mapFloat(rcData->value[xdot_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, -maxXDot, maxXDot);                 // input x velocity (inches/second)
    ydot = mapFloat(rcData->value[ydot_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, -maxYDot, maxYDot);                 // input y velocity (inches/second)
    thetadot = mapFloat(rcData->value[thetadot_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, -maxThetaDot, maxThetaDot); // input angular velocity (radians/second)
    cv_theta = mapFloat(rcData->value[cv_theta_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, 0, 2 * PI);                 // robot yaw angle as measured by the computer vision (cv) system (radians)
    uint16_t wpn_us = crsf->rcToUs(rcData->value[wpn_channel]);                                                                              // get the weapon throttle in microseconds
    if (wpn_us <= 1010)
    {
        wpnThr = 0;
        
    }
    else
    {
        wpnThr = map(wpn_us, 1000, 2000, 48, 2047); // weapon speed mapped to its 11-bit DSHOT value
    }

    if (printChannels)
    {
        // Print RC channels to the Serial monitor
        static uint32_t lastPrint = millis(); // print timer

        if (millis() - lastPrint >= 10) // check if 10ms have passed
        {

            lastPrint = millis(); // update print timer

            Serial.print("RC Channels: <");
            for (int i = 0; i < rcChannelCount; i++)
            {
                Serial.print(rcChannelNames[i]);
                Serial.print(": ");
                Serial.print(crsf->rcToUs(rcData->value[i]));

                if (i < (rcChannelCount - 1))
                {
                    Serial.print(", ");
                }
            }
            Serial.println(">");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // initialize Servo objects to no motion
    // drv1.writeMicroseconds(1500);
    // drv2.writeMicroseconds(1500);
    // drv3.writeMicroseconds(1500);
    // drv4.writeMicroseconds(1500);
    // wpn.writeMicroseconds(988);

    // drv1.attach(drv1Pin);
    // drv2.attach(drv2Pin);
    // drv3.attach(drv3Pin);
    // drv4.attach(drv4Pin);
    // wpn.attach(wpnPin);

    drv1.begin(drv1Pin, DSHOT_MODE);

    crsf = new CRSFforArduino(&Serial1, 44, 43);

    /* Initialise CRSF for Arduino. */
    if (crsf->begin() == true)
    {
        /* CRSF for Arduino initialised successfully.
        We can now register the RC Channels event. */
        crsf->setRcChannelsCallback(onReceiveRcChannels);

        /* Constrain the RC Channels Count to the maximum number
        of channels that are specified by The Crossfire Protocol.*/
        rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;
    }
    else
    {
        /* Clean up any resources,
        if initialisation fails. */
        crsf->end();
        delete crsf;
        crsf = nullptr;
        Serial.println("CRSF Initialization Failed!");
    }

    DSHOT::arm();
    delay(500);
}

void loop()
{
    /* Guard CRSF for Arduino's API with a null check. */
    if (crsf != nullptr)
    {
        /* Call CRSF for Arduino's main function here. */
        crsf->update();
    }
    drv1.set(wpnThr);
    delay(1);
}
