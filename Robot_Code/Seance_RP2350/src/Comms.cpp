/*
Handles the robot's communications with the ELRS receiver using the CRSF serial protocol
*/
#include <CRSFforArduino.hpp> // https://github.com/ZZ-Cat/CRSFforArduino.git
#include <Utilities.h>
#include <Comms.h>

namespace Comms
{

    // Global Input Variables
    float xdot;       // desired x velocity (inches per second)
    float ydot;       // desired y velocity (inches per second)
    float thetadot;   // desired angular velocity (radians per second)
    float cv_theta;   // yaw angle as measured by the computer vision (cv) system (radians)
    int wpn_throttle; // weapon throttle value (microseconds)

    // Global class declaration as a null pointer.
    CRSFforArduino *crsf = nullptr;

    // A flag to hold the fail-safe status.
    bool isFailsafeActive = false;

    // RC Channels labels.
    int rcChannelCount = 16;

    void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData)
    {
        if (rcData->failsafe)
        {
            if (!isFailsafeActive)
            {
                isFailsafeActive = true;

                // Centre all RC channels, except for channels 4 (ARM/AUX1) and 5 (Wpn/Aux2)
                for (int i = 0; i < rcChannelCount; i++)
                {
                    if (i != 4 && i != wpn_channel)
                    {
                        rcData->value[i] = CRSF_RC_CHANNEL_CENTER;
                    }
                }

                // Set channel 4 (ARM/Aux1) and channel 5 (Wpn/Aux2) to their minimum values
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
        xdot = Utilities::mapFloat(rcData->value[xdot_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, -maxXDot, maxXDot);                 // input x velocity (inches/second)
        ydot = Utilities::mapFloat(rcData->value[ydot_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, -maxYDot, maxYDot);                 // input y velocity (inches/second)
        thetadot = Utilities::mapFloat(rcData->value[thetadot_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, -maxThetaDot, maxThetaDot); // input angular velocity (radians/second)
        cv_theta = Utilities::mapFloat(rcData->value[cv_theta_channel], CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, 0, 2 * PI);                 // robot yaw angle as measured by the computer vision (cv) system (radians)
        wpn_throttle = crsf->rcToUs(rcData->value[wpn_channel]);                                                                              // get the weapon throttle in microseconds
    }

    void init()
    {
        crsf = new CRSFforArduino(&Serial1, 0, 1);
        /* Initialise CRSF for Arduino. */
        if (crsf->begin() == true)
        {
            crsf->setRcChannelsCallback(onReceiveRcChannels);
            rcChannelCount = min(rcChannelCount, crsfProtocol::RC_CHANNEL_COUNT);
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
    }

    void read() // Guard CRSF for Arduino's API with a null check, and call the main update function
    {
        if (crsf != nullptr)
        {
            crsf->update();
        }
    }
}