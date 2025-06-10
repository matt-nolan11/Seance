/*
Main Script for Séance, a semi-autonomous, holonomic, 3lb "beetleweight" combat robot. This file
contains high-level control logic. The lower-level code is organized into the relevant header and
source files.
*/

#define DEBUG_MODE // Comment out this line to disable debug mode, which will remove all Serial prints

#include <Comms.h>
#include <Controls.h>
#include <Motors.h>
#include <Utilities.h>

unsigned long lastLoopTime = 0; // variable to store the duration of the last loop execution (in microseconds)

void setup()
{    
    Serial.begin(921600); // for debugging only, should comment this out in final robot code to maintain loop speed
    // Comms::init();
    // ESC::init();

}

void loop()
{
#ifdef DEBUG_MODE
    unsigned long loopStartTime = micros(); // Record the time at the start of the loop
#endif

    // Comms::read();
    // ESC::write_read();

#ifdef DEBUG_MODE
    // Calculate the time elapsed since the last loop execution
    unsigned long loopEndTime = micros();
    unsigned long dt = loopEndTime - loopStartTime; // dt is in microseconds

    // Print statements for checking various loop parameters
    // Note that these print statements will take additional time that slows the loop down, but the dt calculation does not consider this additional time
    // Thus, the printed loop time should be accurate for the scenario when DEBUG_MODE is disabled

    Serial.printf("Loop time (microseconds): %d\n", dt);
    // Serial.printf("Clock Speed (MHz): %.2f\n", clock_get_hz(clk_sys) / 1000000.0);
    // Serial.printf("millis() = %d\n", millis());
    // Serial.printf("MCU Temperature (degC): %.2f\n", analogReadTemp());
#endif
}
