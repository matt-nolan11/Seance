/*
Main Script for Séance, a semi-autonomous, holonomic, 3lb "beetleweight" combat robot. This file
contains high-level control logic. The lower-level code is organized into the relevant header and
source files.
*/

#define DEBUG_MODE // Comment out this line to disable debug mode, which will remove all Serial prints

#include <Comms.h>
#include <Controls.h>
#include <Motors.h>

unsigned long lastLoopTime = 0; // variable to store the duration of the last loop execution (in microseconds)

void setup()
{
    Serial.begin(921600); // for debugging only, should comment this out in final robot code to maintain loop speed
    Comms::init();
    ESC::init();

    lastLoopTime = micros(); // Record the initial time
}

void loop()
{
    Comms::read();
    ESC::write_read();

#ifdef DEBUG_MODE
    // Calculate the time elapsed since the last loop execution
    unsigned long currentLoopTime = micros();
    unsigned long dt = currentLoopTime - lastLoopTime; // dt is in microseconds
    lastLoopTime = currentLoopTime;

    // Print loop execution time for debugging
    Serial.print("Loop time (microseconds): ");
    Serial.println(dt);
#endif
}
