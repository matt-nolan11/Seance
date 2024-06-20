/*
Main Script for Séance, a semi-autonomous, holonomic, 3lb "beetleweight" combat robot. This file
contains high-level control logic. The lower-level code is organized into the relevant header and
source files.
*/

#include <Comms.h>
#include <Controls.h>
#include <ESC.h>

void setup()
{
    Serial.begin(921600); // for debugging only
    Comms::init();
    ESC::init();
}

void loop()
{
   Comms::read(); // read reference signals (r) and external data from the CRSF receiver
   Controls::calc_u(); // calculate plant inputs (u) based on reference signals and internal/external sensor data
   ESC::write_read(); // write plant inputs (u) to the ESC's and read the telemetry
}
