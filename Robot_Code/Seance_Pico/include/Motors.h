/*
Handles the robot's communication with its 5 ESCs (4 drive, 1 weapon), using
bidirectional dshot to get RPM and voltage/current telemetry
*/

#include <dshot/esc.h> // https://github.com/josephduchesne/pico-dshot-bidir

#pragma once

namespace Motors
{

    class motor
    {

    public:
        motor(const char *_name, unsigned int _dshot_gpio, unsigned int _poles, PIO _pio);

        void dshot_begin();

    private:
        const char *name;
        unsigned int dshot_gpio;
        unsigned int poles;
        PIO pio;
        unsigned int _dshot_gpio;

        DShot::ESC dshot;             // dshot ESC object to read and write directly to the ESC
        DShot::Telemetry telem = {0}; // decoded telemetry data
        uint64_t raw_telem;           // raw telemetry data
        
        unsigned long timestamp;
    };

    // Initialize DSHOT communications with the ESCs
    void init();

    // Maps drive plant input u (-u_scale to u_scale) to the 3D Mode DShot scale
    int map_3d(int u);

    // Map weapon throttle (1000 to 2000 us) to the 1-direction DShot scale
    int map_wpn(int us);

    // Write DSHOT values to the ESCs, and read the telemetry replies
    void write_read();

}
