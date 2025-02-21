# RP2350 Robot Code
This Platformio project contains on-robot firmware that is designed specifically around the capabilities and peripherals of the RP2350 MCU. Specifically, it takes advantage of the 12 PIO state machines (compared to the 8 available on the RP2040) to enable the necessary UART / DSHOT communication. It uses Earle Philhower's Arduino-Pico core.
## MCU Resources
- [RP2350 Getting Started / Pinouts](https://wiki.seeedstudio.com/getting-started-xiao-rp2350/)
- [Arduino-Pico with PlatformIO](https://arduino-pico.readthedocs.io/en/latest/platformio.html)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
## Dependencies
This code relies on a set of external libraries to communicate with all of the robot hardware.
| Library | Use Case |
| ------- | -------- |
| [CRSFforArduino](https://github.com/ZZ-Cat/CRSFforArduino.git) | Communicate with an ELRS receiver over UART to read channel values and send telemetry packets back to the transmitter |
| [pico-dshot-bidir](https://github.com/josephduchesne/pico-dshot-bidir) | Use bidirectional DSHOT to communicate with BLHeli_32 or AM32 ESCs, sending throttle values and receiving ESC telemetry (RPM, current, temperature, etc.) |
| [QuickPID](https://github.com/Dlloydev/QuickPID.git) | Implement PID control loops for motor speeds, robot world-frame speeds, etc. |
| IMU Library (TBD) | Communicate with an IMU, probably over I2C or UART, to read the robot's linear accelerations and angular velocities. The exact IMU choice is still TBD