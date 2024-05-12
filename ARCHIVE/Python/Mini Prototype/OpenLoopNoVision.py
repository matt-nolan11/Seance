'''
MAE 598 Applied Project: Open Loop Control Without Computer Vision
Matthew Nolan

Open-loop control without computer vision considering the inverse kinematics of
the robot. Assumes that the angle of the robot is always 0, which results in
robot-frame control rather than world-frame control. It's objectively harder to
drive the robot this way, but doesn't require any integration with
computer vision from a drone or other camera sources, making it easier to
implement.
'''

import serial
import serial.tools.list_ports
import time
import Inverse_Kinematics as IK
import ProControllerInputs as joy
import numpy as np
from scipy.interpolate import interp1d
import keyboard

COLLECT_DATA = False # set whether to collect input data
ports = list(serial.tools.list_ports.comports())  # get list of serial devices plugged into the ports
for p in ports:
    if "CH340" in str(p):
        port = str(p)[-5:-1] # get the COM port info from the last 4 digits of the device info
        arduino = serial.Serial(port=port, baudrate=115200, timeout=None) # serial connection object

outputRange = 1000 # maximum Serial output value to send to the ESP8266

kv = 730/12 # drive motor kv (730 RPM at gearbox output @ 12V nominal)
gear_ratio = 1 # drive gearbox ratio (external)
V = 3*4.2 # battery voltage, 3s lipo, 4.2V per cell @ full charge
max_rpm = kv * V / gear_ratio # maximum theoretical rpm of the drive motors
interp = interp1d([-max_rpm, max_rpm], [-outputRange, outputRange]) # interpolator to convert rpm values to those readable by the ESP8266

previousTime = 0 # stores the "previous time"

theta = 0.0 # angle that the robot is facing (will eventually get from the drone/camera)


def rad_to_channel(rad): # converts wheel speeds in rad/s to RC channel values from 0 to 1000
    rpm = rad*60/(2*np.pi) # convert rad/s to rpm
    
    if any(abs(rpm) > max_rpm): # if any of the required rpm values are outside the possible range
        rpm = rpm/max(abs(rpm))*max_rpm # scale all the rpm values down equally so that they are in range
        
    channels = interp(rpm) # interpolate rpm values to be in range of the Serial output
    return channels

def package(channels): # package channel values into a single output string, to be send over Serial to the ESP8266
    # convert channel values to ints for simpler data transfer
    outString = f"<{int(channels[0])},{int(channels[1])},{int(channels[2])},{int(channels[3])}>"
    return outString

def serialWrite(output): # Serial write to the ESP8266 with non-blocking delays
    global previousTime # pull in the global value for the previous time
    currentTime = time.time() # get the current program time
    
    if currentTime - previousTime >= 0.01: # if at least 10ms have passed since the last write
        arduino.write(bytes(output, "utf-8")) # write the output string to the ESP8266 board
        previousTime = currentTime # update previous time

def joy2arduino(): # handles the entire input/output process
    xdot, ydot, thetadot = joy.get_sticks() # get the stick values and read into desired velocity variables
    
    # compute the wheel velocities (rad/s) required to achieve the specified input robot velocity 
    omega = IK.ik([[xdot],[ydot],[thetadot]], theta)
    
    channels = rad_to_channel(omega) # convert wheel velocities to a range from 0 to 1000
    serialOutput = package(channels) # package channels as a single formatted output string
    serialWrite(serialOutput) # write the output to the ESP8266
    
    
while True: # main operating loop
    joy2arduino()
    
    if keyboard.is_pressed("esc"):
        break
    
arduino.write(bytes("<0,0,0,0>", "utf-8")) # set all channels to 0
arduino.close()
