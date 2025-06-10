'''
MAE 598 Applied Project: Switched System
Matthew Nolan

Switched system that alternates between full open-loop and hybrid open/closed 
loop depending on the user input. 
'''

import serial
import serial.tools.list_ports
import time
import Inverse_Kinematics as IK # custom
import ProControllerInputs as joy # custom
import ArUco_Pose_Estimation as APE # custom
import numpy as np
from scipy.interpolate import interp1d
import keyboard
import cv2
import DataCapture2 as DC2 # custom

PLOT_DATA = True # sets whether to plot the measured and reference data in real time
SAVE_DATA = False # sets whether to save the measured and reference data (cannot be set True if PLOT_DATA is not also True)

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

previousTime = 0 # stores the "previous time," in order to time Serial output commands so as to not overload the ESP8266
previousTrackTime = 0 # stores the previous time for ArUco tracking
previousCollectTime = 0 # previous time for data collection


def clearCapture(capture):
    capture.release()
    cv2.destroyAllWindows()
    
def countCameras():
    for i in range(10):
        try:
            cap = cv2.VideoCapture(i)
            ret, frame = cap.read()
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clearCapture(cap)
        except:
            clearCapture(cap)
            break
    return i-1
        
video = cv2.VideoCapture(countCameras()) # uses the last camera (to account for internal webcams on laptops)
time.sleep(2.0)

z = np.array([0, 0, 0]) # initial estimate for the robot's position [x, y, theta]

group_z = [] # group of position estimate vectors, to compute a running median
num_z = 1 # number of z vectors to include in the median (if 1, don't take a running median at all)

sensor_dt = 1/25 # time period enforced between computer vision readings, for consistency
track_dt = 1/25 # time period enforced for updating the live data plotting

kp = 1.5 # proportional constant
ki = 0.75 # integral constant
kd = 0.01 # derivative time constant
i = 0 # initialize the integrator for the angle error
d = 0 # initialize the derivator for the angle error
previous_e = 0 # store the value of the previous error, for derivation

d = 1.95 # distance between center of marker and center of mass
r = np.array([[0],
              [d],
              [0]]) # vector from the center of the ArUco marker to the center of mass


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
        
def ARuCo_track():
    global z
    ret, frame = video.read()
    output, tvec, euler = APE.pose_estimation(frame) # returns output frame with axes drawn, translation vector of the ArUco marker, and the euler angles representing the rotation of the marker
    if not np.isnan(euler).any():
        # collect position measurement z = [x, y, theta]
        z = np.array([tvec[0][0][0], -tvec[0][0][1], -euler[2]]) # negative to fit sign convention of positive counterclockwise angles
        #print(z) # for debugging
    cv2.imshow('Estimated Pose', output)
    
    return z, ret

def PI_controller(e):
    global i, previous_e
    i += e*sensor_dt # increment the integrator
    
    d = (e - previous_e)/sensor_dt # calculate current error derivative
    
    if abs(e) < 0.01: # if the error on a wheel is less than some threshold
        i = 0 # reset the corresponding integrator, to prevent integral windup
    
    u = kp*e + ki*i + kd*d # PID control law, computes corrective angular velocity to achieve specified angle
    
    if u > joy.maxRot: # if the calculated input is 
        u = joy.maxRot # clamp maximum commanded rotational velocity
    
    return u

def joy2ESP(theta): # handles the entire joystick input/output process
    global previous_thetadot, theta_target, i # pull in global values for previuos desired thetadot and current theta target, and also the integrator
    xdot, ydot, thetadot = joy.get_sticks() # get the stick values and read into desired velocity variables
    
    if thetadot == 0 and previous_thetadot != thetadot: # if the desired thetadot just hit 0
        theta_target = theta # update the theta_target
        i = 0 # reset the integrator
    
    if thetadot == 0: # if the system is in angular position control mode
        error = theta_target - theta # calculate current angular position error
        auto_thetadot = PI_controller(error) # update corrective angular velocity to maintain theta_target
        omega = IK.ik([[xdot],[ydot],[auto_thetadot]], theta)
    else:
        # compute the wheel velocities (rad/s) required to achieve the specified input robot velocity 
        omega = IK.ik([[xdot],[ydot],[thetadot]], theta)
        
    channels = rad_to_channel(omega) # convert wheel velocities to a range from 0 to 1000
    serialOutput = package(channels) # package channels as a single formatted output string
    serialWrite(serialOutput) # write the output to the ESP8266
    return xdot, ydot, thetadot

#-------------------------------------------------------------------------------------------------

start_time = time.time() # time that the main loop starts rolling
previous_thetadot = 0 # stores the previous desired thetadot
theta_target = 0 # stores the angular target

while True: # main loop
    currentTime = time.time() # get the current program time
        
    if currentTime - previousTrackTime >= sensor_dt: # if enough time has passed to estimate a new robot velocity
        z, ret = ARuCo_track() # calculate the current translation vector and yaw angle of the ArUco marker (I dunno what ret does exactly, but stuff breaks without it ¯\_(ツ)_/¯)
        if not ret:
            print("No frames available!")
            break
        group_z.append(z)
        if len(group_z) > num_z:
            group_z.pop(0)
        median_z = np.median(np.array(group_z), axis=0) # compute running median
        theta = median_z[2] # set the value of the robot's theta, based on ArUco tracking feedback
        
        previousTrackTime = currentTime # update previousTrackTime
        #print(median_v_hat) # for debugging
     
    xdot, ydot, thetadot = joy2ESP(theta) # take in joystick inputs and send motor channel values to the ESP8266
    
    previous_thetadot = thetadot # update the previous thetadot
     
    if PLOT_DATA and currentTime - previousCollectTime >= track_dt:
        
        DC2.animate([xdot, ydot, thetadot, theta_target], [theta], start_time)
            
    if keyboard.is_pressed("esc"):
        break
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): # if q is pressed
        break

#-----------------------------------------------------------------------------------------------

arduino.write(bytes("<0,0,0,0>", "utf-8")) # set all channels to 0
arduino.close() # close the Serial port connection
video.release() # close the camera connection
cv2.destroyAllWindows() # close the camera viewing window

if SAVE_DATA:
    DC2.write_data('SwitchedSystem Data/Data') # data/time will automatically be appended to this filename