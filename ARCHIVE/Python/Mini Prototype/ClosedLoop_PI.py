'''
MAE 598 Applied Project: Closed Loop PI Control
Matthew Nolan

Closed-loop control with computer vision considering the inverse kinematics of
the robot. Uses feedback to achieve world-frame control, estimates the actual 
world-frame velocities of the robot with a Kalman filter, estimates the actual
wheel velocities through the inverse kinematics, and uses a PID controller to
compute the voltages that should be applied to the wheels based on the wheel
velocity error.
'''

import serial
import serial.tools.list_ports
import time
import Inverse_Kinematics as IK # custom
import ProControllerInputs as joy # custom
import ArUco_Pose_Estimation as APE # custom
import KinematicKalmanFilter as KF # custom
import numpy as np
from scipy.interpolate import interp1d
import keyboard
import cv2
import DataCapture as DC # custom

PLOT_DATA = True # sets whether to plot the measured and reference data in real time
SAVE_DATA = True # sets whether to save the measured and reference data (cannot be set True if PLOT_DATA is not also True)

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

group_vhat = [] # group of velocity estimation vectors, to compute a running median
num_vhat = 5 # number of vhat vectors to include in the median

group_z = [] # group of position estimate vectors, to compute a running median
num_z = 1 # number of z vectors to include in the median

sensor_dt = 1/25 # time period enforced between computer vision readings, for consistency
track_dt = 1/25 # time period enforced for updating the live data plotting
kf = KF.init(sensor_dt) # initialize the Kalman filter

kp = 5 # proportional constant
ki = 1 # integral constant
i = np.array([[0.],[0.],[0.],[0.]]) # initialize the integrator for each wheel

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
    global i
    i += e*sensor_dt # increment the integrator
    
    for index, error in enumerate(e):
        if abs(error) < 0.01: # if the error on a wheel is less than some threshold
            i[index] = 0 # reset the corresponding integrator, to prevent integral windup
    
    u = kp*e + ki*i # PI control law, computes outputs
    
    if any(abs(u) > outputRange): # if any of the control inputs are outside the allowable range
        u = u/max(abs(u))*outputRange # manually scale them down by the same amount such that the largest value is at the edge of the range
    
    return u

def joy2ESP(omega_hat, theta): # handles the entire joystick input/output process
    xdot, ydot, thetadot = joy.get_sticks() # get the stick values and read into desired velocity variables
    
    # compute the wheel velocities (rad/s) required to achieve the specified input robot velocity 
    omega = IK.ik([[xdot],[ydot],[thetadot]], theta)
    error = omega - omega_hat # calculate the controller error
    
    u = PI_controller(error) # compute control inputs (channel values) based on the error
    
    serialOutput = package(u) # package channels as a single formatted output string
    serialWrite(serialOutput) # write the output to the ESP8266
    return xdot, ydot, thetadot

#-------------------------------------------------------------------------------------------------

start_time = time.time() # time that the main loop starts rolling

while True: # main loop
    currentTime = time.time() # get the current program time
    
    z, ret = ARuCo_track() # calculate the current translation vector and yaw angle of the ArUco marker (I dunno what ret does exactly, but stuff breaks without it ¯\_(ツ)_/¯)
    if not ret:
        print("No frames available!")
        break
    group_z.append(z)
    if len(group_z) > num_z:
        group_z.pop(0)
    median_z = np.median(np.array(group_z), axis=0) # compute running median
    theta = median_z[2] # set the value of the robot's theta, based on ArUco tracking feedback
    
    r[0:2] = [[-d*np.sin(theta)], [d*np.cos(theta)]] # re-calculate vector from center of ArUco marker to center of mass
    
    if currentTime - previousTrackTime >= sensor_dt: # if enough time has passed to estimate a new robot velocity
        v_hat = KF.estimate(kf, median_z) # estimate actual robot velocities [xdot, ydot, thetadot] through the Kalman filter
        v_hat += (v_hat[2]*IK.Q @ r).reshape([3,]) # update linear velocities to compensate for offset between centar of ArUco marker and center of rotation
        
        group_vhat.append(v_hat)
        if len(group_vhat) > num_vhat:
            group_vhat.pop(0) # remove the first (oldest) element from the list
        median_v_hat = np.median(np.array(group_vhat), axis=0)
        previousTrackTime = currentTime # update previousTrackTime
        #print(median_v_hat) # for debugging
        
    omega_hat = IK.ik([[median_v_hat[0]],[median_v_hat[1]],[median_v_hat[2]]], theta) # estimate wheel velocities from robot velocity
     
    xdot, ydot, thetadot = joy2ESP(omega_hat, theta) # take in joystick inputs and send motor channel values to the ESP8266
     
    if PLOT_DATA and currentTime - previousCollectTime >= track_dt:
        
        DC.animate([xdot, ydot, thetadot, 0], [median_v_hat[0], median_v_hat[1], median_v_hat[2], median_z[0], median_z[1], theta], start_time)
            
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
    DC.write_data('ClosedLoopPI Data/Data')