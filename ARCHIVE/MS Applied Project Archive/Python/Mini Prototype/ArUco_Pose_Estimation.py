'''
MAE 598 Applied Project: ArUco Marker Pose Estimation
Matthew Nolan

Tracks the linear and rotational position (in inches and radians, respectively)
of the AruCo marker on the robot over time, and draws the marker border and
pose axes on a window showing the camera stream. Also includes the option to 
report the framerate.
'''


import cv2
import numpy as np
import math
import time

calibration_matrix_path = "ArUco/calibration_matrix.npy"
distortion_coefficients_path = "ArUco/distortion_coefficients.npy"

k = np.load(calibration_matrix_path) # load in the camera calibration matrix
d = np.load(distortion_coefficients_path) # load in the camera distortion coefficient matrix

aruco_dict_type = cv2.aruco.DICT_4X4_50 # type dictionary for the ArUco marker format
marker_id = 0 # change this to change which markers you want to detect! Should probably switch it up between matches
markerLength = 3.75 # length of the marker sizes, in inches
previousFrameTime = 0 # stores the previous time for the video frame, for computing the framerate
font = cv2.FONT_HERSHEY_SIMPLEX # font to use for text drawing on the video window
all_fps = [] # stores a list of fps values, to take a running average
fps_mean_count = 8 # sets the number of terms in the running average

FPS_COUNTER = False # controls whether the FPS is calculated and displayed

def rotM2Euler(R): # convertes the rotation matrix to Euler angles
    Euler = np.array([
        math.atan2(R[2][1], R[2][2]),
        math.atan2(-R[2][0], math.sqrt(R[2][1]**2+R[2][2]**2)),
        math.atan2(R[1][0], R[0][0])
        ])
 
    return Euler

def pose_estimation(frame):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type) # set ArUco marker type
    parameters = cv2.aruco.DetectorParameters_create() # create detector parameters

    global previousFrameTime
    
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters) # detect marker corners and ids

        # If markers are detected
    if len(corners) > 0 and (ids == marker_id).any():
        for i in range(0, len(ids)): # provisions for tracking multiple markers down the line
            
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, k, d)
                rotMat,_ = cv2.Rodrigues(rvec) # converts Rodriguez (modified axis-angle) representation to rotation matrix
                euler = rotM2Euler(rotMat) # converts rotation matrix to Euler angles 
                
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners) 
    
                # Draw Axes
                cv2.drawFrameAxes(frame, k, d, rvec, tvec, markerLength/2)
                
    else:
        euler = np.nan
        tvec = np.nan
    
    if FPS_COUNTER:
        newFrameTime = time.time() # current frame time
        fps_now = 1/(newFrameTime-previousFrameTime) # calculate the current fps
        all_fps.append(fps_now)
        if len(all_fps) > fps_mean_count:
            all_fps.pop(0) # remove the first (oldest) element from the list
            
        mean_fps = sum(all_fps)/fps_mean_count
        mean_fps = str(int(mean_fps)) # convert to an int, and then into a string so it can be printed on the output window
        cv2.putText(frame, mean_fps, (7, 30), font, 1, (100, 255, 0), 2, cv2.LINE_AA) # put the FPS count on the frame
        
        previousFrameTime = newFrameTime # update previous frame time
    
    return frame, tvec, euler



