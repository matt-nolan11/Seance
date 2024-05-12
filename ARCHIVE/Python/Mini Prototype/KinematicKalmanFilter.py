"""
MAE 598 Applied Project: Open Loop Control With Computer Vision
Matthew Nolan

Estimates the linear and rotational velocities of the robot from the linear and
rotational positions over time using a Kalman filter with a second-order 
kinematic model. 
"""

from filterpy.common import kinematic_kf # implements a kinematic kalman filter (i.e. estimate position and velocity using only position measurements)
from filterpy.common import Q_discrete_white_noise # creates a discrewte Gaussian (white) noise model
import numpy as np


order = 2 # order of the kinematics applied (1 = constant velocity, 2 = constant acceleration, etc)
# function to initialize the kalman filter
def init(dt): # inputs are the sensor measurement z and the (fixed) time between measurements
    kf = kinematic_kf(dim=3, order=order, dim_z=1, dt=dt, order_by_dim=False)
    kf.alpha = 1.2 # set a fading factor
    kf.x = np.zeros([3*(order+1),]) # initial state estimate
    P_diag = 10000*np.ones((order+1)*3)
    P_diag[0:3] = [100, 100, 100]
    kf.P = np.diag(P_diag) # initial state covariance matrix (initializes to eye(dim**order), multiplied by 1000 here)
    kf.R *= 10000 # measurement noise covariance
    kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=1000, block_size=order+1, order_by_dim=False) # process noise covariance
    return kf

# function to estimate the robot state based on the measured position over time
def estimate(kf, z): # inputs are the initialized filter model and the current sensor reading
    kf.predict() # predict system state and covariance
    kf.update(z) # compute Kalman gain and update state estimate
    state = kf.x # pull out state estimate vector
    vhat = state[3:6] # pull out the three estimated velocities from the 9-d state vector
    return vhat