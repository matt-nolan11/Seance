"""
MAE 598 Applied Project: Switch Pro Controller Input Handling
Matthew Nolan

Sets deadzone values for the controller sticks and contains a simple function
to get the stick data in terms of the maximum linear speed (inches/s) and
rotational speed (radians/s)
                       
"""

import XInput
import numpy as np

XInput.set_deadzone(XInput.DEADZONE_LEFT_THUMB, 8000)
XInput.set_deadzone(XInput.DEADZONE_RIGHT_THUMB, 8000)

maxLin = 3 # maximum desired linear speed in ft/s
maxRot = 0.75*np.pi # maximum desired rotational speed in rad/s

def get_sticks():
    state = XInput.get_state(0) # get the state of the whole controller
    # read stick values into an array
    sticks = XInput.get_thumb_values(state)
    LX = sticks[0][0]*maxLin*12 # multiply by 12 to convert into inches/s
    LY = sticks[0][1]*maxLin*12
    RX = -sticks[1][0]*maxRot # multiply by -1 to fit the sign convention of positive counterclockwise rotations
    return LX, LY, RX