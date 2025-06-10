"""
MAE 598 Applied Project: Data Capture
Matthew Nolan

Functions for live plotting sensor and input data, and then saving said data to
.csv files for later analysis and visualziation.
"""

import csv
import time
from matplotlib import pyplot as plt

datetime = '-'.join([str(p) for p in list(time.localtime()[0:3])])+" "+';'.join([str(p) for p in list(time.localtime()[3:6])]) # current date time, converted into a string
fieldnames = ['Time',
              'Reference X Velocity',
              'Reference Y Velocity',
              'Reference Angular Velocity',
              'Reference Angle',
              'Measured Angle'] # column names for csv output file

window = 120 # number of points to keep on the x-axis, such that the live plotting x window moves in time

num_plots = 4
fig = plt.figure(figsize=(20,8),) # create figure for plotting
ax1 = fig.add_subplot(2,2,1) # subplot 1, x velocity
ax2 = fig.add_subplot(2,2,2) # subplot 2, y velocity
ax3 = fig.add_subplot(2,2,3) # subplot 3, angular velocity
ax6 = fig.add_subplot(2,2,4) # subplot 6, angular position (only needed for the switched velocity/position system)

# set up lines to plot, will update the contents of the plot in the animate function
xdot_r, = ax1.plot(0,0,label="Reference Values") # requested x velocity

ydot_r, = ax2.plot(0,0,label="Reference Values") # requested y velocity

thetadot_r, = ax3.plot(0,0,label="Reference Values") # requested angular velocity

theta_r, = ax6.plot(0,0,label="Reference Values") # requested angle
theta_m, = ax6.plot(0,0,label="Measured Values") # measured angle


# Add chart labels and legends
fig.suptitle("Measured vs Reference Values", fontsize=15)

ax1.set_title("X Velocity")
ax2.set_title("Y Velocity")
ax3.set_title("Angular Velocity")
ax6.set_title("Angular Position")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("X Velocity (in/s)")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Y Velocity (in/s)")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Angular Velocity (rad/s)")
ax6.set_xlabel("Time (s)")
ax6.set_ylabel("Angular Position (rad)")

ax1.legend()
ax2.legend()
ax3.legend()
ax6.legend()

fig.tight_layout(pad=2)

# define empty lists to store the data to be plotted and saved
xdot_r_data = []
ydot_r_data = []
thetadot_r_data = []
theta_r_data = []

theta_m_data = []

t_data = [] # time data

def animate(reference, measured, start_time): # reference and measured lists are in the order [xdot, ydot, thetadot, theta]
    t_data.append(time.time() - start_time) # calculate the current program time vector
    # append input values to their data storage lists    
    xdot_r_data.append(reference[0])
    ydot_r_data.append(reference[1])
    thetadot_r_data.append(reference[2])
    theta_r_data.append(reference[3])
    
    theta_m_data.append(measured[0])
    
    
    # set the y data to be plotted
    xdot_r.set_ydata(xdot_r_data[-window:]) # only plot the last {window} data points
    ydot_r.set_ydata(ydot_r_data[-window:])
    thetadot_r.set_ydata(thetadot_r_data[-window:])
    theta_r.set_ydata(theta_r_data[-window:])
    
    theta_m.set_ydata(theta_m_data[-window:])
    
    # set the x data to be plotted
    for item in [xdot_r, ydot_r, thetadot_r, theta_r, theta_m]:
        item.set_xdata(t_data[-window:])
        
    # calculate the window range of each chart in each dimension
    tmin = min(t_data[-window:]) # all charts have the same x limits in time
    tmax = max(t_data[-window:])
    
    ax1_ymin = min(xdot_r_data[-window:])
    ax2_ymin = min(ydot_r_data[-window:])
    ax3_ymin = min(thetadot_r_data[-window:])
    ax6_ymin = min(min(theta_r_data[-window:]), min(theta_m_data[-window:]))
    
    ax1_ymax = max(xdot_r_data[-window:])
    ax2_ymax = max(ydot_r_data[-window:])
    ax3_ymax = max(thetadot_r_data[-window:])
    ax6_ymax = max(max(theta_r_data[-window:]), max(theta_m_data[-window:]))
    
    # apply calculated window ranges
    for plot in [ax1, ax2, ax3, ax6]:
        plot.set_xlim(tmin, tmax)
    
    ax1.set_ylim(ax1_ymin, ax1_ymax)
    ax2.set_ylim(ax2_ymin, ax2_ymax)
    ax3.set_ylim(ax3_ymin, ax3_ymax)
    ax6.set_ylim(ax6_ymin, ax6_ymax)
    
    # update the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
    
def write_data(filename): # for writing the collected data to a csv file
    with open(filename + ' ' + datetime + '.csv','w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
        
        for i in range(len(t_data)):
            info = { # information to write
                    'Time': t_data[i],
                    'Reference X Velocity': xdot_r_data[i],
                    'Reference Y Velocity': ydot_r_data[i],
                    'Reference Angular Velocity': thetadot_r_data[i],
                    'Reference Angle': theta_r_data[i],
                    'Measured Angle': theta_m_data[i]
                    }
            csv_writer.writerow(info)
        
    