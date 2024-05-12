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
              'Measured X Velocity',
              'Measured Y Velocity',
              'Measured Angular Velocity',
              'Measured Angle',
              'Measured X',
              'Measured Y'] # column names for csv output file

window = 120 # number of points to keep on the x-axis, such that the live plotting x window moves in time

num_plots = 6
fig = plt.figure(figsize=(20,8),) # create figure for plotting
ax1 = fig.add_subplot(2,3,1) # subplot 1, x velocity
ax2 = fig.add_subplot(2,3,2) # subplot 2, y velocity
ax3 = fig.add_subplot(2,3,3) # subplot 3, angular velocity
ax4 = fig.add_subplot(2,3,4) # subplot 4, x position
ax5 = fig.add_subplot(2,3,5) # subplot 5, y position
ax6 = fig.add_subplot(2,3,6) # subplot 6, angular position (only needed for the switched velocity/position system)

# set up lines to plot, will update the contents of the plot in the animate function
xdot_r, = ax1.plot(0,0,label="Reference Values") # requested x velocity
xdot_m, = ax1.plot(0,0,label="Measured Values") # measured x velocity

ydot_r, = ax2.plot(0,0,label="Reference Values") # requested y velocity
ydot_m, = ax2.plot(0,0,label="Measured Values") # measured y velocity

thetadot_r, = ax3.plot(0,0,label="Reference Values") # requested angular velocity
thetadot_m, = ax3.plot(0,0,label="Measured Values") # measured angular velocity

x, = ax4.plot(0,0,label="Measured Values")
y, = ax5.plot(0,0,label="Measured Values")

theta_r, = ax6.plot(0,0,label="Reference Values") # requested angle
theta_m, = ax6.plot(0,0,label="Measured Values") # measured angle


# Add chart labels and legends
fig.suptitle("Measured vs Reference Values", fontsize=15)

ax1.set_title("X Velocity")
ax2.set_title("Y Velocity")
ax3.set_title("Angular Velocity")
ax4.set_title("X Position")
ax5.set_title("Y Position")
ax6.set_title("Angular Position")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("X Velocity (in/s)")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Y Velocity (in/s)")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Angular Velocity (rad/s)")
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("X Position (in)")
ax5.set_xlabel("Time (s)")
ax5.set_ylabel("Y Position (in)")
ax6.set_xlabel("Time (s)")
ax6.set_ylabel("Angular Position (rad)")

ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()
ax5.legend()
ax6.legend()

fig.tight_layout(pad=2)

# define empty lists to store the data to be plotted and saved
xdot_r_data = []
ydot_r_data = []
thetadot_r_data = []
theta_r_data = []

xdot_m_data = []
ydot_m_data = []
thetadot_m_data = []
theta_m_data = []

x_data = []
y_data = []
t_data = [] # time data

def animate(reference, measured, start_time): # reference and measured lists are in the order [xdot, ydot, thetadot, theta]
    t_data.append(time.time() - start_time) # calculate the current program time vector
    # append input values to their data storage lists    
    xdot_r_data.append(reference[0])
    ydot_r_data.append(reference[1])
    thetadot_r_data.append(reference[2])
    theta_r_data.append(reference[3])
    
    xdot_m_data.append(measured[0])
    ydot_m_data.append(measured[1])
    thetadot_m_data.append(measured[2])
    x_data.append(measured[3])
    y_data.append(measured[4])
    theta_m_data.append(measured[5])
    
    
    # set the y data to be plotted
    xdot_r.set_ydata(xdot_r_data[-window:]) # only plot the last {window} data points
    ydot_r.set_ydata(ydot_r_data[-window:])
    thetadot_r.set_ydata(thetadot_r_data[-window:])
    theta_r.set_ydata(theta_r_data[-window:])
    
    xdot_m.set_ydata(xdot_m_data[-window:])
    ydot_m.set_ydata(ydot_m_data[-window:])
    thetadot_m.set_ydata(thetadot_m_data[-window:])
    theta_m.set_ydata(theta_m_data[-window:])
    x.set_ydata(x_data[-window:])
    y.set_ydata(y_data[-window:])
    
    # set the x data to be plotted
    for item in [xdot_r, ydot_r, thetadot_r, theta_r, xdot_m, ydot_m, thetadot_m, theta_m, x, y]:
        item.set_xdata(t_data[-window:])
        
    # calculate the window range of each chart in each dimension
    tmin = min(t_data[-window:]) # all charts have the same x limits in time
    tmax = max(t_data[-window:])
    
    ax1_ymin = min(min(xdot_r_data[-window:]), min(xdot_m_data[-window:]))
    ax2_ymin = min(min(ydot_r_data[-window:]), min(ydot_m_data[-window:]))
    ax3_ymin = min(min(thetadot_r_data[-window:]), min(thetadot_m_data[-window:]))
    ax4_ymin = min(x_data[-window:])
    ax5_ymin = min(y_data[-window:])
    ax6_ymin = min(min(theta_r_data[-window:]), min(theta_m_data[-window:]))
    
    ax1_ymax = max(max(xdot_r_data[-window:]), max(xdot_m_data[-window:]))
    ax2_ymax = max(max(ydot_r_data[-window:]), max(ydot_m_data[-window:]))
    ax3_ymax = max(max(thetadot_r_data[-window:]), max(thetadot_m_data[-window:]))
    ax4_ymax = max(x_data[-window:])
    ax5_ymax = max(y_data[-window:])
    ax6_ymax = max(max(theta_r_data[-window:]), max(theta_m_data[-window:]))
    
    # apply calculated window ranges
    for plot in [ax1, ax2, ax3, ax4, ax5, ax6]:
        plot.set_xlim(tmin, tmax)
    
    ax1.set_ylim(ax1_ymin, ax1_ymax)
    ax2.set_ylim(ax2_ymin, ax2_ymax)
    ax3.set_ylim(ax3_ymin, ax3_ymax)
    ax4.set_ylim(ax4_ymin, ax4_ymax)
    ax5.set_ylim(ax5_ymin, ax5_ymax)
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
                    'Measured X Velocity': xdot_m_data[i],
                    'Measured Y Velocity': ydot_m_data[i],
                    'Measured Angular Velocity': thetadot_m_data[i],
                    'Measured Angle': theta_m_data[i],
                    'Measured X': x_data[i],
                    'Measured Y': y_data[i]
                    }
            csv_writer.writerow(info)
        
    