#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_velocity_based_position_controller')
parser.add_argument('directory', type=str, help='Data directory')
parser.add_argument(
    '-a', '--axis',
    type=int,
    default=0,
    help="Axis to plot can be 0 for x, 1, for y, 2 for z and 3 for yaw")
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'velocity_based_position_controller'),
    delimiter=',', names=True)
# 0 for x, 1 for y  2 for z and 3 for yaw
plot_axis = args.axis
ts = (data['Time'] - data['Time'][0]) / 1e9
error_names = ['x_diff', 'y_diff', 'z_diff', 'yaw_diff']
cumulative_error_names = ['xi_diff', 'yi_diff', 'zi_diff', 'yawi_diff']
control_names = ['cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_yawrate']
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(ts, data[error_names[plot_axis]])
plt.ylabel(error_names[plot_axis])
plt.subplot(2, 1, 2)
plt.plot(ts, data[cumulative_error_names[plot_axis]])
plt.ylabel(cumulative_error_names[plot_axis])
plt.xlabel('Time (seconds)')
plt.show()
plt.figure(2)
plt.plot(ts, data[control_names[plot_axis]])
plt.ylabel(control_names[plot_axis])
plt.xlabel('Time (seconds)')
