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
    help="Axis to plot can be 0 for x, 1, for y, 2 for z and 3 for yawrate")
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'rpyt_based_velocity_controller'),
    delimiter=',', names=True)
# 0 for x, 1 for y  2 for z and 3 for yaw
plot_axis = args.axis
ts = (data['Time'] - data['Time'][0]) / 1e9
error_names = ['Errorx', 'Errory', 'Errorz', 'Erroryawrate']
signal_names = ['Vx', 'Vy', 'Vz', 'Yawrate']
control_names = ['Goalvx', 'Goalvy', 'Goalvz', 'Goalvyawrate']
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(ts, data[error_names[plot_axis]])
plt.ylabel(error_names[plot_axis])
plt.subplot(2, 1, 2)
plt.plot(ts, data[control_names[plot_axis]], 'r')
plt.plot(ts, data[signal_names[plot_axis]], 'b')
plt.legend(['Desired '+signal_names[plot_axis], signal_names[plot_axis]])
plt.ylabel(control_names[plot_axis])
plt.xlabel('Time (seconds)')
plt.show()
