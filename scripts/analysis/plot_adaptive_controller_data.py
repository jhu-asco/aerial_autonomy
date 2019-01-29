#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_adaptiver_controller_data')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'adaptive_controller'),
    delimiter=',', names=True)
# 0 for x, 1 for y  2 for z and 3 for yaw
ts = (data['Time'] - data['Time'][0]) / 1e9
error_names = ['delta_px', 'delta_py', 'delta_pz']
goal_names = ['x_goal', 'y_goal', 'z_goal']
state_names = ['x', 'y', 'z']
acc_names = ['acc_x','acc_y','acc_z']
for plot_axis in range(3):
  plt.figure(plot_axis+1)
  plt.subplot(2, 1, 1)
  plt.plot(ts, data[error_names[plot_axis]])
  plt.ylabel(error_names[plot_axis])
  plt.subplot(2, 1, 2)
  plt.plot(ts, data[goal_names[plot_axis]])
  plt.plot(ts, data[state_names[plot_axis]])
  plt.legend(['goal', 'position'])
  plt.xlabel('Time (seconds)')

plt.figure()
for ii in range(3):
  plt.subplot(3,1,ii+1)
  plt.plot(ts, data[acc_names[ii]])
  plt.xlabel('Time (seconds)')
  plt.ylabel(acc_names[ii])

plt.figure()
plt.subplot(2,1,1)
plt.plot(ts,data['roll'])
plt.xlabel('Time')
plt.ylabel('Roll')
plt.subplot(2,1,2)
plt.plot(ts,data['pitch'])
plt.xlabel('Time')
plt.ylabel('Pitch')

plt.figure()
plt.plot(ts, data['mhat'])
plt.xlabel('Time (sec)')
plt.ylabel('Mhat')
plt.show()
