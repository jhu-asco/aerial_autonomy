#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_velocity_based_position_controller')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'velocity_based_position_controller'),
    delimiter=',', names=True)
# 0 for x, 1 for y  2 for z and 3 for yaw
ts = (data['Time'] - data['Time'][0]) / 1e9
error_names = ['x_diff', 'y_diff', 'z_diff', 'yaw_diff']
goal_names = ['x_goal', 'y_goal', 'z_goal', 'yaw_goal']
state_goal_names = ['x', 'y', 'z', 'yaw']
cumulative_error_names = ['xi_diff', 'yi_diff', 'zi_diff', 'yawi_diff']
control_names = ['cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_yawrate']
for plot_axis in range(4):
  plt.figure(plot_axis+1)
  plt.subplot(2, 2, 1)
  plt.plot(ts, data[error_names[plot_axis]])
  plt.ylabel(error_names[plot_axis])
  plt.subplot(2, 2, 2)
  plt.plot(ts, data[cumulative_error_names[plot_axis]])
  plt.ylabel(cumulative_error_names[plot_axis])
  plt.xlabel('Time (seconds)')
  plt.subplot(2, 2, 3)
  plt.plot(ts, data[control_names[plot_axis]])
  plt.ylabel(control_names[plot_axis])
  plt.xlabel('Time (seconds)')
  plt.subplot(2, 2, 4)
  plt.plot(ts, -data[goal_names[plot_axis]] + data[error_names[plot_axis]])
  plt.plot(ts, data[goal_names[plot_axis]])
  plt.legend(['position', 'goal'])
  plt.ylabel(state_goal_names[plot_axis])
  plt.xlabel('Time (seconds)')
plt.show()
