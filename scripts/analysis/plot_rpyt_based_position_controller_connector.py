#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_rpyt_based_position_controller_connector')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'rpyt_based_position_controller_drone_connector'),
    delimiter=',', names=True)
ts = (data['Time'] - data['Time'][0]) / 1e9
position_names = ['x', 'y', 'z']
goal_names = ['goal_x', 'goal_y', 'goal_z']
vel_names = ['vx', 'vy', 'vz']
acc_names = ['ax', 'ay', 'az']
rpy_names = ['r', 'p', 'y']
cmd_names = ['r_cmd', 'p_cmd', 't_cmd']
bias_names = ['r_bias', 'p_bias']
plt.figure(1)
for plot_axis in range(3):
  plt.subplot(3, 1, plot_axis+1)
  plt.plot(ts, data[position_names[plot_axis]])
  plt.plot(ts, data[goal_names[plot_axis]])
  plt.ylabel(position_names[plot_axis])
  plt.xlabel('Time (seconds)')
  plt.legend(['Actual', 'Goal'])

plt.figure(2)
for plot_axis in range(2):
  plt.subplot(2, 1, plot_axis+1)
  plt.plot(ts, data[rpy_names[plot_axis]])
  plt.plot(ts, data[cmd_names[plot_axis]])
  plt.ylabel(rpy_names[plot_axis])
  plt.xlabel('Time (seconds)')
  plt.legend(['Actual', 'Command'])

plt.figure(3)
for plot_axis in range(2):
  plt.subplot(2, 1, plot_axis+1)
  plt.plot(ts, data[bias_names[plot_axis]])
  plt.ylabel(bias_names[plot_axis])
  plt.xlabel('Time (seconds)')

plt.figure(4)
for plot_axis in range(3):
  plt.subplot(3, 1, plot_axis+1)
  plt.plot(ts, data[acc_names[plot_axis]])
  plt.ylabel(acc_names[plot_axis])
  plt.xlabel('Time (seconds)')

plt.figure(5)
plt.plot(ts, data['t_cmd'])
plt.ylabel('Thrust')
plt.xlabel('Time (seconds)')

plt.show()
