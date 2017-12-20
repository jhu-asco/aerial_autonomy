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
        'rpyt_based_velocity_controller'),
    delimiter=',', names=True)
world_acc = np.stack((data['World_acc_x'], data['World_acc_y'], data['World_acc_z']-9.81))
world_acc_norm = np.linalg.norm(world_acc, axis=0)
# 0 for x, 1 for y  2 for z and 3 for yaw
ts = (data['Time'] - data['Time'][0]) / 1e9
error_names = ['Errorx', 'Errory', 'Errorz', 'Erroryawrate']
signal_names = ['Vx', 'Vy', 'Vz', 'Yawrate']
control_names = ['Goalvx', 'Goalvy', 'Goalvz', 'Goalvyawrate']
for plot_axis in range(4):
  plt.figure(plot_axis+1)
  plt.subplot(2, 1, 1)
  plt.plot(ts, data[error_names[plot_axis]])
  plt.ylabel(error_names[plot_axis])
  plt.subplot(2, 1, 2)
  plt.plot(ts, data[control_names[plot_axis]], 'r')
  plt.plot(ts, data[signal_names[plot_axis]], 'b')
  plt.legend(['Desired '+signal_names[plot_axis], signal_names[plot_axis]])
  plt.ylabel(control_names[plot_axis])
  plt.xlabel('Time (seconds)')
plt.figure(5)
plt.plot(ts, world_acc_norm)
plt.xlabel('Time (seconds)')
plt.ylabel('World acceleration Norm(m/ss)')
plt.show()
