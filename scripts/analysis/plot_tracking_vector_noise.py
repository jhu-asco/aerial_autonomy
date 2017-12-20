#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

def rolling_window(a, window):
    shape = a.shape[:-1] + (a.shape[-1] - window + 1, window)
    strides = a.strides + (a.strides[-1],)
    return np.lib.stride_tricks.as_strided(a, shape=shape, strides=strides)

def find_chunks(x, min_gap, min_size):
  starts = np.array([])
  ends = np.array([])
  start = 0
  for i in range(x.size-1):
    if x[i+1] - x[i] > min_gap or i+1 == x.size-1:
      end = i
      if x[end] - x[start] > min_size:
        starts = np.append(starts, start)
        ends = np.append(ends, end)
        start = i+1
  return starts, ends

def get_chunk_means(x, starts, ends):
  means = np.array([])
  for i in range(starts.size):
    means = np.append(means, np.mean(x[int(starts[i]):int(ends[i])]))

  return means


parser = argparse.ArgumentParser(
    prog='plot_tracking_vector_noise')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'rpyt_relative_pose_visual_servoing_connector'),
    delimiter=',', names=True)
measured_pos = np.stack((data['tracking_x'], data['tracking_y'], data['tracking_z']))
viewing_angle = data['Viewing_angle']
tracking_length = data['Tracking_length']
ts = (data['Time'] - data['Time'][0]) / 1e9
pos_names = ['X Noise', 'Y Noise', 'Z Noise']
window_size = 20
noise_thresh = 0.002**2

tracking_length = tracking_length[window_size/2:(1-window_size/2)]
viewing_angle = viewing_angle[window_size/2:(1-window_size/2)]

windows = rolling_window(measured_pos[0, :], window_size)
var = np.var(windows, axis = 1)
ts_window = ts[window_size/2:(1-window_size/2)]
thresh_idx = var < noise_thresh
ts_thresh = ts_window[thresh_idx]
var_thresh = var[thresh_idx]

starts, ends = find_chunks(ts_thresh, 0.5, 5.0) 

var_all = np.array([]).reshape(0, starts.size)
for plot_axis in range(3):
  windows = rolling_window(measured_pos[plot_axis, :], window_size)
  var = np.var(windows, axis = 1)
  var_thresh = var[thresh_idx]
  var_i = get_chunk_means(var_thresh, starts, ends)
  var_all = np.append(var_all, var_i.reshape(1, starts.size), axis=0)

  plt.figure(plot_axis+1)
  plt.plot(ts_window, np.sqrt(var))
  plt.ylabel(pos_names[plot_axis])
  plt.xlabel('Time (seconds)')

viewing_angles = get_chunk_means(viewing_angle[thresh_idx], starts, ends)
tracking_lengths = get_chunk_means(tracking_length[thresh_idx], starts, ends)

plt.figure(4)
plt.plot(ts[window_size/2:(1-window_size/2)], tracking_length)
plt.ylabel('Distance (m)')
plt.xlabel('Time (seconds)')
plt.figure(5)
plt.plot(ts[window_size/2:(1-window_size/2)], viewing_angle)
plt.ylabel('Viewing Angle (rad)')
plt.xlabel('Time (seconds)')

plt.figure(6)
for plot_axis in range(3):
  plt.plot(viewing_angles, np.sqrt(var_all[plot_axis, :]), 'o')
plt.ylabel('Noise (m)')
plt.xlabel('Viewing Angle (rad)')
plt.legend(['X', 'Y', 'Z'])
plt.figure(7)
for plot_axis in range(3):
  plt.plot(tracking_lengths, np.sqrt(var_all[plot_axis, :]), 'o')
plt.ylabel('Noise (m)')
plt.xlabel('Distance (m)')
plt.legend(['X', 'Y', 'Z'])

plt.show()
