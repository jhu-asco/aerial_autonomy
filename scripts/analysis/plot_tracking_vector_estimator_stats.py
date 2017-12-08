#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_tracking_vector_estimator')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'tracking_vector_estimator'),
    delimiter=',', names=True)
state_labels = ['Marker_x', 'Marker_y', 'Marker_z']
noise_labels = ['Noise_x', 'Noise_y', 'Noise_z']
meas_labels = ['Measured_Marker_x', 'Measured_Marker_y', 'Measured_Marker_y']
meas_noise_labels = ['Meas_noise_x', 'Meas_noise_y', 'Meas_noise_z']
ts = (data['Time'] - data['Time'][0]) / 1e9
plt.figure(1)
for i in range(3):
  plt.subplot(2, 3, i+1)
  plt.errorbar(ts, data[meas_labels[i]], yerr=data[meas_noise_labels[i]])
  plt.errorbar(ts, data[state_labels[i]], yerr=data[noise_labels[i]])
  plt.ylabel(state_labels[i])
  plt.xlabel('Time (seconds)')
  plt.legend([meas_labels[i], state_labels[i]])
plt.show()
