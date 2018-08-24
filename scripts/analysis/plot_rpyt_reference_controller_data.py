#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os
import pandas as pd
from set_axes_equal import set_axes_equal
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser(
    prog='plot_rpyt_reference_controller')
parser.add_argument('folder', type=str, help='Data folder')
parser.add_argument('--tStart', type=float, default=0.0, help='Start time')
parser.add_argument('--tEnd', type=float, default=1e3, help='End time')
args = parser.parse_args()
connector_data = pd.read_csv(os.path.join(args.folder, 'rpyt_reference_connector'))
ctrlr_data = pd.read_csv(os.path.join(args.folder, 'rpyt_reference_controller'))
ts = (connector_data['#Time'] - connector_data['#Time'][0])/1e9
ts1 = (ctrlr_data['#Time'] - connector_data['#Time'][0]) / 1e9
iStart = np.argmin(np.abs(ts - args.tStart))
iEnd = np.argmin(np.abs(ts - args.tEnd))

interp_error_list = []
error_labels = ['Errorx','Errory','Errorz', 'Erroryaw', 'Errorvx', 'Errorvy', 'Errorvz']
for label in error_labels:
    interp_error_list.append(np.interp(ts, ts1, ctrlr_data[label].values))
interp_errors = np.vstack(interp_error_list).T
states = connector_data[['x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz']].values
ref_states = states + interp_errors

labels = ['$p_x$', '$p_y$', '$p_z$', 'yaw', '$v_x$', '$v_y$', '$v_z$']
units = ['m','m','m','rad','m/s', 'm/s', 'm/s']
legend = ['Tracked', 'Reference']
ts_sub = ts[iStart:iEnd]
ncols = states.shape[1]
for i in range(ncols):
    plt.figure()
    plt.plot(ts_sub, states[iStart:iEnd, i])
    plt.plot(ts_sub, ref_states[iStart:iEnd, i])
    plt.ylabel(labels[i]+' ('+units[i]+')')
    plt.xlabel('Time (seconds)')
    plt.legend(legend)
    plt.tight_layout()
    simplified_label = labels[i].replace('$','')+'.eps'
    plt.savefig(os.path.join(args.folder,simplified_label),
                             bbox_inches='tight')
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(states[iStart:iEnd,0], states[iStart:iEnd,1], states[iStart:iEnd,2])
ax.plot(ref_states[iStart:iEnd,0], ref_states[iStart:iEnd,1], ref_states[iStart:iEnd,2])
ax.legend(legend)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
set_axes_equal(ax)
rms_errors = np.sqrt(np.mean(np.square(interp_errors[iStart:iEnd, :]), axis=0))
np.set_printoptions(precision=2, suppress=True)
header = 'RMSX, RMSY, RMSZ, RMSYaw'
np.savetxt(os.path.join(args.folder, 'rms_errors.csv'),
           rms_errors[:,np.newaxis].T, fmt='%.2f',
           delimiter=',',
           header=header)
print("RMS ERRORS: ", rms_errors)
# Plot rpy and rpy_desired
labels = ['roll', 'pitch']
rpy = connector_data[labels].values
rpy_d = ctrlr_data[['Cmd_roll','Cmd_pitch']].values
if 'Sensor_roll' in connector_data.columns:
  sensor_rpy_labels = ['Sensor_roll', 'Sensor_pitch', 'Sensor_yaw']
  sensor_rpy = connector_data[sensor_rpy_labels].values
for i in range(2):
    plt.figure()
    plt.plot(ts_sub, rpy[iStart:iEnd, i])
    plt.plot(ts_sub, rpy_d[iStart:iEnd, i])
    if 'Sensor_roll' in connector_data.columns:
      plt.plot(ts_sub, sensor_rpy[iStart:iEnd, i])
    print("Mean diff ",labels[i],': ', np.mean(rpy[iStart:iEnd, i] - rpy_d[iStart:iEnd, i]))
    print("Std diff: ",labels[i],': ', np.std(rpy[iStart:iEnd, i] - rpy_d[iStart:iEnd, i]))
    plt.xlabel('Time (seconds)')
    plt.ylabel(labels[i]+' (rad)')
plt.figure()
plt.plot(ts_sub, connector_data['Thrust_gain'][iStart:iEnd])
plt.xlabel('Time (seconds)')
plt.ylabel('Thrust gain')
if 'accx' in connector_data.columns:
  plt.figure()
  acc_labels = ['accx', 'accy', 'accz']
  acc = connector_data[acc_labels].values
  for i in range(3):
    plt.subplot(3,1, i+1)
    plt.plot(ts_sub, acc[iStart:iEnd, i])
    plt.ylabel(acc_labels[i])
  plt.xlabel('Time (seconds)')
plt.show()
