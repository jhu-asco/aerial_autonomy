#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os
import tf.transformations as tf


def quat2RPY(qx, qy, qz, qw):
    N = len(qx)
    rpy_out = np.empty((N, 3))
    for i in range(N):
        ypr = tf.euler_from_quaternion(
            np.array([qx[i], qy[i], qz[i], qw[i]]), 'rzyx')
        rpy_out[i, 0] = ypr[2]
        rpy_out[i, 1] = ypr[1]
        rpy_out[i, 2] = ypr[0]
    return rpy_out


parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('directory', type=str, help='Data directory')
parser.add_argument('--write_data', dest='write_data',
                    action='store_true', default=False)
parser.add_argument('--sensor_delay', type=float, default=0.15)
parser.add_argument('--max_t', type=float, default=0.0)
args = parser.parse_args()
mocap_data = np.genfromtxt(os.path.join(args.directory, 'mocap_logger'),
                           delimiter=',', names=True)
rpyt_data = np.genfromtxt(os.path.join(args.directory,
                                       'manual_rpyt_controller'),
                          delimiter=',', names=True)
rpyt_ts = (rpyt_data['Time'] - rpyt_data['Time'][0]) / 1e9
if args.max_t == 0:
    max_t = rpyt_ts[-1]
    idx2 = (rpyt_ts <= max_t)
else:
    max_t = args.max_t
    idx2 = (rpyt_ts < args.max_t)
mocap_ts = (mocap_data['Time'] - rpyt_data['Time'][0]) / 1e9
mocap_ts = mocap_ts - args.sensor_delay
idx = np.logical_and(mocap_ts >= 0, mocap_ts <= max_t)
mocap_ts = mocap_ts[idx]
mocap_rpy = quat2RPY(mocap_data['Qx'], mocap_data['Qy'], mocap_data['Qz'],
                     mocap_data['Qw'])
# Write data to file
if args.write_data:
    data_dir = './nn_data/'
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    in_dir_tup = args.directory.split('/')
    in_dir_name = in_dir_tup[-1]
    if not in_dir_name:
        in_dir_name = in_dir_tup[-2]
    timestamp = in_dir_name[4:]
    out_file = os.path.join(data_dir, 'nn_data' + timestamp)
    mocap_xyzrpy = np.vstack((mocap_ts, mocap_data['X'][idx],
                              mocap_data['Y'][idx], mocap_data['Z'][idx],
                              mocap_rpy[idx, :].T)).T
    cmd_data = np.stack((rpyt_ts[idx2], rpyt_data['Roll_cmd'][idx2],
                         rpyt_data['Pitch_cmd'][idx2],
                         rpyt_data['Yaw_cmd'][idx2],
                         rpyt_data['Thrust_cmd'][idx2]), axis=1)
    np.savez(out_file, control_data=[cmd_data], sensor_data=[mocap_xyzrpy])
labels = ['X', 'Y', 'Z']
plt.figure(1)
for i, label in enumerate(labels):
    plt.subplot(2, 3, i + 1)
    plt.plot(mocap_ts, mocap_data[label][idx])
    plt.ylabel(label)
rpy_labels = ['Roll', 'Pitch', 'Yaw']
for i, label in enumerate(rpy_labels):
    plt.subplot(2, 3, i + 4)
    plt.plot(mocap_ts, mocap_rpy[idx, i])
    plt.plot(rpyt_ts[idx2], rpyt_data[rpy_labels[i] + '_cmd'][idx2])
    plt.legend(['Measured', 'Commanded'])
    plt.ylabel(label)
    plt.xlabel('Time(seconds)')

plt.figure(2)
plt.plot(rpyt_ts[idx2], rpyt_data['Thrust_cmd'][idx2])
plt.ylabel('Thrust cmd')
plt.xlabel('Time(seconds)')
plt.show()
