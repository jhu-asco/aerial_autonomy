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
args = parser.parse_args()
mocap_data = np.genfromtxt(os.path.join(args.directory, 'mocap_logger'),
                           delimiter=',', names=True)
rpyt_data = np.genfromtxt(os.path.join(args.directory,
                                       'manual_rpyt_controller'),
                          delimiter=',', names=True)
mocap_ts = (mocap_data['Time'] - rpyt_data['Time'][0]) / 1e9
idx = mocap_ts > 0
mocap_ts = mocap_ts[idx]
mocap_rpy = quat2RPY(mocap_data['Qx'], mocap_data['Qy'], mocap_data['Qz'],
                     mocap_data['Qw'])
rpyt_ts = (rpyt_data['Time'] - rpyt_data['Time'][0]) / 1e9
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
    plt.plot(rpyt_ts, rpyt_data[rpy_labels[i] + '_cmd'])
    plt.legend(['Measured', 'Commanded'])
    plt.ylabel(label)
    plt.xlabel('Time(seconds)')

plt.figure(2)
plt.plot(rpyt_ts, rpyt_data['Thrust_cmd'])
plt.ylabel('Thrust cmd')
plt.xlabel('Time(seconds)')
plt.show()
