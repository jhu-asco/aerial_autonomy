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
arm_cmd_data  = np.genfromtxt(os.path.join(args.directory,
                                           'arm_sine_controller'),
                              delimiter=',', names=True)
arm_sens_data = np.genfromtxt(os.path.join(args.directory,
                                           'arm_sine_controller_connector'),
                              delimiter=',', names=True)
t0 = rpyt_data['Time'][0]
rpyt_ts = (rpyt_data['Time'] - t0) / 1e9
arm_cmd_ts = (arm_cmd_data['Time'] - t0) / 1e9
if args.max_t == 0:
    max_t = rpyt_ts[-1]
    rpyt_cmd_idx = (rpyt_ts <= max_t)
    arm_cmd_idx = (arm_cmd_ts <= max_t)
else:
    max_t = args.max_t
    rpyt_cmd_idx = (rpyt_ts < args.max_t)
    arm_cmd_idx = (arm_cmd_ts <= args.max_t)
mocap_ts = (mocap_data['Time'] - t0) / 1e9
mocap_ts = mocap_ts - args.sensor_delay
m_idx = np.logical_and(mocap_ts >= 0, mocap_ts <= max_t)
mocap_ts = mocap_ts[m_idx]
mocap_rpy = quat2RPY(mocap_data['Qx'], mocap_data['Qy'], mocap_data['Qz'],
                     mocap_data['Qw'])
# Currently no delay in arm sensor data
arm_sens_ts = (arm_sens_data['Time'] - t0) / 1e9
a_idx = np.logical_and(arm_sens_ts >= 0, arm_sens_ts <= max_t)
arm_sens_ts = arm_sens_ts[a_idx]
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
    mocap_xyzrpy = np.vstack((mocap_ts, mocap_data['X'][m_idx],
                              mocap_data['Y'][m_idx], mocap_data['Z'][m_idx],
                              mocap_rpy[m_idx, :].T)).T
    arm_sens_out = np.vstack((arm_sens_ts, arm_sens_data['Ja_0'][a_idx],
                              arm_sens_data['Ja_1'][a_idx],
                              arm_sens_data['Jv_0'][a_idx],
                              arm_sens_data['Jv_1'][a_idx])).T
    c_idx = rpyt_cmd_idx
    rpyt_cmd_data = np.stack((rpyt_ts[c_idx], rpyt_data['Roll_cmd'][c_idx],
                              rpyt_data['Pitch_cmd'][c_idx],
                              rpyt_data['Yaw_cmd'][c_idx],
                              rpyt_data['Thrust_cmd'][c_idx]), axis=1)
    c_idx = arm_cmd_idx
    arm_cmd_out = np.stack((arm_cmd_ts[c_idx], arm_cmd_data['Jad_0'][c_idx],
                            arm_cmd_data['Jad_1'][c_idx]), axis=1)
    np.savez(out_file, control_data=[rpyt_cmd_data, arm_cmd_out], sensor_data=[mocap_xyzrpy, arm_sens_out])
labels = ['X', 'Y', 'Z']
plt.figure(1)
for i, label in enumerate(labels):
    plt.subplot(2, 3, i + 1)
    plt.plot(mocap_ts, mocap_data[label][m_idx])
    plt.ylabel(label)
rpy_labels = ['Roll', 'Pitch', 'Yaw']
for i, label in enumerate(rpy_labels):
    plt.subplot(2, 3, i + 4)
    plt.plot(mocap_ts, mocap_rpy[m_idx, i])
    plt.plot(rpyt_ts[rpyt_cmd_idx], rpyt_data[rpy_labels[i] + '_cmd'][rpyt_cmd_idx])
    plt.legend(['Measured', 'Commanded'])
    plt.ylabel(label)
    plt.xlabel('Time(seconds)')

plt.figure(2)
plt.plot(rpyt_ts[rpyt_cmd_idx], rpyt_data['Thrust_cmd'][rpyt_cmd_idx])
plt.ylabel('Thrust cmd')
plt.xlabel('Time(seconds)')
# Ja, Jv
arm_sens_labels = ['Ja_0', 'Ja_1', 'Jv_0', 'Jv_1']
plt.figure(3)
for i, label in enumerate(arm_sens_labels):
    plt.subplot(2, 2, i + 1)
    plt.plot(arm_sens_ts, arm_sens_data[label][a_idx])
    plt.ylabel(label)
arm_cmd_labels = ['Jad_0', 'Jad_1']
for i, label in enumerate(arm_cmd_labels):
    plt.subplot(2, 2, i + 1)
    plt.plot(arm_cmd_ts[arm_cmd_idx], arm_cmd_data[label][arm_cmd_idx])
    plt.legend([arm_sens_labels[i], label])

plt.show()
