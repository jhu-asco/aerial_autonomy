#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import pandas as pd
import seaborn as sns
import scipy.signal as signal
import tf.transformations as tf


def transformGlobalAcc(yaw_arr, acc_arr):
    """
    Transform global acc to gravity compensated body frame
    """
    rotated_acc = np.empty_like(acc_arr)
    for i, yaw in enumerate(yaw_arr):
        R = tf.euler_matrix(yaw, 0, 0, 'rzyx')
        rotated_acc[i] = np.dot(R[:3, :3].T, acc_arr[i])
    return rotated_acc

def transformBodyAcc(rp_arr, body_acc_arr):
    """
    Transform body acc to gravity compensated body frame
    """
    rotated_acc = np.empty_like(body_acc_arr)
    for i, rp in enumerate(rp_arr):
        R = tf.euler_matrix(0, rp[1], rp[0], 'rzyx')
        rotated_acc[i] = np.dot(R[:3, :3], body_acc_arr[i])
    return rotated_acc

def getAcceleration(ts, vel_arr, low_pass_freq=1):
    dt = np.diff(ts)
    acc_unfiltered = np.diff(vel_arr.T)/dt
    nyquist_freq = 0.5/np.mean(dt)
    frac = low_pass_freq/nyquist_freq
    b, a = signal.butter(8, frac)
    acc_filtered = signal.filtfilt(b, a, acc_unfiltered)
    return acc_filtered.T

def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

def getRPAcc(acc_arr):
    acc_copy = np.copy(acc_arr)
    acc_copy[:,2] = acc_copy[:,2] + 9.81
    acc_normalized = normalized(acc_copy)
    roll = np.arcsin(-acc_normalized[:, 1])
    pitch = np.empty_like(roll)
    idx = np.logical_and(np.abs(acc_normalized[:, 0]) < 1e-4,
                         np.abs(acc_normalized[:,2]) < 1e-4)
    pitch[idx] = 0
    idx_neg = np.logical_not(idx)
    cos_inv_roll = 1.0/np.cos(roll[idx_neg])
    pitch[idx_neg] = np.arctan2(acc_normalized[idx_neg,0]*cos_inv_roll,
                                acc_normalized[idx_neg,2]*cos_inv_roll)
    return np.vstack((roll, pitch)).T

    
parser = argparse.ArgumentParser(
    prog='plot_rpyt_reference_controller')
parser.add_argument('folder', type=str, help='Data folder')
parser.add_argument('--tStart', type=float, default=0.0, help='Start time')
parser.add_argument('--tEnd', type=float, default=1e3, help='End time')
args = parser.parse_args()
connector_data = pd.read_csv(os.path.join(args.folder, 'rpyt_reference_connector'))
ts = (connector_data['#Time'] - connector_data['#Time'][0])/1e9
ctrlr_data = pd.read_csv(os.path.join(args.folder, 'rpyt_reference_controller'))
ts_ctrlr= (ctrlr_data['#Time'] - connector_data['#Time'][0])/1e9
iStart = np.argmin(np.abs(ts - args.tStart))
iEnd = np.argmin(np.abs(ts - args.tEnd))
# Get acc from mocap
vel_xyz = connector_data[['vx','vy','vz']].values
yaw = connector_data['Sensor_yaw'].values
if 'accx' in connector_data.columns:
    body_acc_imu = connector_data[['accx', 'accy', 'accz']].values
else:
    estimator_data = pd.read_csv(os.path.join(args.folder, 'thrust_gain_estimator'))
    ts_estimator= (estimator_data['#Time'] - connector_data['#Time'][0])/1e9
    body_acc_imu = np.empty((ts.size, 3))
    body_acc_imu_original = estimator_data[['body_x_acc', 'body_y_acc', 'body_z_acc']].values
    for i in range(3):
        body_acc_imu[:,i] = np.interp(ts, ts_estimator, body_acc_imu_original[:,i])
rp_imu = connector_data[['roll','pitch']].values
global_acc = getAcceleration(ts[iStart:iEnd], vel_xyz[iStart:iEnd, :])
rotated_acc = transformGlobalAcc(yaw[iStart:(iEnd-1)], global_acc)
rotated_acc_imu = transformBodyAcc(rp_imu, body_acc_imu)
ts1 = ts[iStart:(iEnd-1)]
for i, label in enumerate(['ax', 'ay', 'az']):
    plt.figure()
    plt.plot(ts1, rotated_acc[:, i])
    plt.plot(ts[iStart:iEnd], rotated_acc_imu[iStart:iEnd,i])
    plt.legend(('Mocap','IMU'))
    plt.ylabel(label)
    plt.xlabel('Time (seconds)')

# %% Thrust gain analysis
gravity_neg = np.array([0,0,9.81])
mocap_acc_norm = np.linalg.norm(rotated_acc + gravity_neg, axis=1)
imu_acc_norm = np.linalg.norm(rotated_acc_imu+gravity_neg, axis=1)
imu_z_gravity_comp = body_acc_imu[:,2] + 9.81*np.cos(rp_imu[:,0])*np.cos(rp_imu[:,1])
cmd_thrust = np.interp(ts, ts_ctrlr, ctrlr_data['Cmd_thrust'])
scale_mocap = mocap_acc_norm/cmd_thrust[iStart:(iEnd-1)]
scale_imu = imu_acc_norm/cmd_thrust
scale_imu_z = imu_z_gravity_comp/cmd_thrust
plt.figure()
plt.plot(ts1, scale_mocap)
plt.plot(ts[iStart:iEnd], scale_imu[iStart:iEnd])
plt.plot(ts[iStart:iEnd], scale_imu_z[iStart:iEnd])
plt.legend(['Mocap', 'IMU','IMU_Z'])
plt.ylabel('Thrust gain')
plt.xlabel('Time (seconds)')
#%% RP analysis
rp_acc_mocap = getRPAcc(rotated_acc)
rp_acc_imu = getRPAcc(rotated_acc_imu)
rp_sensor = connector_data[['Sensor_roll', 'Sensor_pitch']].values
rd = np.interp(ts, ts_ctrlr, ctrlr_data['Cmd_roll'])
pd = np.interp(ts, ts_ctrlr, ctrlr_data['Cmd_pitch'])
rp_d = np.vstack((rd, pd)).T

sns.set(font_scale=2.0, style="white")

for i, label in enumerate(['Roll(rad)', 'Pitch (rad)']):
    plt.figure()
    #plt.plot(ts1, rp_acc_mocap[:,i])
    #plt.plot(ts[iStart:iEnd], rp_sensor[iStart:iEnd, i])
    plt.plot(ts[iStart:iEnd], rp_acc_imu[iStart:iEnd,i])
    plt.plot(ts[iStart:iEnd], rp_imu[iStart:iEnd,i])
    #plt.plot(ts[iStart:iEnd], rp_d[iStart:iEnd,i])
    #plt.legend(['Mocap_Acc', 'Acc_IMU', 'Fused_Quad', 'Command'])
    plt.legend(['Acc_IMU', 'IMU'])
    plt.ylabel(label)
    plt.xlabel('Time (seconds)')
    plt.grid(True)
    plt.savefig(label+'.eps', bbox_inches='tight', dpi=300)
    mocap_diff = rp_acc_mocap[:,i] - rp_imu[iStart:(iEnd-1),i]
    imu_diff = rp_acc_imu[iStart:iEnd,i] - rp_imu[iStart:iEnd,i]
    print label+':'
    print "Mean diff mocap", np.mean(mocap_diff)
    print "Std diff mocap", np.std(mocap_diff)
    print "Mean diff IMU", np.mean(imu_diff)
    print "Std diff IMU", np.std(imu_diff)
plt.figure()
plt.plot(ts[iStart:iEnd], yaw[iStart:iEnd])
plt.ylabel('yaw (rad)')
plt.xlabel('Time (seconds)')
plt.show()
