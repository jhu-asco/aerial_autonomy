#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os


parser = argparse.ArgumentParser(
    prog='plot_arm_data')
parser.add_argument('directory', type=str, help='Data directory')
parser.add_argument('--max_t', type=float, default=0.0)
args = parser.parse_args()
arm_cmd_data  = np.genfromtxt(os.path.join(args.directory,
                                           'arm_sine_controller'),
                              delimiter=',', names=True)
arm_sens_data = np.genfromtxt(os.path.join(args.directory,
                                           'arm_sine_controller_connector'),
                              delimiter=',', names=True)
arm_cmd_ts = (arm_cmd_data['Time'] - arm_cmd_data['Time'][0]) / 1e9
if args.max_t == 0:
    max_t = arm_cmd_ts[-1]
    idx2 = (arm_cmd_ts <= max_t)
else:
    max_t = args.max_t
    idx2 = (arm_cmd_ts < args.max_t)
arm_sens_ts = (arm_sens_data['Time'] - arm_cmd_data['Time'][0]) / 1e9
idx = np.logical_and(arm_sens_ts >= 0, arm_sens_ts <= max_t)
arm_sens_ts = arm_sens_ts[idx]

# Ja, Jv
sens_labels = ['Ja_0', 'Ja_1', 'Jv_0', 'Jv_1']
plt.figure(1)
for i, label in enumerate(sens_labels):
    plt.subplot(2, 2, i + 1)
    plt.plot(arm_sens_ts, arm_sens_data[label][idx])
    plt.ylabel(label)
cmd_labels = ['Jad_0', 'Jad_1']
for i, label in enumerate(cmd_labels):
    plt.subplot(2, 2, i + 1)
    plt.plot(arm_cmd_ts, arm_cmd_data[label][idx2])
    plt.legend([sens_labels[i], label])
plt.show()
