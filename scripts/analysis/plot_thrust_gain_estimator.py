#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_thrust_gain_estimator')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'thrust_gain_estimator'),
    delimiter=',', names=True)
ts = (data['Time'] - data['Time'][0]) / 1e9
gravity_component = 9.81 * np.cos(data['roll']) * np.cos(data['pitch'])
scaled_thrust = data['thrust_command'] * data['thrust_gain']
error_thrust_command = data['body_z_acc'] - \
    scaled_thrust + gravity_component
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(ts, data['roll'])
plt.ylabel('roll')
plt.subplot(2, 1, 2)
plt.plot(ts, data['pitch'])
plt.ylabel('pitch')
plt.xlabel('Time(sec)')
plt.figure(2)
ax = plt.gca()
ax2 = ax.twinx()
ax.plot(ts, data['body_z_acc'])
ax.set_ylabel('Body z acc (m/ss)')
ax2.plot(ts, data['thrust_command'], 'r')
ax2.set_ylabel('Thrust Command')
plt.xlabel('Time (sec)')
ax2.legend(('Thrust command',), loc=2)
ax.legend(('Body z acc',))
plt.figure(3)
plt.plot(ts, data['thrust_gain'])
plt.ylabel('Estimated Thrust gain')
plt.xlabel('Time (sec)')
plt.figure(4)
plt.plot(ts, error_thrust_command)
plt.ylabel('Body_Z_Acc_diff')
plt.xlabel('Time (sec)')
plt.figure(5)
plt.plot(ts, scaled_thrust)
plt.xlabel('Time (sec)')
plt.ylabel('Scaled thrust in g')
plt.show()
