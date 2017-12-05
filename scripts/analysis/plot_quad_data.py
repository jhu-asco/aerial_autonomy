#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'rpyt_relative_pose_visual_servoing_connector'),
    delimiter=',', names=True)
ts = (data['Time'] - data['Time'][0]) / 1e9
print "initial time: ", data['Time'][0]
plt.figure(1)
plt.subplot(3,1,1)
plt.plot(ts, data['vel_x'])
plt.subplot(3,1,2)
plt.plot(ts, data['vel_y'])
plt.subplot(3,1,3)
plt.plot(ts, data['vel_z'])
plt.figure(2)
plt.subplot(3,1,1)
plt.plot(ts, data['roll'])
plt.subplot(3,1,2)
plt.plot(ts, data['pitch'])
plt.subplot(3,1,3)
plt.plot(ts, data['yaw'])
plt.show()
