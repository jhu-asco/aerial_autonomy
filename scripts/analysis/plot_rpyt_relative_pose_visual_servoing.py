#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_rpyt_relative_pose_visual_servoing')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'rpyt_relative_pose_visual_servoing_connector'),
    delimiter=',', names=True)

ts = (data['Time'] - data['Time'][0]) / 1e9

ylim = 180.
ylim_pos = 5.

plt.figure(1)
plt.subplot(3, 1, 1)
plt.plot(ts, data['tracking_x'])
plt.ylim([-ylim_pos, ylim_pos])
plt.subplot(3, 1, 2)
plt.plot(ts, data['tracking_y'])
plt.ylim([-ylim_pos, ylim_pos])
plt.subplot(3, 1, 3)
plt.plot(ts, data['tracking_z'])
plt.ylim([-ylim_pos, ylim_pos])
plt.xlabel('Time (s)')
plt.suptitle('Tracking Position (m)')

plt.figure(2)
plt.subplot(3, 1, 1)
plt.plot(ts, data['tracking_r'] * 180./np.pi)
plt.ylim([-ylim, ylim])
plt.subplot(3, 1, 2)
plt.plot(ts, data['tracking_p'] * 180./np.pi)
plt.ylim([-ylim, ylim])
plt.subplot(3, 1, 3)
plt.plot(ts, data['tracking_y'] * 180./np.pi)
plt.ylim([-ylim, ylim])
plt.xlabel('Time (s)')
plt.suptitle('Tracking Angle (rad)')

plt.figure(3)
plt.subplot(3, 1, 1)
plt.plot(ts, data['object_x'])
plt.ylim([-ylim_pos, ylim_pos])
plt.subplot(3, 1, 2)
plt.plot(ts, data['object_y'])
plt.ylim([-ylim_pos, ylim_pos])
plt.subplot(3, 1, 3)
plt.plot(ts, data['object_z'])
plt.ylim([-ylim_pos, ylim_pos])
plt.xlabel('Time (s)')
plt.suptitle('Object in Cam Position (m)')

plt.figure(4)
plt.subplot(3, 1, 1)
plt.plot(ts, data['object_r'] * 180./np.pi)
plt.ylim([-ylim, ylim])
plt.subplot(3, 1, 2)
plt.plot(ts, data['object_p'] * 180./np.pi)
plt.ylim([-ylim, ylim])
plt.subplot(3, 1, 3)
plt.plot(ts, data['object_y'] * 180./np.pi)
plt.ylim([-ylim, ylim])
plt.xlabel('Time (s)')
plt.suptitle('Object in Cam Angle (rad)')
plt.show()
