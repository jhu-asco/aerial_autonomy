#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

parser = argparse.ArgumentParser(
    prog='plot_qrotor_backstepping_controller')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
data = np.genfromtxt(
    os.path.join(
        args.directory,
        'qrotor_backstepping_controller'),
    delimiter=',', names=True)
ts = (data['Time'] - data['Time'][0]) / 1e9
error_names = ['Errorx', 'Errory', 'Errorz', 'Errorvx', 'Errorvy', 'Errorvz']
error_x = data['p_x'] - data['pd_x']
error_y = data['p_y'] - data['pd_y']
error_z = data['p_z'] - data['pd_z']
error_vx = data['v_x'] - data['vd_x']
error_vy = data['v_y'] - data['vd_y']
error_vz = data['v_z'] - data['vd_z']
plt.figure(1)
plt.subplot(3,1,1)
plt.plot(ts, data['p_x'])
plt.plot(ts, data['pd_x'])
plt.plot(ts, error_x)
plt.ylabel('x')
plt.legend(['position x','Desired', error_names[0]])
plt.subplot(3,1,2)
plt.plot(ts, data['p_y'])
plt.plot(ts, data['pd_y'])
plt.plot(ts, error_y)
plt.ylabel('y')
plt.legend(['position y','Desired', error_names[1]])
plt.subplot(3,1,3)
plt.plot(ts, data['p_z'])
plt.plot(ts, data['pd_z'])
plt.plot(ts, error_z)
plt.xlabel('Time (seconds)')
plt.ylabel('z')
plt.legend(['position z','Desired', error_names[2]])
plt.figure(2)
plt.subplot(3,1,1)
plt.plot(ts, data['v_x'])
plt.plot(ts, data['vd_x'])
plt.plot(ts, error_vx)
plt.ylabel('vx')
plt.legend(['vel x','Desired', error_names[3]])
plt.subplot(3,1,2)
plt.plot(ts, data['v_y'])
plt.plot(ts, data['vd_y'])
plt.plot(ts, error_vy)
plt.ylabel('vy')
plt.legend(['vel y','Desired', error_names[4]])
plt.subplot(3,1,3)
plt.plot(ts, data['v_z'])
plt.plot(ts, data['vd_z'])
plt.plot(ts, error_vz)
plt.xlabel('Time (seconds)')
plt.ylabel('vz')
plt.legend(['vel z','Desired', error_names[5]])
plt.show()