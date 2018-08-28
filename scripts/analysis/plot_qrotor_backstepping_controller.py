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

controller_data = np.genfromtxt(
        os.path.join(args.directory, 
                     'qrotor_backstepping_controller'), 
                     delimiter=',', names=True)
connector_data = np.genfromtxt(
        os.path.join(args.directory,
                     'qrotor_backstepping_controller_connector'), 
                     delimiter=',', names=True)
speed = np.sqrt(np.square(controller_data['v_x']) + 
    np.square(controller_data['v_y']) + np.square(controller_data['v_z']))
ts = (controller_data['Time'] - controller_data['Time'][0]) / 1e9
ts1 = (connector_data['Time'] - connector_data['Time'][0]) / 1e9

error_names = ['error', 'errory', 'errorz', 'errorvx', 'errorvy', 'errorvz']
error_x = controller_data['p_x'] - controller_data['pd_x']
error_y = controller_data['p_y'] - controller_data['pd_y']
error_z = controller_data['p_z'] - controller_data['pd_z']
error_vx = controller_data['v_x'] - controller_data['vd_x']
error_vy = controller_data['v_y'] - controller_data['vd_y']
error_vz = controller_data['v_z'] - controller_data['vd_z']
plt.figure(1)
plt.subplot(3,1,1)
plt.title('Position profile')
plt.plot(ts, controller_data['p_x'], 'b-', linewidth=3)
plt.plot(ts, controller_data['pd_x'], 'g--', linewidth=5)
plt.plot(ts, error_x, 'r-', linewidth=3)
plt.ylabel('x (m)')
plt.legend(['actual','desired', error_names[0]])
plt.subplot(3,1,2)
plt.plot(ts, controller_data['p_y'], 'b-', linewidth=3)
plt.plot(ts, controller_data['pd_y'], 'g--', linewidth=5)
plt.plot(ts, error_y, 'r-', linewidth=3)
plt.ylabel('y (m)')
plt.subplot(3,1,3)
plt.plot(ts, controller_data['p_z'], 'b-', linewidth=3)
plt.plot(ts, controller_data['pd_z'], 'g--', linewidth=5)
plt.plot(ts, error_z, 'r-', linewidth=3)
plt.xlabel('Time (seconds)')
plt.ylabel('z (m)')
plt.figure(2)
plt.subplot(4,1,1)
plt.title('Velocity profile')
plt.plot(ts, controller_data['v_x'], 'b-', linewidth=3)
plt.plot(ts, controller_data['vd_x'], 'g--', linewidth=5)
plt.plot(ts, error_vx, 'r-', linewidth=3)
plt.ylabel('vx (m/s)')
plt.legend(['actual','desired', error_names[0]])
plt.subplot(4,1,2)
plt.plot(ts, controller_data['v_y'], 'b-', linewidth=3)
plt.plot(ts, controller_data['vd_y'], 'g--', linewidth=5)
plt.plot(ts, error_vy, 'r-', linewidth=3)
plt.ylabel('vy (m/s)')
plt.subplot(4,1,3)
plt.plot(ts, controller_data['v_z'], 'b-', linewidth=3)
plt.plot(ts, controller_data['vd_z'], 'g--', linewidth=5)
plt.plot(ts, error_vz, 'r-', linewidth=3)
plt.ylabel('vz (m/s)')
plt.subplot(4,1,4)
plt.plot(ts, speed, 'b-', linewidth=3)
plt.xlabel('Time (seconds)')
plt.ylabel('v (m/s)')

plt.figure(3)
plt.subplot(3,1,1)
plt.title('RPY profile')
plt.plot(ts1, connector_data['roll']*180/np.pi, 'b-', linewidth=3)
plt.plot(ts1, connector_data['roll_cmd']*180/np.pi, 'g-', linewidth=3)
plt.ylabel('roll (deg)')
plt.legend(['actual','cmd'])
plt.subplot(3,1,2)
plt.plot(ts1, connector_data['pitch']*180/np.pi, 'b-', linewidth=3)
plt.plot(ts1, connector_data['pitch_cmd']*180/np.pi, 'g--', linewidth=5)
plt.ylabel('pitch (deg)')
plt.subplot(3,1,3)
plt.plot(ts1, connector_data['yaw']*180/np.pi, 'b-', linewidth=3)
plt.xlabel('time (seconds)')
plt.ylabel('yaw (deg)')

plt.figure(4)
plt.plot(ts1, connector_data['thrust'], 'b-', linewidth=3)
plt.axhline(y=0.8*9.81*3.4, linewidth=3, color='r', ls = '--')
plt.axhline(y=1.2*9.81*3.4, linewidth=3, color='r', ls = '--')
plt.axhline(y=9.81*3.4, linewidth=2, color='k', ls = '--')
plt.legend(['thrust', 'control bounds'])
plt.title('Thrust profile')
plt.xlabel('time (seconds)')
plt.ylabel('thrust (N)')
plt.show()