#!/usr/bin/env python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
import pandas as pd
import seaborn as sns
import numpy as np
from set_axes_equal import set_axes_equal
# %% Getting data
parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('folder', type=str, help='Data folder')
parser.add_argument('--tStart', type=float, default=0.0, help='Start time')
parser.add_argument('--tEnd', type=float, default=1e3, help='End time')
parser.add_argument('--prefix', type=str, default='airm', help='quad or airm')
parser.add_argument('--delay', type=int, default=0, help='number of places to right shift rpy_d')
parser.add_argument('--kexp', type=float, default=0.99, help='Gain on previous bias for filtering')
args = parser.parse_args()
state_data = pd.read_csv(os.path.join(args.folder, args.prefix+'_mpc_state_estimator'))
ts = state_data['#Time'].values
ts = (ts - ts[0])/1e9
iStart = np.argmin(np.abs(ts - args.tStart))
iEnd = np.argmin(np.abs(ts - args.tEnd))
if args.prefix == 'airm':
    states = state_data[['x','y','z','ja1','ja2']].values
else:
    states = state_data[['x','y','z']].values
# rpy and rpy_desired
rpy = state_data[['r','p','y.1']].values
rpy_d = state_data[['rd','pd','yd']].values
rpy_d = np.roll(rpy_d, args.delay, axis=0)
rpy_d[:args.delay, :] = 0
rpy_d[:,2] = rpy_d[:,2] + rpy[0,2]-rpy_d[0,2]
# Bias
delta_rp = rpy[iStart:iEnd, :2] - rpy_d[iStart:iEnd, :2]
bias_array = np.empty_like(delta_rp)
bias = np.array([0, 0])
for i, error_rp in enumerate(delta_rp):
    bias = bias*args.kexp + (1-args.kexp)*error_rp
    bias_array[i] = bias
# %% Plotting
sns.set_style('whitegrid')
sns.set(font_scale = 1.2)
labels = ['roll', 'pitch', 'yaw']
ts_sub = ts[iStart:iEnd]
legend = ['Tracked', 'Reference']
for i in range(3):
    plt.figure(1+i)
    ref_angle = rpy_d[iStart:iEnd, i]
    plt.plot(ts_sub, rpy[iStart:iEnd, i])
    plt.plot(ts_sub, rpy_d[iStart:iEnd, i])
    print("Mean diff ",labels[i],': ', np.mean(rpy[iStart:iEnd, i] - rpy_d[iStart:iEnd, i]))
    print("Std diff: ",labels[i],': ', np.std(rpy[iStart:iEnd, i] - rpy_d[iStart:iEnd, i]))
    plt.xlabel('Time (seconds)')
    plt.ylabel(labels[i]+' (rad)')
    if i < 2:
        plt.plot(ts_sub, bias_array[iStart:iEnd, i])
        plt.legend(legend+['Bias'])
    else:
        plt.legend(legend)
    plt.tight_layout()
    plt.savefig(os.path.join(args.folder,labels[i]+'.eps'),
                             bbox_inches='tight')
# Plot delta_pitch vs pitch:
delta_p = rpy[iStart:iEnd, 1] - rpy_d[iStart:iEnd, 1]
plt.figure(4)
plt.plot(rpy_d[iStart:iEnd, 1], delta_p, 'b*')
plt.xlabel('Pitch command (rad)')
plt.ylabel('Error pitch (rad)')
plt.savefig(os.path.join(args.folder,'delta_pitch.eps'),
                         bbox_inches='tight')
plt.show()
