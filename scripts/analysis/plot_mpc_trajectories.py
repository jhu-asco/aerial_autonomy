#!/usr/bin/env python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
import pandas as pd
import seaborn as sns
import numpy as np
# %% Getting data
parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('folder', type=str, help='Data folder')
parser.add_argument('--tStart', type=float, default=0.0, help='Start time')
parser.add_argument('--tEnd', type=float, default=1e3, help='End time')
parser.add_argument('--prefix', type=str, default='airm', help='quad or airm')
args = parser.parse_args()
state_data = pd.read_csv(os.path.join(args.folder, args.prefix+'_mpc_state_estimator'))
error_data = pd.read_csv(os.path.join(args.folder, 'ddp_'+args.prefix+'_mpc_controller'))
ts = state_data['#Time'].values
ts1 = (error_data['#Time'].values - ts[0])/1e9
ts = (ts - ts[0])/1e9
iStart = np.argmin(np.abs(ts - args.tStart))
iEnd = np.argmin(np.abs(ts - args.tEnd))
interp_error_list = []
error_labels = ['Errorx','Errory','Errorz']
if args.prefix == 'airm':
    error_labels = error_labels + ['Errorja1','Errorja2']
for label in error_labels:
    interp_error_list.append(np.interp(ts, ts1, error_data[label].values))
interp_errors = np.vstack(interp_error_list).T
if args.prefix == 'airm':
    states = state_data[['x','y','z','ja1','ja2']].values
else:
    states = state_data[['x','y','z']].values
ref_states = states - interp_errors
# %% Plotting
sns.set_style('whitegrid')
sns.set(font_scale = 1.2)
plt.figure(1)
labels = ['$p_x$', '$p_y$', '$p_z$', '$r_1$', '$r_2$']
units = ['m','m','m','rad', 'rad']
legend = ['Tracked', 'Reference']
ts_sub = ts[iStart:iEnd]
ncols = states.shape[1]
for i in range(ncols):
    plt.figure(i+1)
    plt.plot(ts_sub, states[iStart:iEnd, i])
    plt.plot(ts_sub, ref_states[iStart:iEnd, i])
    plt.ylabel(labels[i]+' ('+units[i]+')')
    plt.xlabel('Time (seconds)')
    plt.legend(legend)
    plt.tight_layout()
    plt.savefig(os.path.join(args.folder,labels[i]+'.eps'),
                             bbox_inches='tight')

fig = plt.figure(6)
ax = fig.add_subplot(111, projection='3d')
ax.plot(states[iStart:iEnd,0], states[iStart:iEnd,1], states[iStart:iEnd,2])
ax.plot(ref_states[iStart:iEnd,0], ref_states[iStart:iEnd,1], ref_states[iStart:iEnd,2])
ax.legend(legend)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
rms_errors = np.sqrt(np.mean(np.square(interp_errors[iStart:iEnd, :]), axis=0))
np.set_printoptions(precision=2, suppress=True)
header = 'RMSX, RMSY, RMSZ'
if args.prefix=='airm':
    header = header + ', RMSJ1, RMSJ2'
np.savetxt(os.path.join(args.folder, 'rms_errors.csv'),
           rms_errors[:,np.newaxis].T, fmt='%.2f',
           delimiter=',',
           header=header)
print("RMS ERRORS: ", rms_errors)
# %%For 3d plot save it yourself
plt.savefig(os.path.join(args.folder,'trajectory.eps'),
            bbox_inches='tight')
plt.show()
