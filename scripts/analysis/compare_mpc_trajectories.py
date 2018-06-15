#!/usr/bin/env python3
import matplotlib.pyplot as plt
import argparse
import os
import pandas as pd
import seaborn as sns
import numpy as np
# Args: -f ../../log_mpc_June_13th_2018/data_18_06_12_23_16_26 ../../log_mpc_June_13th_2018/data_18_06_12_22_59_56 -l RNN FF -s ../../
# %% Getting data
parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('-f', '--folders', type=str,nargs='+',
                    help='Data folders to compare')
parser.add_argument('-l', '--legends', type=str, nargs='+',
                    help='Legends for each folder')
parser.add_argument('-s', '--save_folder', type=str, default='./',
                    help='Folder to save final plot')
parser.add_argument('--tStart', type=float, default=0.0, help='Start time')
parser.add_argument('--tEnd', type=float, default=1e3, help='End time')
args = parser.parse_args()
error_df_list = []
assert(len(args.folders) == len(args.legends))

for iFolder, folder in enumerate(args.folders):
    state_data = pd.read_csv(os.path.join(folder, 'mpc_state_estimator'))
    error_data = pd.read_csv(os.path.join(folder, 'ddp_mpc_controller'))
    ts = state_data['#Time'].values
    ts1 = (error_data['#Time'].values - ts[0])/1e9
    ts = (ts - ts[0])/1e9
    iStart = np.argmin(np.abs(ts - args.tStart))
    iEnd = np.argmin(np.abs(ts - args.tEnd))
    interp_error_list = []
    labels = ['Errorx','Errory','Errorz','Errorja1','Errorja2']
    for label in labels:
        interp_error_list.append(np.interp(ts, ts1, error_data[label].values))
    interp_errors = np.vstack(interp_error_list).T
    abs_errors = np.abs(interp_errors)
    folder_label = args.legends[iFolder]
    readable_labels = ['X (m)', 'Y (m)', 'Z (m)', 'J1 (rad)', 'J2 (rad)']
    df = pd.DataFrame(abs_errors, columns=readable_labels)
    df = df.stack().reset_index()
    df.columns = ['Index', 'Sensor Channels', 'Mean Absolute Error']
    df['FolderLabel'] = [folder_label]*df.shape[0]
    error_df_list.append(df)

error_df = pd.concat(error_df_list)
# %% Plotting
sns.set_style('whitegrid')
sns.set(font_scale = 1.2)
plt.figure(1)
sns.barplot('Sensor Channels', 'Mean Absolute Error',
            'FolderLabel', data=error_df, ci=95)
plt.savefig(os.path.join(args.save_folder, 'mpc_error_plot.eps'),
            bbox_inches='tight')