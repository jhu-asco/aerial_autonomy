#!/usr/bin/env python
"""
Created on Sun Sep  9 18:04:20 2018
Compare MPC Errors with RPYT ref Errors
@author: gowtham
"""
import argparse
import pandas as pd
import seaborn as sns
import os
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('log_folder', type=str, help='Log folder')
args = parser.parse_args()

mpc_folders = ['data_18_09_07_21_02_26', 'data_18_09_07_21_12_58']

mpc_errors = []
for folder in mpc_folders:
    mpc_errors.append(pd.read_csv(os.path.join(args.log_folder,
                                               folder, 'errors.csv')))

cumulative_mpc_df = pd.concat(mpc_errors, ignore_index=True)
# %%
cumulative_mpc_df = cumulative_mpc_df.drop(columns=['Unnamed: 0'])
cumulative_mpc_unstack_df = cumulative_mpc_df.unstack()
cumulative_mpc_unstack_df = cumulative_mpc_unstack_df.reset_index()
cumulative_mpc_unstack_df = cumulative_mpc_unstack_df.drop(columns=['level_1'])
cumulative_mpc_unstack_df.columns = ['Axis', 'Error']
cumulative_mpc_unstack_df['Error'] = cumulative_mpc_unstack_df['Error'].abs()
cumulative_mpc_unstack_df['Controller'] = 'MPC'
#cumulative_mpc_df['Controller'] = 'MPC'

cumulative_rpyt_df = pd.read_csv(os.path.join(args.log_folder,
                                              'cumulative_error.csv'))
cumulative_rpyt_df = cumulative_rpyt_df.drop(columns=['Unnamed: 0', 'Time'])
cum_rpyt_unstack_df = cumulative_rpyt_df.unstack()
cum_rpyt_unstack_df = cum_rpyt_unstack_df.reset_index()
cum_rpyt_unstack_df = cum_rpyt_unstack_df.drop(columns=['level_1'])
cum_rpyt_unstack_df.columns = ['Axis', 'Error']
cum_rpyt_unstack_df['Error'] = cum_rpyt_unstack_df['Error'].abs()
cum_rpyt_unstack_df['Controller'] = 'Acceleration control'

combined_errors = pd.concat((cumulative_mpc_unstack_df,
                             cum_rpyt_unstack_df), ignore_index=True)
# %%
sns.set()
plt.figure()
ax = sns.barplot(data=combined_errors, x='Axis', y='Error', hue='Controller')
ax.yaxis.grid(True)
plt.savefig('mean_absolute_error_compare.eps', bbox_inches='tight', dpi=300)
plt.savefig('mean_absolute_error_compare.png', bbox_inches='tight', dpi=300)
plt.show()



