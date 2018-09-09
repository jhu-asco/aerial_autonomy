#!/usr/bin/env python
import pandas as pd
import seaborn as sns
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os

parser = argparse.ArgumentParser(prog='pick_place_rms_error')
parser.add_argument('meta_file', type=str,
                    help='File containing log information')
parser.add_argument('--log_folder', type=str, help='Folder containing logs')
parser.add_argument('--cumulative_df', type=str, help='Load previous dataset',
                    default=None)
parser.add_argument('--prefix', type=str,
                    help='Prefix for logs in the meta file')
args = parser.parse_args()
error_labels = ['Errorx','Errory','Errorz', 'Erroryaw', 'Errorvx', 'Errorvy',
                    'Errorvz']
meta_df = pd.read_csv(args.meta_file)
if args.cumulative_df is None:
    list_dfs = []
    current_df = None
    error_df = None
    ts = None
    # Procedure:
    # Go through the meta file rows and
    for i, log_file in enumerate(meta_df['Data']):
        if isinstance(log_file, str):
            current_df = pd.read_csv(os.path.join(args.log_folder,
                                                  args.prefix + log_file,
                                                  'rpyt_reference_controller'))
            connector_df = pd.read_csv(os.path.join(args.log_folder,
                                                    args.prefix + log_file,
                                                    'rpyt_reference_connector'))
            ts = (connector_df['#Time'] - connector_df['#Time'][0])/1e9
            ts1 = (current_df['#Time'] - current_df['#Time'][0]) / 1e9
            interp_error_list = [ts]
            for label in error_labels:
                interp_error_list.append(np.interp(ts, ts1,
                                                   current_df[label].values))
            error_df = pd.DataFrame(np.vstack(interp_error_list).T,
                                    columns=['Time']+error_labels)
        if current_df is None:
            print("No data frame to evaluate")
            continue
        tStart = meta_df['Start time'].iloc[i]
        #tEnd = meta_df['Place End time'].iloc[i]
        #if tEnd < 0:
        tEnd = meta_df['Pick End time'].iloc[i]
        iStart = np.argmin(np.abs(ts - tStart))
        iEnd = np.argmin(np.abs(ts - tEnd))
        list_dfs.append(error_df.iloc[iStart:iEnd])
    # Finally bar plot errors using seaborn :)
    cumulative_error_df = pd.concat(list_dfs, ignore_index=True)
    cumulative_error_df.to_csv(os.path.join(args.log_folder,
                                            'cumulative_error.csv'))
else:
    cumulative_error_df = pd.read_csv(args.cumulative_df)
# Plot:
ax = sns.barplot(data=cumulative_error_df[error_labels].abs(), ci=2)
ax.yaxis.grid(which="major")
plt.savefig('mean_absolute_errors.eps', bbox_inches='tight', dpi=300)
plt.figure()
total_time = meta_df['Total time']
total_time[total_time > 0].hist()
plt.xlabel('Total time (seconds)')
plt.ylabel('Count')
plt.savefig('total_times_histogram.eps',bbox_inches='tight',dpi=300)
plt.figure()
meta_df['Pickup times'].hist()
plt.xlabel('Pickup time (seconds)')
plt.ylabel('Count')
plt.savefig('pickup_times_histogram.eps', bbox_inches='tight', dpi=300)
plt.show()

