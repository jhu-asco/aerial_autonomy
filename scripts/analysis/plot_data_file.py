#!/usr/bin/env python2
import matplotlib.pyplot as plt
import argparse
import os
import pandas as pd
# %%
parser = argparse.ArgumentParser(
    prog='plot_quad_data')
parser.add_argument('file', type=str, help='Data file')
args = parser.parse_args()
data = pd.read_csv(os.path.join(
        args.file))
ts = data['#Time'].values
ts = (ts - ts[0]) / 1e9
print "initial time: ", data['#Time'][0]
plt.figure(1)
plt.clf()
for i, column in enumerate(data.columns):
    if i == 0:
        continue
    data_array = data[column].values
    f = plt.subplot(3, 8, i)
    plt.plot(ts, data_array)
    if i <=  16:
        f.axes.get_xaxis().set_ticks([])
    plt.ylabel(column)