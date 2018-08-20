#!/usr/bin/env python2
import matplotlib.pyplot as plt
import argparse
import os
import pandas as pd
import numpy as np
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
dt = np.diff(ts)
rdot = np.diff(data["r"])/dt
pdot = np.diff(data["p"])/dt
ydot = np.diff(data["y.1"])/dt

rdot_filt = np.empty_like(rdot)
rdot_filt[0] = rdot[0]
k = 0.9
N = len(rdot_filt)
for i in range(N-1):
    rdot_filt[i+1] = k*rdot_filt[i] + (1-k)*rdot[i]

plt.figure(1)
plt.clf()
plt.plot(ts[:-1], rdot, 'b')
plt.plot(ts[:-1], rdot_filt, 'r')
plt.legend(['roll_dot', 'roll_filt_dot'])
plt.show(block=True)
