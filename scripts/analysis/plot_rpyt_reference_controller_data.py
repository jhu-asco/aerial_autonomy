#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os
import pandas as pd

parser = argparse.ArgumentParser(
    prog='plot_rpyt_reference_controller')
parser.add_argument('directory', type=str, help='Data directory')
args = parser.parse_args()
ctrlr_data = pd.read_csv(os.path.join(args.directory, 'rpyt_reference_controller'))
#connector_data = pd.read_csv(os.path.join(args.directory, 'rpyt_reference_connector'))
# 0 for x, 1 for y  2 for z and 3 for yaw
ts = (ctrlr_data['#Time'] - ctrlr_data['#Time'][0]) / 1e9
#ts1 = (connector_data['#Time'] - ctrlr_data['#Time'][0])/1e9

error_names = ['Errorx', 'Errory', 'Errorz', 'Erroryaw', 'Errorvx', 'Errorvy', 'Errorvz']
for i, error_name in enumerate(error_names):
    plt.figure(i+1)
    plt.plot(ts, ctrlr_data[error_name])
    plt.ylabel(error_name)
    plt.xlabel('Time (seconds)')

plt.show()
