#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 15 19:12:36 2017

Find average error after starting from max error of 0.1

@author: gowtham
"""

import glob
import numpy as np

folders = ['data_17_09_14_21_16_37', 'data_17_09_14_22_05_44', 'data_17_09_14_22_12_01', 'data_17_09_14_22_41_57', 'data_17_09_14_22_48_18/', 'data_17_09_15_02_36_00/', 'data_17_09_15_03_10_31/', 'data_17_09_15_03_29_23/']
rms = lambda error_array: np.sqrt(np.mean(np.square(np.squeeze(error_array))))
tolerance = 0.2
for axis in ['x_diff', 'y_diff', 'z_diff']:
    error_data = []
    for folder in folders:
        for csv_file in glob.glob(folder+'/split_data/*.csv'):    
            data = np.genfromtxt(csv_file, delimiter=',', names=True)
            diff = data[axis]
            indices = np.argwhere(diff > -tolerance)
            if len(indices)  == 0:
                print "Something wrong here"
            error_data.append(diff[indices[0, 0]:])
    error_data_array = np.hstack(error_data)
    print "RMS error for {0}: {1} ".format(axis,  rms(error_data_array))

time_taken = []
for folder in folders:
    for csv_file in glob.glob(folder + '/split_data/*.csv'):
        data = np.genfromtxt(csv_file, delimiter=',', names=True)
        time_taken.append((data['Time'][-1] - data['Time'][0])/1e9)

time_taken_array = np.array(time_taken)
mean_time_taken = np.mean(time_taken_array)
tdiff_mean = time_taken_array - mean_time_taken
print ("Mean time taken: {0}, RMS Time taken: {1}, "
       "Min time taken: {2}, Max time taken: {3}").format(mean_time_taken,
                        rms(tdiff_mean), np.min(time_taken_array), np.max(time_taken_array))
        