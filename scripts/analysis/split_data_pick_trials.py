#!/usr/bin/env python2
import argparse
import os
import numpy as np

parser = argparse.ArgumentParser(
    prog='plot_velocity_based_position_controller')
parser.add_argument('directory', type=str, help='Data directory')
parser.add_argument('--x_error_cutoff', type=float, default=-0.8, help='Min x error')
args = parser.parse_args()


data_folder = os.path.join(os.curdir, args.directory)
split_folder = os.path.join(data_folder, 'split_data')
if not os.path.exists(split_folder):
    os.makedirs(split_folder)
file_name = os.path.join(data_folder,
                                  'velocity_based_position_controller')
data = np.genfromtxt(
                     file_name,
                     delimiter=',', names=True)
data_normal = np.genfromtxt(
                     os.path.join(data_folder,
                                  'velocity_based_position_controller'),
                     delimiter=',')
ts = (data['Time'] - data['Time'][0])/1e9

N = len(data['x_diff'])
inds = data['x_diff'] < args.x_error_cutoff
inds_values = np.arange(N)[inds]
idiff = inds_values[1:] - inds_values[:-1]
idiff_values = np.argwhere(idiff > 1)
splits = inds_values[idiff_values + 1]
start_indices = np.insert(np.squeeze(splits), 0, 0)
x_diff = data['x_diff']
end_indices = []
start_indices_temp = np.append(start_indices, -1)
for i in range(len(start_indices_temp)-1):
    max_index = start_indices_temp[i] + np.argmax(x_diff[start_indices_temp[i]:start_indices_temp[i+1]])
    end_index = max_index
    start_index = start_indices_temp[i]
    if x_diff[max_index] > 0.5:
        while end_index > start_index:
            if x_diff[end_index] > 0:
                end_index = end_index - 1
            else:
                break
    end_indices.append(end_index)
f = file(file_name, 'r')
header = f.readline()[1:-1]
f.close()
print "Number of segments: ", len(start_indices)
print "HEader: ", header
for i in range(len(start_indices)):
    data_split = data_normal[start_indices[i]:end_indices[i], :]
    print "Tstart: {0}, Tdiff: {1}, Xdiff_end: {2}".format(ts[start_indices[i]], ts[end_indices[i]] - ts[start_indices[i]], x_diff[end_indices[i]])
    file_name = 'split'+str(i)+'.csv'
    np.savetxt(os.path.join(split_folder, file_name), data_split, header=header, delimiter=',')