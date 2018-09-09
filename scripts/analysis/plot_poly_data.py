#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser(
    prog='plot_poly_data')
parser.add_argument('file', type=str, help='Data file')
args = parser.parse_args()

df = pd.read_csv(args.file)
f = plt.figure()
ax =  f.add_subplot(111, projection='3d')
ax.plot(df['x'],df['y'], zs=df['z'])

plt.figure()
plt.plot(df['#Time'], df['x'])
plt.plot(df['#Time'], df['y'])
plt.plot(df['#Time'], df['z'])

plt.figure()
plt.plot(df['#Time'], df['vx'])
plt.plot(df['#Time'], df['vy'])
plt.plot(df['#Time'], df['vz'])

plt.figure()
plt.plot(df['#Time'], df['roll'])
plt.plot(df['#Time'], df['pitch'])

plt.show()
