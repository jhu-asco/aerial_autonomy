import numpy as np
import matplotlib.pyplot as plt
import sys

if sys.argc < 2:
    print "Usage ./plot_velocity_based_position_controller.py [File_name]"
    sys.exit(-1)

data = np.genfromtxt(sys.argv[-1], delimiter=',')
# 0 for x, 1 for y  2 for z and 3 for yaw
plot_axis = 3;
ts = (data[:,0] - data[0, 0])/1e9
plt.figure(1+plot_axis)
plt.subplot(2,1,1)
plt.plot(ts, data[:, 1+plot_axis]);
plt.ylabel('Error')
plt.subplot(2,1,2)
plt.plot(ts, data[:, 5+plot_axis]);
plt.ylabel('Cumulative Error')
plt.xlabel('Time (seconds)')
plt.show()
plt.figure(2+plot_axis)
plt.plot(ts, data[:, 9+plot_axis])
plt.ylabel('Control')
plt.xlabel('Time (seconds)')
