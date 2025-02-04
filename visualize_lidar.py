import numpy as np
import matplotlib.pyplot as plt
laser_range = np.loadtxt('lidar.txt')
# plt.figure()
# plt.polar(np.arange(0,360,360/len(laser_range))/180*np.pi,laser_range)
# plt.show()
# print("done")

# Replace zero values with NaN
laser_range[laser_range == 0] = np.nan  

# Find the index of the maximum (non-NaN) value
lr2i = np.nanargmax(laser_range)  

# Print the index and its corresponding value
print("Index of max range:", lr2i)
print("Max range value:", laser_range[lr2i])

# Plot the data
plt.figure()
plt.polar(np.arange(0, 360, 360/len(laser_range))/180*np.pi, laser_range)
plt.show()

# show odom data
quat = np.loadtxt('odom.txt', skiprows=13, delimiter=':',usecols=1, max_rows=4)
print (quat)

# show map data
omap = np.loadtxt('map.txt')
plt.imshow(omap, origin='lower')
plt.show()