import numpy as np
import matplotlib.pyplot as plt

def f1():
    
    laser_range = np.loadtxt('lidar.txt')

    laser_range[laser_range==0] = np.nan
    lr2i = np.nanargmax(laser_range)
    print('Picked direction: %d %f m' % (lr2i, laser_range[lr2i]))
    print('lr2i = ', lr2i)
    print('laser_range[lr2i] = ', laser_range[lr2i])

    plt.figure()
    plt.polar(np.arange(0,360,360/len(laser_range))/180*np.pi,laser_range)
    plt.show()

def f2():
    quat = np.loadtxt('odom.txt', skiprows=13, delimiter=':', usecols=1, max_rows=4)
    print (quat)

def f3():
    omap = np.loadtxt('map.txt')
    plt.imshow(omap, origin='lower')
    plt.show()

if __name__ == "__main__":
    f1()


