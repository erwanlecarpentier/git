
from __future__ import division
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
from mpl_toolkits.mplot3d import Axes3D
from scipy import *
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d

## File reading
trajectory_path = "data/state.dat"
trajectory_data = np.loadtxt(trajectory_path,dtype=float)
thermal_data_path = "data/config1.csv"
thermal_data_buffer = pd.read_csv(thermal_data_path,sep = ' ')
thermal_data = thermal_data_buffer[thermal_data_buffer["tBirth"]==0]

## Trajectory plot
x = trajectory_data[:,0]
y = trajectory_data[:,1]
z = trajectory_data[:,2]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x,y,z,color='#6699ff')
ax.set_xlim3d((-1300, 1300))
ax.set_ylim3d((-1300, 1300))
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

## Thermals center plot at t=0
size = len(thermal_data["CentreX"])
for i in range(0,size):
    xth=linspace(thermal_data["CentreX"][i],thermal_data["CentreX"][i],1000)
    yth=linspace(thermal_data["CentreY"][i],thermal_data["CentreY"][i],1000)
    zth=linspace(0,1000,1000)
    ax.plot(xth,yth,zth,color='#ff6600')

## Border plot
limit_circle = Circle((0, 0), 1200, color='black', fill=False)
ax.add_patch(limit_circle)
art3d.pathpatch_2d_to_3d(limit_circle, z=0, zdir="z")

plt.show()

