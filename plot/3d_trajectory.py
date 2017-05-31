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
traj_path = "data/state.dat"
traj_data = np.loadtxt(traj_path,dtype=float)
#t_plot = 1000 				 # time step until which we plot
t_plot = traj_data[:,10][-1] # time step until which we plot
traj_data = traj_data[traj_data[:,10] <= t_plot]

th_data_path = "config/thermal_scenario.csv"
th_data = pd.read_csv(th_data_path,sep = ';')
th_data = th_data[th_data["t_birth"]<=t_plot]
th_data = th_data[th_data["t_birth"]+th_data["lifespan"]>=t_plot]

## Trajectory plot
x = traj_data[:,0]
y = traj_data[:,1]
z = traj_data[:,2]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x,y,z,color='#6699ff')
ax.set_xlim3d((-1300, 1300))
ax.set_ylim3d((-1300, 1300))
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.scatter(x[0], y[0], z[0], color='black', marker="x") # starting point
ax.scatter(x[-1],y[-1],z[-1],color='black', marker="x") # ending point
ax.text(x[0]+10,  y[0]+10,  z[0]+10,  "Start")
ax.text(x[-1]+10, y[-1]+10, z[-1]+10, "End")


## Thermals plot
th_x_dat = th_data["x"].tolist()
th_y_dat = th_data["y"].tolist()
height = th_data["w_star"].tolist()
for i in range(0,len(th_x_dat)):
	height[i] = int(200.*height[i])
	ax.plot([th_x_dat[i],th_x_dat[i]], [th_y_dat[i],th_y_dat[i]],zs=[0,height[i]],color='#ff6600')

## Border plot
limit_circle = Circle((0, 0), 1200, color='black', fill=False)
ax.add_patch(limit_circle)
art3d.pathpatch_2d_to_3d(limit_circle, z=0, zdir="z")

plt.show()

