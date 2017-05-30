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

## Parameters
t_plot = 1000 # time step until which we plot

## File reading
traj_path = "data/state.dat"
traj_data = np.loadtxt(traj_path,dtype=float)
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

## Thermals plot
nbth = len(th_data["x"])
th_x_dat = th_data["x"].tolist()
th_y_dat = th_data["y"].tolist()
th_wstar_dat = th_data["w_star"].tolist()
for i in range(0,nbth):
    x_th=linspace(th_x_dat[i],th_x_dat[i],1000)
    y_th=linspace(th_y_dat[i],th_y_dat[i],1000)
    z_th=linspace(0,1000,1000)
    ax.plot(x_th,y_th,z_th,color='#ff6600')

## Border plot
limit_circle = Circle((0, 0), 1200, color='black', fill=False)
ax.add_patch(limit_circle)
art3d.pathpatch_2d_to_3d(limit_circle, z=0, zdir="z")

plt.show()

