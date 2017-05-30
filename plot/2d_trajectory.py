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
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(x,y,color='#6699ff')
ax.set_xlim((-1300, 1300))
ax.set_ylim((-1300, 1300))
ax.set_xlabel('x')
ax.set_ylabel('y')

x_end = x[-1]
y_end = y[-1]
ax.scatter(x_end,y_end,color='black',marker="x")

## Thermals plot
nbth = len(th_data["x"])
th_x_dat = th_data["x"].tolist()
th_y_dat = th_data["y"].tolist()
for i in range(0,nbth):
    x_th = th_x_dat[i]
    y_th = th_y_dat[i]
    ax.scatter(x_th,y_th,color='#ff6600',marker="x")

## Border plot
limit_circle = Circle((0, 0), 1200, color='black', fill=False)
ax.add_patch(limit_circle)

plt.show()

