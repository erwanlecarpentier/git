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

########## FILE READING ##########

## State file reading
traj_path = "data/state.dat"
traj_data = np.loadtxt(traj_path,dtype=float)
#t_plot = 1000 				 # time step until which we plot
t_plot = traj_data[:,10][-1] # time step until which we plot
traj_data = traj_data[traj_data[:,10] <= t_plot]

## Thermal file reading
th_data_path = "config/fz_scenario.csv"
th_data = pd.read_csv(th_data_path,sep = ';')
th_data = th_data[th_data["t_birth"]<=t_plot]
th_data = th_data[th_data["t_birth"]+th_data["lifespan"]>=t_plot]

## Flight zone config file reading
fz_cfg_path = "config/fz_config.csv"
fz_cfg = pd.read_csv(fz_cfg_path,sep = ';')

##################################

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111)

## Plot boundaries
#limit_circle = Circle((0, 0), 1200, color='black', fill=False)
#ax.add_patch(limit_circle)
x_min = fz_cfg["x_min"][0]
x_max = fz_cfg["x_max"][0]
y_min = fz_cfg["x_min"][0]
y_max = fz_cfg["x_max"][0]
limit_rectangle = Rectangle((x_min,y_min),x_max-x_min,y_max-y_min,fill=False)
ax.add_patch(limit_rectangle)

## Define borders and label
off = 200
ax.set_xlim((x_min-off,x_max+off))
ax.set_ylim((y_min-off,y_max+off))
ax.set_xlabel('x')
ax.set_ylabel('y')

## Plot trajectory
x = traj_data[:,0]
y = traj_data[:,1]
ax.plot(x,y,color='#6699ff')
ax.scatter(x[0], y[0], color='black', marker="x") # starting point
ax.scatter(x[-1],y[-1],color='black', marker="x") # ending point
ax.text(x[0]+10,  y[0]+10,  "Start")
ax.text(x[-1]+10, y[-1]+10, "End")

## Plot Thermals
nbth = len(th_data["x"])
th_x_dat = th_data["x"].tolist()
th_y_dat = th_data["y"].tolist()
for i in range(0,nbth):
    x_th = th_x_dat[i]
    y_th = th_y_dat[i]
    ax.scatter(x_th,y_th,color='#ff6600',marker="x")

plt.show()

