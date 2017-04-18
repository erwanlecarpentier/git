
from __future__ import division
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 15:38:06 2017

@author: adrien Bufort
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
from mpl_toolkits.mplot3d import Axes3D
from scipy import *
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d

"""
Dans cette partie on plot les résultats des fichiers de sorties
"""

"""
on utilise le script comme ca
./data_plot.py nom_du_fichier
avec le nom du fichier un fichier .txt avec lignes et colonnes
"""

## File reading
fname = "data/data_plane.txt"
data = np.loadtxt(fname,dtype=float)
data_coordonnees = data[:,[0,1,2]]

## Trajectory plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(data[:,0], data[:,1], data[:,2],color='#6699ff')
ax.set_xlim3d((-1300, 1300))
ax.set_ylim3d((-1300, 1300))
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

## Thermals plot
filename = "data/config1csv.txt"
data2 = pd.read_csv(filename,sep = ' ')
data_t_0 = data2[data2["tBirth"]==0]
size = len(data_t_0["CentreX"])
for i in range(0,size):
    x=linspace(data_t_0["CentreX"][i],data_t_0["CentreX"][i],1000)
    y=linspace(data_t_0["CentreY"][i],data_t_0["CentreY"][i],1000)
    z=linspace(0,1000,1000)
    ax.plot(x,y,z,color='#ff6600')

## Border plot
limit_circle = Circle((0, 0), 1200, color='black', fill=False)
ax.add_patch(limit_circle)
art3d.pathpatch_2d_to_3d(limit_circle, z=0, zdir="z")

plt.show()

