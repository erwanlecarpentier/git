import sys, os
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

filename="data/updraft_field.dat"
data = pd.read_csv(filename,sep = ' ')

x = data["x"]
y = data["y"]
wz = data["updraft"]

fig = plt.figure()
ax = fig.gca(projection='3d')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Updraft velocity [m/s]')
cs = ax.plot_trisurf(x,y,wz,cmap=cm.jet, linewidth=0.2)

plt.show()

