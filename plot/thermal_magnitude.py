import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from numpy import ma
from matplotlib import colors, ticker, cm
from matplotlib.mlab import bivariate_normal

D = pd.read_csv("data/updraft_field.dat",sep = ';')

# 1. Plot true model for t=500
data = D[D["t"]==0]

x = data["x"]
y = data["y"]
wz = data["updraft"]
N = int(len(wz)**.5)
wz = wz.values.reshape(N, N)
wz = (wz.T)[::-1] # fix reshape

ax1 = plt.subplot(2,1,1)
plt.imshow(wz, interpolation='none', extent=(np.amin(x), np.amax(x), np.amin(y), np.amax(y)), cmap='hot')
plt.colorbar()
ax1.set_title('True model; z = 10m; t = 0s')
ax1.set_xlabel('x')
ax1.set_ylabel('y')

# 2. Plot true model for t=550
data = D[D["t"]==50]

x = data["x"]
y = data["y"]
wz = data["updraft"]
N = int(len(wz)**.5)
wz = wz.values.reshape(N, N)
wz = (wz.T)[::-1] # fix reshape

ax2 = plt.subplot(2,1,2)
plt.imshow(wz, interpolation='none', extent=(np.amin(x), np.amax(x), np.amin(y), np.amax(y)), cmap='hot')
plt.colorbar()
ax2.set_title('True model; z = 10m; t = 50s')
ax2.set_xlabel('x')
ax2.set_ylabel('y')

plt.show()

