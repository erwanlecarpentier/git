import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from numpy import ma
from matplotlib import colors, ticker, cm
from matplotlib.mlab import bivariate_normal

filename="data/updraft_field.dat"
data = pd.read_csv(filename,sep = ' ')

x = data["x"]
y = data["y"]
wz = data["updraft"]

N = int(len(wz)**.5)
wz = wz.values.reshape(N, N)
wz = (wz.T)[::-1] # fix reshape

plt.imshow(wz, interpolation='none', extent=(np.amin(x), np.amax(x), np.amin(y), np.amax(y)), cmap='hot')
plt.colorbar()
plt.xlabel('x')
plt.ylabel('y')

plt.show()

