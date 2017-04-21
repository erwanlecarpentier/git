
"""
Plot the evolution of every variables
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
from mpl_toolkits.mplot3d import Axes3D

plt.close('all')
data = np.loadtxt("data/state.dat",dtype=float)
data_wind = np.loadtxt("data/wind.dat",dtype=float)

x = data[:,0]
y = data[:,1]
z = data[:,2]
V = data[:,3]
gamma = data[:,4]
khi = data[:,5]
alpha = data[:,6]
beta = data[:,7]
sigma = data[:,8]
Edot = data[:,9]
t = data[:,10]
updraft = data_wind[:,2]

def b():
	return '#333399';
def o():
	return '#ff6600';
def g():
	return '#00cc66';

ax1 = plt.subplot(2,2,1)
plt.plot(t,z,label='z',color=b())
ax1.set_ylabel('z', color=b())
ax1.tick_params('y', colors=b())
ax1bis = ax1.twinx()
ax1bis.plot(t,updraft,color=o())
ax1bis.set_ylabel('Updraft', color=o())
ax1bis.tick_params('y', colors=o())
ax1bis.axhline(y=0., color='black', linestyle='--')

ax2 = plt.subplot(2,2,2)
plt.plot(t,V,label='V',color=b())
plt.plot(t,Edot,label='Edot',color=o())
plt.legend(loc='upper left')
plt.axhline(y=0., color='black', linestyle='--')

ax3 = plt.subplot(2,2,3)
plt.plot(t,gamma,label='gamma',color=b())
plt.plot(t,khi,label='khi',color=o())
plt.legend(loc='upper left')

ax4 = plt.subplot(2,2,4)
plt.plot(t,alpha,label='alpha',color=b())
plt.plot(t,beta,label='beta',color=o())
plt.plot(t,sigma,label='sigma',color=g())
plt.legend(loc='upper left')

ax1.set_xlabel('time (s)')
ax2.set_xlabel('time (s)')
ax3.set_xlabel('time (s)')
ax4.set_xlabel('time (s)')

plt.show()

