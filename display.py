#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 11 12:31:22 2018

@author: mickn
"""

"""
Display Motion History of SC
"""

import numpy as np
import matplotlib.pyplot as plt
import time

t_i = time.clock()
#images = []     #array for animation
#fig = plt.figure(figsize=(27,9))
delt=0.001

#interval = np.arange(-0.6,0.6,0.05)
#plt.colorbar()

quat = np.loadtxt('./quat_eom.csv', delimiter=',')
x = np.arange(quat.size/quat[0].size).reshape(-1)
labels=["q1","q2","q3","q4","wx","wy","wz"]

fig = plt.figure(figsize=(9,3))
plt.xlabel("time [s]")
plt.ylabel("q []")
for i in range(4):
    #fig = plt.figure(figsize=(27,9))
    plt.plot(x*delt, quat[:,i], label=labels[i])
    filename = "./images/quat{}.png".format(i)
    plt.legend()
    plt.savefig(filename)
    print(i)

fig = plt.figure(figsize=(9,3))
plt.xlabel("time [s]")
plt.ylabel("omega [rad/s]")
for i in range(4,7):
#    fig = plt.figure(figsize=(27,9))
    plt.plot(x*delt, quat[:,i], label=labels[i])
    filename = "./images/quat{}.png".format(i)
    plt.legend()
    plt.savefig(filename)
    print(i)

#ani = animation.FuncAnimation(fig, images, interval=100)


t_e = time.clock()
print("runtime:",t_e-t_i, "[sec]")