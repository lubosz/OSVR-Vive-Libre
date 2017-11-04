#!/usr/bin/python3

from IPython import embed

#import build/temp.linux-x86_64-3.6.pyvive

print("Oh hai")
embed()

#import pyvive
#a.poll()

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors


jet = cm = plt.get_cmap('jet') 
cNorm  = colors.Normalize(vmin=0, vmax=31)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

plt.ion()

import pyvive
vive = pyvive.PyViveLibre()
angles = vive.pollAngles()

for i, a in angles.items():
  #embed()
  plt.scatter(a[0], a[1])
plt.pause(0.05)

"""
for i in range(10):
    y = np.random.random()
    plt.scatter(i, y)
    plt.pause(0.05)

while True:
    plt.pause(0.05)
    
"""

while True:
    angles = vive.pollAngles()
    plt.clf()
    plt.axis([100000, 300000, 100000, 300000])
    for i, a in angles.items():
      plt.scatter(a[0], a[1], color=scalarMap.to_rgba(i))
    plt.pause(0.05)

