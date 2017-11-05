#!/usr/bin/python3

import signal
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors
import json
import pyvive

from IPython import embed
from mpl_toolkits.mplot3d import Axes3D

class SigHandler:
  def __init__(self):
    self.quit = False
    signal.signal(signal.SIGINT, self.exit)
    signal.signal(signal.SIGTERM, self.exit)

  def exit(self, signum, frame):
    self.quit = True

class VivePos:

  def __init__(self):
    jet = cm = plt.get_cmap('jet') 
    cNorm  = colors.Normalize(vmin=0, vmax=31)
    self.scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

  def get_light_sensor_positions(self, config_text):
      j = json.loads(config_text)
      points = j["lighthouse_config"]["modelPoints"]
      normals = j["lighthouse_config"]["modelNormals"]
      return points, normals

  def plot_config(self, config_text, f):
      vectors = []
      points, normals = self.get_light_sensor_positions(config_text)

      
      ax = f.add_subplot(2,2,1, aspect=1, projection='3d')

      for i in range(0, 32):
          v = [points[i][0], -points[i][2], points[i][1],
               normals[i][0], -normals[i][2], normals[i][1]]
          vectors.append(v)

      i = 0
      for p in points:
          ax.scatter(p[0], -p[2], p[1], color=self.scalarMap.to_rgba(i))
          i+=1

      soa = np.array(vectors)
      x, y, z, u, v, w = zip(*soa)
      ax.quiver(x, y, z, u, v, w, pivot='tail', length=0.1)

      ax.set_xlim3d(-0.2, 0.2)
      ax.set_ylim3d(-0.2, 0.2)
      ax.set_zlim3d(-0.2, 0.2)

      #plt.show()


  def plot_r_t_vec(self, rvec, tvec, f):
      xs = []
      ys = []
      zs = []
      
      vectors = []

      ax = f.add_subplot(2,2,(3,4), aspect=1, projection='3d')

      #v1 = [0,0,0, tvec[0], tvec[1], tvec[2]]
      v2 = [tvec[0], tvec[1], tvec[2], tvec[0]+ rvec[0], tvec[1]+rvec[1], tvec[2]+rvec[2]]

      xs.append(tvec[0])
      ys.append(tvec[1])
      zs.append(tvec[2])

      #vectors.append(v1)
      vectors.append(v2)

      soa = np.array(vectors)
      x, y, z, u, v, w = zip(*soa)
      ax.quiver(x, y, z, u, v, w, pivot='tail', length=0.1)
      ax.scatter(xs, ys, zs, depthshade=False)

      ax.set_xlim3d(-2, 2)
      ax.set_ylim3d(-2, 2)
      ax.set_zlim3d(-2, 2)


  def plot_angles(self, vive, f):
    
    #ax = f.add_subplot(2,2,2, aspect=1)

    plt.ion()
    
    handler = SigHandler()

    while not handler.quit:
        plt.clf()

        angles = vive.pollAngles()
        plt.axis([100000, 300000, 100000, 300000])
        for i, a in angles.items():
          plt.scatter(a[0], a[1], color=self.scalarMap.to_rgba(i))
          
        plt.pause(0.05)

  def plot_config_and_pnp(self):
        self.plot_config(config_str, f)

        #retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]])

        objectPoints = np.random.random((10,3,1))
        print("object points", objectPoints)

        imagePoints = np.random.random((10,2,1))
        print("image points", imagePoints)

        cameraMatrix = np.eye(3)
        distCoeffs = np.zeros(4)

        print("dist coeffs", distCoeffs)

        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)

        print("retval", retval)
        print("rvec", rvec)
        print("tvec", tvec)

        self.plot_r_t_vec(rvec, tvec, f)
        
        plt.show()

if __name__ == '__main__':
  vive = pyvive.PyViveLibre()

  config_str = vive.get_config().decode()

  config_json = json.loads(config_str)

  f = plt.figure()
  
  vive_pos = VivePos()

  #embed()

  vive_pos.plot_config_and_pnp()

  #
  #print(config_str)

  #vive_pos.plot_angles(vive, f)


  print("Goodbye!")

