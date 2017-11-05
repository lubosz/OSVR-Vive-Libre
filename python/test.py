#!/usr/bin/python3

import signal
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors
import json
import pyvive

from math import tan
import math

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
    self.vive = pyvive.PyViveLibre()
    jet = cm = plt.get_cmap('jet') 
    cNorm  = colors.Normalize(vmin=0, vmax=31)
    self.scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

  def get_light_sensor_positions(self, config_text):
      j = json.loads(config_text)
      points = j["lighthouse_config"]["modelPoints"]
      normals = j["lighthouse_config"]["modelNormals"]
      return points, normals

  def plot_config(self):
      vectors = []
      
      f = plt.figure()
      
      config_str = self.vive.get_config().decode()
      
      points, normals = self.get_light_sensor_positions(config_str)

      
      ax = f.add_subplot(1,1,1, aspect=1, projection='3d')

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

      plt.show()


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


  def plot_angles(self):
    
    f = plt.figure()
    
    #ax = f.add_subplot(2,2,2, aspect=1)

    plt.ion()
    
    handler = SigHandler()
    

    
	  # The J axis (horizontal) sweep starts 71111 ticks after the sync
	  # pulse start (32°) and ends at 346667 ticks (156°).
	  # The K axis (vertical) sweep starts at 55555 ticks (23°) (25?) and ends
	  # at 331111 ticks (149°).



    while not handler.quit:
        plt.clf()

        angles = self.vive.pollAngles()
        #plt.axis([149, 25, 32, 156])
        plt.axis([0, 149-25, 0, 156-32])
        #plt.axis([-0.14, 4.4, 0.66, -1.9])
        
        #plt.axis([-50, 50, -50, 50])
        
        """
        plt.axhline(y=71111)
        plt.axhline(y=346667)
                
        plt.axvline(x=55555)
        plt.axvline(x=331111)
        """
        
        for i, a in angles.items():
          x_rad =(149-25) - ((a[0]/2222.22)-25)
          y_rad =(a[1]/2222.22)-32
        
          plt.scatter((x_rad), (y_rad), color=self.scalarMap.to_rgba(i))
          
        plt.pause(0.05)

  def plot_pnp(self):
        #retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]])

        objectPoints = np.random.random((10,3,1))
        print("object points", objectPoints)

        imagePoints = np.random.random((10,2,1))
        print("image points", imagePoints)

        cameraMatrix = np.eye(3)
        distCoeffs = np.ones(4)

        print("dist coeffs", distCoeffs)

        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)

        print("retval", retval)
        print("rvec", rvec)
        print("tvec", tvec)

        self.plot_r_t_vec(rvec, tvec, f)

  def plot_actual_pnp(self):
        #retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]])

        

        #f = plt.figure()
        config_str = self.vive.get_config().decode()

        points, normals = self.get_light_sensor_positions(config_str)
        np_points = np.array(points)
        
        #embed()

        print("point 4", points[4])

        objectPoints = np.random.random((10,3,1))
        #print("object points", objectPoints)
        
        #print("points", np_points)
        
        #cameraMatrix = np.eye(3)
        
        fx = 1.0
        fy = 1.0
        cx = 0.0
        cy = 0.0
        
        cameraMatrix = np.array([[fx,0.0,cx],
                         [0.0,fy,cy],
                         [0.0,0.0,1.0]])
        
        #print("cameraMatrix", cameraMatrix)
        
        fx = 0.5
        w = 149-25
        h = 156-32
        
        cameraMatrix = np.float64([[fx*w, 0, 0.5*(w-1)],
                        [0, fx*w, 0.5*(h-1)],
                        [0.0,0.0,      1.0]])
        
        #print("cam2", cam2)
        
        #exit()
        
        distCoeffs = np.zeros(4)
        
        plt.ion()
    
        handler = SigHandler()



        x_min = -0.001
        x_max = 0.001
        y_min = -0.001
        y_max = 0.001
        z_min = -0.001
        z_max = 0.001

        xs = []
        ys = []
        zs = []
        
        f = plt.figure()

        ax = f.add_subplot(1,1,1, aspect=1, projection='3d')

        while not handler.quit:
            #plt.clf()

            object_points = []
            image_points = []

            ax.set_xlim3d(-2.5, 2.5)
            ax.set_ylim3d(-2.5, 2.5)
            ax.set_zlim3d(-2.5, 2.5)
            
            max_samples = 40
            j = 0

            angles = self.vive.pollAngles()
            #plt.axis([100000, 300000, 100000, 300000])
            for i, a in angles.items():
              #plt.scatter(a[0], a[1], color=self.scalarMap.to_rgba(i))
              #image_points.append(a)
              #image_points.append([a[0]/2222.22, a[1]/2222.22])
              j+=1
              

              
              x_rad = (a[0]/2222.22)-25
              y_rad = (a[1]/2222.22)-32
              
              vec2d = [x_rad, y_rad]
        
              image_points.append(vec2d)
              
              fixed_points = [points[i][0], -points[i][2], points[i][1]]
              object_points.append(fixed_points)
              #vec3d = np.array(points[i]).reshape((3,1))
              #object_points.append(points[i])
              
              if j >= max_samples:
                break
              
            image_points_np = np.array(image_points)
            object_points_np = np.array(object_points)
            
            print("we have %d image points and %d object points" % (len(image_points), len(object_points)))
            
            if len(object_points_np) > 3 and len(image_points) == len(object_points):
            
              print("image points", image_points_np)
               
              print("object points", object_points_np)
               
              #retval, rvec, tvec = cv2.solvePnP(object_points_np, image_points_np, cameraMatrix, distCoeffs)
              
              retval, rvec, tvec, inliers = cv2.solvePnPRansac(object_points_np, image_points_np, cameraMatrix, distCoeffs, useExtrinsicGuess=False)
              
              #exit()
              
              print("RET", retval, "tvec", tvec, "rvec", rvec)
              
              """
              """
              
              if retval:
              
                mult = 10
                
                if tvec[0] > x_max: # and tvec[0] < x_max * mult:
                  x_max = tvec[0]
                if tvec[0] < x_min: # and tvec[0] > x_min * mult:
                  x_min = tvec[0]
                 
                if tvec[1] > y_max: # and tvec[1] < y_max * mult:
                  y_max = tvec[1]
                if tvec[1] < y_min: # and tvec[1] > y_min * mult:
                  y_min = tvec[1]
                  
                if tvec[2] > z_max: # and tvec[2] < z_max * mult:
                  z_max = tvec[2]
                if tvec[2] < z_min: # and tvec[2] > z_min * mult:
                  z_min = tvec[2]
                """
                xs.append(tvec[0])
                ys.append(tvec[1])
                zs.append(tvec[2])
                """
                
                #ax.set_xlim3d(x_min, x_max)
                #ax.set_ylim3d(y_min, y_max)
                #ax.set_zlim3d(z_min, z_max)
                
                ax.scatter(tvec[0], tvec[1], tvec[2], depthshade=False)


              
            plt.pause(0.05)

        """
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
        """

if __name__ == '__main__':


  #config_json = json.loads(config_str)
  
  vive_pos = VivePos()

  #embed()
  #vive_pos.plot_config()
  
  vive_pos.plot_actual_pnp()
  
  #vive_pos.plot_config_and_pnp()

  #
  #print(config_str)

  #vive_pos.plot_angles()


  print("Goodbye!")

