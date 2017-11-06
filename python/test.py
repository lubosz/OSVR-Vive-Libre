#!/usr/bin/python3

import json
import pyvive
import signal

import cv2
import matplotlib.cm as cmx
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import numpy as np


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
        jet = plt.get_cmap('jet')
        c_norm = colors.Normalize(vmin=0, vmax=31)
        self.scalarMap = cmx.ScalarMappable(norm=c_norm, cmap=jet)

    @staticmethod
    def get_light_sensor_positions(config_text):
        j = json.loads(config_text)
        points = j["lighthouse_config"]["modelPoints"]
        normals = j["lighthouse_config"]["modelNormals"]
        return points, normals

    def plot_config(self):
        vectors = []

        f = plt.figure()

        config_str = self.vive.get_config().decode()

        points, normals = self.get_light_sensor_positions(config_str)

        ax = f.add_subplot(1, 1, 1, aspect=1, projection='3d')

        for i in range(0, 32):
            v = [points[i][0], -points[i][2], points[i][1],
                 normals[i][0], -normals[i][2], normals[i][1]]
            vectors.append(v)
        i = 0
        for p in points:
            ax.scatter(p[0], -p[2], p[1], color=self.scalarMap.to_rgba(i))
            i += 1

        soa = np.array(vectors)
        x, y, z, u, v, w = zip(*soa)
        ax.quiver(x, y, z, u, v, w, pivot='tail', length=0.1)

        ax.set_xlim3d(-0.2, 0.2)
        ax.set_ylim3d(-0.2, 0.2)
        ax.set_zlim3d(-0.2, 0.2)

        plt.show()

    def plot_angles(self):
        # ax = f.add_subplot(2,2,2, aspect=1)

        plt.ion()

        handler = SigHandler()

        # The J axis (horizontal) sweep starts 71111 ticks after the sync
        # pulse start (32°) and ends at 346667 ticks (156°).
        # The K axis (vertical) sweep starts at 55555 ticks (23°) (25?) and ends
        # at 331111 ticks (149°).

        while not handler.quit:
            plt.clf()

            angles = self.vive.pollAngles()
            # plt.axis([149, 25, 32, 156])
            plt.axis([0, 149-25, 0, 156-32])
            # plt.axis([-0.14, 4.4, 0.66, -1.9])

            # plt.axis([-50, 50, -50, 50])

            for i, a in angles.items():
                x_rad = (149 - 25) - ((a[0] / 2222.22) - 25)
                y_rad = (a[1] / 2222.22) - 32

                plt.scatter(x_rad, y_rad, color=self.scalarMap.to_rgba(i))

            plt.pause(0.05)

    def plot_actual_pnp(self):
        config_str = self.vive.get_config().decode()

        points, normals = self.get_light_sensor_positions(config_str)

        # cameraMatrix = np.eye(3)

        """
        fx = 1.0
        fy = 1.0
        cx = 0.0
        cy = 0.0

        cameraMatrix = np.array([[fx,0.0,cx],
                                 [0.0,fy,cy],
                                 [0.0,0.0,1.0]])
        """
        # print("cameraMatrix", cameraMatrix)

        fx = 0.5
        w = 149-25
        h = 156-32

        camera_matrix = np.float64([[fx * w, 0.0,    0.5*(w-1)],
                                    [0.0,    fx * w, 0.5*(h-1)],
                                    [0.0,    0.0,    1.0]])

        dist_coeffs = np.zeros(4)

        plt.ion()

        handler = SigHandler()

        f = plt.figure()

        ax = f.add_subplot(1, 1, 1, aspect=1, projection='3d')

        while not handler.quit:
            # plt.clf()

            object_points = []
            image_points = []

            ax.set_xlim3d(-2.5, 2.5)
            ax.set_ylim3d(-2.5, 2.5)
            ax.set_zlim3d(-2.5, 2.5)

            max_samples = 40
            j = 0

            angles = self.vive.pollAngles()
            # plt.axis([100000, 300000, 100000, 300000])
            for i, a in angles.items():
                # plt.scatter(a[0], a[1], color=self.scalarMap.to_rgba(i))
                # image_points.append(a)
                # image_points.append([a[0]/2222.22, a[1]/2222.22])
                j += 1

                x_rad = (a[0]/2222.22)-25
                y_rad = (a[1]/2222.22)-32

                vec2d = [x_rad, y_rad]

                image_points.append(vec2d)

                fixed_points = [points[i][0], -points[i][2], points[i][1]]
                object_points.append(fixed_points)
                # vec3d = np.array(points[i]).reshape((3,1))
                # object_points.append(points[i])

                if j >= max_samples:
                    break

            image_points_np = np.array(image_points)
            object_points_np = np.array(object_points)

            print("we have %d image points and %d object points" % (len(image_points), len(object_points)))

            if len(object_points_np) > 3 and len(image_points) == len(object_points):
                print("image points", image_points_np)
                print("object points", object_points_np)

                # retval, rvec, tvec = cv2.solvePnP(object_points_np, image_points_np, cameraMatrix, distCoeffs)
                retval, rvec, tvec, inliers = cv2.solvePnPRansac(object_points_np, image_points_np, camera_matrix,
                                                                 dist_coeffs, useExtrinsicGuess=False)

                print("RET", retval, "tvec", tvec, "rvec", rvec)

                if retval:
                    ax.scatter(tvec[0], tvec[1], tvec[2], depthshade=False)
            plt.pause(0.05)

if __name__ == '__main__':
    vive_pos = VivePos()

    # vive_pos.plot_config()
    vive_pos.plot_actual_pnp()
    # vive_pos.plot_angles()

    print("Goodbye!")

