import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from pycg3d.cg3d_point import CG3dPoint
from pycg3d import utils
import numpy as np


class Draw:
    def __init__(self, joints, target, first_pos):
        self.joints = joints
        self.first_pose = first_pos
        self.target = target

    def set_axes_radius(self, ax, origin, radius):
        ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
        ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
        ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

    def set_axes_equal(self, ax):
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ])

        origin = np.mean(limits, axis=1)
        radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
        self.set_axes_radius(ax, origin, radius)

    def fill_array(self, joints):
        w, h = 3, len(joints)
        coordinate = [[0 for x in range(w)] for y in range(h)]
        # coordinate =[][len(body_part_index)]
        x = []
        y = []
        z = []
        for i in range(len(joints)):
            x.append(joints[i][0])
            y.append(joints[i][1])
            z.append(joints[i][2])
        coordinate[0][:] = x
        coordinate[1][:] = y
        coordinate[2][:] = z
        return coordinate

    def draw_final(self):
        # The Second Pose
        coordinate = self.fill_array(self.joints)
        x_prime_right_arm = coordinate[0]
        y_prime_right_arm = coordinate[1]
        z_prime_right_arm = coordinate[2]


        # # ...............................................................................................
        coordinate = self.fill_array( self.first_pose)
        x_RArm = coordinate[0]
        y_RArm = coordinate[1]
        z_RArm = coordinate[2]


        fig = plt.figure()
        ax = fig.gca(projection='3d')

        ax.plot3D(x_RArm, y_RArm, z_RArm, color='red')

        ax.plot3D(x_prime_right_arm, y_prime_right_arm,
                  z_prime_right_arm, color='green')

        # # ax.scatter3D(joints[head[1]][0], joints[head[1]][1], joints[head[1]][2],edgecolors='green')
        ax.scatter3D(self.target[0], self.target[1], self.target[2])
        #
        self.set_axes_equal(ax)
        plt.show()

    def distance_calculation(i, j):
        i_point = CG3dPoint(i[0], i[1], i[2])
        j_point = CG3dPoint(j[0], j[1], j[2])

        return utils.distance(i_point, j_point)
