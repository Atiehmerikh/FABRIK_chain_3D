import numpy as np
import matplotlib.pyplot as plt
from fabrik_chain_3d.output_writer import *
import math
from fabrik_chain_3d import Mat as Mat, Utils as Util
from mpl_toolkits.mplot3d import Axes3D


class Visualization():
    def __init__(self,target_position,chain,deg,rotations,base_address="./test/output/", joints_file_address="joints-position.txt"):
        self.target_position = target_position
        self.chain = chain
        self.deg = deg
        self.rotations = rotations
        self.base_address = base_address
        self.joints_file_address = joints_file_address
        self.bone_twist_limit = 2.8973 * 180 / math.pi
        self.fixed_base_orientation = [1, 0, 0, 0]

    def solve_for_orientation(self, outer_joint_orientation, inner_joint_orientation, loop):
        q1 = outer_joint_orientation
        q2 = inner_joint_orientation
        # finding the rotor that express rotation between two orientational frame(between outer and inner joint)
        rotor = Util.Utils().find_rotation_quaternion(q1, q2)
        if rotor[0] > 1:
            rotor[0] = 0.99
        if rotor[0] < -1:
            rotor[0] = -0.99
        needed_rotation = math.acos(rotor[0]) * 2 * (180 / np.pi)
        self.rotations[loop] = needed_rotation * (np.pi / 180)
        if needed_rotation <= self.bone_twist_limit:
            # if the rotation is inside the limited
            return Mat.Mat().multiply_two_quaternion(rotor, outer_joint_orientation)
        else:
            # the maximum allowed rotation angle
            theta = (self.bone_twist_limit) * (np.pi / 180)
            self.rotations[loop] = theta
            # the rotation axis
            if abs(rotor[0]) == 1:
                return rotor
            v1 = np.dot(rotor[1:], (1 / math.sqrt(1 - rotor[0] ** 2)))
            w = math.cos(theta / 2)
            x = v1[0] * math.sin(theta / 2)
            y = v1[1] * math.sin(theta / 2)
            z = v1[2] * math.sin(theta / 2)
            return [w, x, y, z]

    def angles(self):
            angles = []
            # for base bone twist
            base_bone = self.chain[0]
            base_bone_orientation = base_bone.get_bone_orientation()
            # number 3 belongs to the rotation matrix which is 0 for bone 3 rotation, 1 for bone 5 rotation,
            # 2 for bone 7 rotation, 3 for bone 0 rotation
            self.solve_for_orientation(base_bone_orientation, self.fixed_base_orientation, 3)
            angles.append(self.rotations[3])
            # for bone 2(in schematic)
            angles.append(self.deg[0])
            angles.append(self.rotations[0])
            # for bone 3(in schematic)
            angles.append(self.deg[1])
            angles.append(self.rotations[1])
            # for bone 4(in schematic)
            angles.append(self.deg[2])
            angles.append(self.rotations[2])

            s = ','.join([str(n) for n in angles])
            print(s)

    def draw_chain(self):
            self.angles()
            coordinate = self.fill_array(self.points_retrieval())
            x_prime = coordinate[0]
            y_prime = coordinate[1]
            z_prime = coordinate[2]

            # f = OutputWriter(self.base_address, self.joints_file_address).joint_writer()
            # for i in range(0, len(x_prime)):
            #     f.write(str(x_prime[i]))
            #     f.write(' ')
            #     f.write(str(y_prime[i]))
            #     f.write(' ')
            #     f.write(str(z_prime[i]))
            #
            #     f.write("\n")
            # f.close()
            fig = plt.figure()
            # ax = fig.gca(projection='3d')
            ax = Axes3D(fig)
            ax.plot3D(x_prime, y_prime, z_prime, color='red')
            ax.scatter3D(self.target_position[0], self.target_position[1], self.target_position[2])
            ax.scatter3D(x_prime, y_prime, z_prime)

            for i in range(len(x_prime) - 1):
                print('length bone ' + str(i + 1))
                x = x_prime[i] - x_prime[i + 1]
                y = y_prime[i] - y_prime[i + 1]
                z = z_prime[i] - z_prime[i + 1]
                print("%.2f" % round(math.sqrt(x * x + y * y + z * z), 2))
            plt.show()

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

    def fill_array(self, start_locations):
            w, h = 3, len(start_locations)
            coordinate = [[0 for x in range(w)] for y in range(h)]
            # coordinate =[][len(body_part_index)]
            x = [0]
            y = [0]
            z = [0]
            length = [0.316, 0.088, 0.088]
            for i in range(len(start_locations)):
                if i >= 1:
                    uv = [(start_locations[i][0] - start_locations[i - 1][0]),
                          (start_locations[i][1] - start_locations[i - 1][1]),
                          (start_locations[i][2] - start_locations[i - 1][2])]
                    uv = Mat.Mat().normalization(uv)
                    scale = np.dot(uv, length[i - 1])
                    middle_points = [a + b for a, b in zip(start_locations[i - 1], scale)]
                    x.append(middle_points[0])
                    y.append(middle_points[1])
                    z.append(middle_points[2])

                x.append(start_locations[i][0])
                y.append(start_locations[i][1])
                z.append(start_locations[i][2])

            coordinate[0][:] = x
            coordinate[1][:] = y
            coordinate[2][:] = z
            return coordinate

    def points_retrieval(self):
            start_locations = []
            end_locations = []
            for i in range(0, len(self.chain)):
                start_loc = self.chain[i].get_start_point_position()
                # end_loc = self.chain.get_bone(i).set_end_point()
                start_locations.append(start_loc)
                # end_locations.append(end_loc)
            end_effector_bone = self.chain[len(self.chain) - 1].end_point
            start_locations.append(end_effector_bone)
            return start_locations