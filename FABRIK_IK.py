import numpy as np
import math
from pycg3d.cg3d_point import CG3dPoint
from pycg3d import utils
from Constraints import constraints
from draw import Draw
# import angle_calculator as angleCalculator


# This is solver of Inverse kinematic of a whole chain of human body with foot on the ground
# For more info about the procedure refer to https://www.sciencedirect.com/science/article/pii/S1524070311000178
# and for the constraints refer to https://www.researchgate.net/publication/271771862_Extending_FABRIK_with_model_constraints


def distance_calculation(i, j):
    i_point = CG3dPoint(i[0], i[1], i[2])
    j_point = CG3dPoint(j[0], j[1], j[2])

    return utils.distance(i_point, j_point)


class FABRIK:

    def __init__(self, joints, orientation, target, target_orientation, length, theta, constraint_type,
                 rotation_angle_limit):
        self.n = len(joints)
        self.joints = joints
        self.orientation = orientation
        self.constraint_type = constraint_type
        self.Theta = theta
        self.firstPos = length
        self.tolerance = 0.01
        self.target = target
        self.targetOrientation = target_orientation
        self.rotation_angle_limit = rotation_angle_limit
        self.end_effector = 4
        self.n = len(joints)

    def position_calculator(self, i, j):
        r = distance_calculation(self.joints[i], self.joints[j])
        landa = distance_calculation(self.firstPos[i], self.firstPos[j]) / r
        pos = (1 - landa) * self.joints[i] + landa * self.joints[j]
        self.joints[j] = pos

    def orientation_calculator(self, outer_joint, inner_joint):
        q1 = self.orientation[outer_joint]
        q2 = self.orientation[inner_joint]
        theta1 = math.acos(q1[0]) * 2 * (180 / np.pi)
        theta2 = math.acos(q2[0]) * 2 * (180 / np.pi)
        v1 = q1[1:] / math.sqrt(1 - q1[0] ** 2)

        needed_rotation = abs(theta2 - theta1)
        if needed_rotation <= self.rotation_angle_limit:
            self.orientation[inner_joint] = self.orientation[outer_joint]
        else:
            theta = (self.rotation_angle_limit) * (np.pi / 180)
            w = math.cos(theta / 2)
            x = v1[0] * math.sin(theta / 2)
            y = v1[1] * math.sin(theta / 2)
            z = v1[2] * math.sin(theta / 2)
            self.orientation[inner_joint] = [w, x, y, z]

    def forward_arm(self, target, target_orientation):
        # set end effector as target
        self.joints[self.n - 1] = target
        self.orientation[self.n - 1] = target_orientation
        for i in range(self.n - 2, -1, -1):
            self.position_calculator(i + 1, i)
            self.orientation_calculator(i + 1, i)
            if i < self.n - 2:
                mconstraint = constraints(self.joints, i + 1, self.Theta[i + 1], self.firstPos,
                                          self.constraint_type[i + 1], self.orientation[i + 1])
                constraint_return = mconstraint.rotational_constraint()
                if constraint_return[0] != 0:
                    for j in range(3):
                        self.joints[i][j] = constraint_return[j]

    def backward_arm(self):
        # set root as initial position
        self.joints[0] = self.firstPos[0]
        n = len(self.joints)
        for i in range(2, n):
            self.position_calculator(i - 1, i)

    def solve(self):
        counter = 0
        sum_l = 0
        n = len(self.firstPos)
        ###################################################################
        # is target in reach or not

        # arm length
        for i in range(0,n-1):
            sum_l = sum_l + distance_calculation(self.firstPos[i],self.firstPos[i + 1])

        if sum_l < distance_calculation(self.firstPos[0], self.target):
            print("target is out of reach!!!!!!!")
            return
        else:
            dif = distance_calculation(self.joints[n-1], self.target)

        # target is in reach

    ##########################################################################
            while dif > self.tolerance:
                self.forward_arm( self.target, self.targetOrientation)
                self.backward_arm()
                dif = distance_calculation(self.joints[n - 1], self.target)
                counter = counter + 1
                if counter > 10:
                    break

            # f = open("angles.txt", "w")
            # m_angle = angleCalculator.AngleCalculator(f, self.joints)
            # m_angle.calculate()
            draw_obj = Draw(self.joints, self.target, np.loadtxt("joints_position_fixed.txt"))
            draw_obj.draw_final()
