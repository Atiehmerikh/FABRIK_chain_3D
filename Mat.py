from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_point import CG3dVector
import math


def rotate_about_axis(source, angle_degs, rotation_axis):
    rotation_axis = CG3dVector(rotation_axis[0], rotation_axis[1], rotation_axis[2])
    source = CG3dVector(source[0], source[1], source[2])
    sin_theta = math.sin(angle_degs * math.pi / 180)
    cos_theta = math.cos(angle_degs * math.pi / 180)
    one_minus_cos_theta = 1.0 - cos_theta

    xy_one = rotation_axis[0] * rotation_axis[1] * one_minus_cos_theta
    xz_one = rotation_axis[0] * rotation_axis[2] * one_minus_cos_theta
    yz_one = rotation_axis[1] * rotation_axis[2] * one_minus_cos_theta

    m00 = rotation_axis[0] * rotation_axis[0] * one_minus_cos_theta + cos_theta
    m01 = xy_one + rotation_axis[2] * sin_theta
    m02 = xz_one - rotation_axis[1] * sin_theta

    m10 = xy_one - rotation_axis[2] * sin_theta
    m11 = rotation_axis[1] * rotation_axis[1] * one_minus_cos_theta + cos_theta
    m12 = yz_one + rotation_axis[0] * sin_theta

    m20 = xz_one + rotation_axis[1] * sin_theta
    m21 = yz_one - rotation_axis[0] * sin_theta
    m22 = rotation_axis[2] * rotation_axis[2] * one_minus_cos_theta + cos_theta

    rotation_matrix = [[m00, m01, m02],
                       [m10, m11, m12],
                       [m20, m21, m22]]
    result_times = []
    for i in range(0, len(rotation_matrix)):
        result_row = 0
        for j in range(0, 3):
            result_row += rotation_matrix[i][j] * source[j]
        result_times.append(result_row)
    rotated_vector = CG3dVector(result_times[0], result_times[1], result_times[2])
    return rotated_vector

