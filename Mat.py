import math
import numpy as np
import Utils as Util


def rotate_about_axis(source, angle_degs, rotation_axis):
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
    for i in range(0, 3):
        result_row = 0
        for j in range(0, 3):
            result_row += rotation_matrix[i][j] * source[j]
        result_times.append(result_row)
    return result_times


def normalization(vector):
    l = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if l==0:
        l+=0.001
    normal_vector = [vector[0] / l, vector[1] / l, vector[2] / l]
    return normal_vector


def angle_between_degrees(v1,v2):
    v1 = Util.normalization(v1)
    v2 = Util.normalization(v2)
    # print(Util.dot_product(v1,v2))
    return math.acos(round(Util.dot_product(v1,v2),5))*180/np.pi


def dot_product(v1,v2):
    v1 = Util.normalization(v1)
    v2 = Util.normalization(v2)
    return np.inner(v1,v2)


 # multiplication of two quaternion(representing the orientation of joints)
def multiply_two_quaternion(q1, q2):
        a = q1[0]
        b = q1[1]
        c = q1[2]
        d = q1[3]
        e = q2[0]
        f = q2[1]
        g = q2[2]
        h = q2[3]

        m0 = round(a * e - b * f - c * g - d * h, 2)
        m1 = round(b * e + a * f + c * h - d * g, 2)
        m2 = round(a * g - b * h + c * e + d * f, 2)
        m3 = round(a * h + b * g - c * f + d * e, 2)
        return [m0, m1, m2, m3]
