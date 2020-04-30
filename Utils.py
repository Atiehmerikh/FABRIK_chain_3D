import math
import Mat
import numpy as np


def get_angle_between_degs(v1, v2):
    len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
    len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2)

    result = math.acos(np.dot(v1, v2) / (len_v1 * len_v2)) * 180 / math.pi
    return result


def get_distance_between(p1, p2):
    result = [x + y for x, y in zip(p2, np.dot(p1, -1))]
    return math.sqrt(result[0]**2+result[1]**2+result[2]**2)


def normalization(vector):
    l = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if l == 0:
        raise Exception("target is too close")
    else:
        normal_vector = [vector[0] / l, vector[1] / l, vector[2] / l]
        return normal_vector


def project_on_to_plane(vector, plane_normal):
    plane_normal_length = math.sqrt(plane_normal[0] ** 2 +
                                    plane_normal[1] ** 2 +
                                    plane_normal[2] ** 2)
    if plane_normal_length < 0:
        raise Exception("Plane normal can not be zero")
    n = Mat.normalization(plane_normal)
    vector = Mat.normalization(vector)
    dott = np.inner(vector, plane_normal)
    result = [x + y for x, y in zip(vector, np.dot(n,dott))]
    return Mat.normalization(result)


def gen_perpendicular_vector_quick(vector):
    if abs(vector[1]) < 0.99:
        perp = [vector[2], 0, vector[0]]
    else:
        perp = [vector[2], 0, -vector[1]]

    return Mat.normalization(perp)


def get_angle_limited_uv(vector_to_limit, vector_base_line, angle_limit_degs):
    vector_to_limit_length = math.sqrt(vector_to_limit[0] ** 2 + vector_to_limit[1] ** 2 + vector_to_limit[2] ** 2)
    vector_base_line_length = math.sqrt(vector_base_line[0] ** 2 + vector_base_line[1] ** 2 + vector_base_line[2] ** 2)

    angle_between = math.acos(np.inner(vector_to_limit, vector_base_line) / (
                vector_to_limit_length * vector_base_line_length)) * 180 / math.pi
    if angle_between > angle_limit_degs:
        correction_axis = (vector_to_limit ^ vector_base_line).normalization()
        return Mat.rotate_about_axis(vector_base_line, angle_limit_degs, correction_axis).normalization()
    else:
        return Mat.normalization(vector_to_limit)


def create_rotation_matrix(reference_direction):
    if reference_direction[1] == 1.0:
        reference_direction[1] -= 0.0001
        return Mat.normalization(reference_direction)

    m20 = reference_direction[0]
    m21 = reference_direction[1]
    m22 = reference_direction[2]
    cross = np.cross(reference_direction, [0, 1, 0])
    l = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    x_dir = [cross[0] / l, cross[1] / l, cross[2] / l]
    m00 = x_dir[0]
    m01 = x_dir[1]
    m02 = x_dir[2]

    cross = np.cross([m00, m01, m02], [m20, m21, m22])
    l = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    y_dir = [cross[0] / l, cross[1] / l, cross[2] / l]

    m10 = y_dir[0]
    m11 = y_dir[1]
    m12 = y_dir[2]

    rotation_matrix = [[m00, m01, m02],
                       [m10, m11, m12],
                       [m20, m21, m22]]
    return rotation_matrix


def times(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2]]


def get_signed_angle_between_degs(reference_vector, other_vector, normal_vector):
    reference_vector_length = math.sqrt(reference_vector[0] ** 2 + reference_vector[1] ** 2 + reference_vector[2] ** 2)
    other_vector_length = math.sqrt(other_vector[0] ** 2 + other_vector[1] ** 2 + other_vector[2] ** 2)

    unsigned_angle = math.acos(
        np.inner(reference_vector, other_vector) / (reference_vector_length * other_vector_length)) * 180 / math.pi
    sign_measure = np.inner(np.cross(reference_vector, other_vector), normal_vector)
    if sign_measure >= 0:
        sign = 1
    else:
        sign = -1
    return np.dot(unsigned_angle, sign)


def negated(vector):
    return np.dot(-1, vector)


def approximately_equal(a, b, tolerance):
        if abs(a - b) < tolerance:
            return 1
        else:
            return 0


def length_calc(vector):
    return math.sqrt(vector[0]**2+vector[1]**2+vector[2]**2)