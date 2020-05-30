import math
import Mat
import numpy as np


def validate_direction(v1):
    length = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
    if length == 0:
        raise Exception("Length of vector cannot be zero.")


def validate_length(a):
    if a <= 0:
        raise Exception("Length must be a greater than or equal to zero.")


def get_angle_between_degs(v1, v2):
    len_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
    len_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2)

    result = math.acos(round(np.dot(v1, v2) / (len_v1 * len_v2),3)) * 180 / math.pi
    return result


def get_distance_between(p1, p2):
    result = [x + y for x, y in zip(p2, np.dot(p1, -1))]
    return math.sqrt(result[0] ** 2 + result[1] ** 2 + result[2] ** 2)


def normalization(vector):
    l = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    if l == 0:
        l += 0.01
    normal_vector = [vector[0] / l, vector[1] / l, vector[2] / l]
    return normal_vector


def dot_product(v1, v2):
    v1 = Mat.normalization(v1)
    v2 = Mat.normalization(v2)
    return np.inner(v1, v2)


def project_on_to_plane(vector, plane_normal):
    plane_normal_length = math.sqrt(plane_normal[0] ** 2 +
                                    plane_normal[1] ** 2 +
                                    plane_normal[2] ** 2)
    if plane_normal_length < 0:
        raise Exception("Plane normal can not be zero")
    n = Mat.normalization(plane_normal)
    vector = Mat.normalization(vector)
    dott = Mat.dot_product(vector, plane_normal)
    result = [x - y for x, y in zip(vector, np.dot(n, dott))]
    return Mat.normalization(result)


def gen_perpendicular_vector_quick(vector):
    if abs(vector[1]) < 0.99:
        perp = [vector[2], 0, vector[0]]
    else:
        perp = [vector[2], 0, -vector[1]]

    return Mat.normalization(perp)


def get_angle_limited_uv(vector_to_limit, vector_base_line, angle_limit_degs):
    angle_between = Mat.angle_between_degrees(vector_base_line, vector_to_limit)
    if angle_between > angle_limit_degs:
        correction_axis = Mat.normalization(
            np.cross(Mat.normalization(vector_base_line), Mat.normalization(vector_to_limit)))
        return Mat.normalization(Mat.rotate_about_axis(vector_base_line, angle_limit_degs, correction_axis))
    else:
        return Mat.normalization(vector_to_limit)


def create_rotation_matrix(reference_direction):
    if reference_direction[1] == 1.0:
        reference_direction[1] -= 0.001
        reference_direction = Mat.normalization(reference_direction)

    m20 = reference_direction[0]
    m21 = reference_direction[1]
    m22 = reference_direction[2]
    cross = np.cross(reference_direction, [0, 1, 0])
    x_dir = Mat.normalization(cross)
    m00 = x_dir[0]
    m01 = x_dir[1]
    m02 = x_dir[2]

    cross = np.cross([m00, m01, m02], [m20, m21, m22])
    y_dir = Mat.normalization(cross)

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
    unsigned_angle = Mat.angle_between_degrees(reference_vector, other_vector)
    sign_measure = Mat.dot_product(np.cross(reference_vector, other_vector), normal_vector)
    if sign_measure >= 0:
        sign = 1
    else:
        sign = -1
    return unsigned_angle * sign


def negated(vector):
    return np.dot(-1, vector)


def approximately_equal(a, b, tolerance):

    if abs(a - b) < tolerance:
        return True
    else:
        return False


def length_calc(vector):
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


# Finding the rotation rotor between outer joint and inner joint of each FABRIK iteration
def find_rotation_quaternion(outer_quaternion, inner_quaternion):
        conjucate = [outer_quaternion[0], -outer_quaternion[1], -outer_quaternion[2], -outer_quaternion[3]]
        length = math.sqrt(outer_quaternion[0] ** 2 + outer_quaternion[1] ** 2 +
                           outer_quaternion[2] ** 2 + outer_quaternion[3] ** 2)
        inverse = np.dot(conjucate, (1 / length))
        rotation = Mat.multiply_two_quaternion(inner_quaternion, inverse)
        return rotation

