from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_point import CG3dVector
import math
import Utils as Util


def make_point(p):
    p = CG3dPoint(p[0], p[1], p[2])
    return p


def make_vector(v):
    v = CG3dVector(v[0], v[1], v[2])
    return v


def get_angle_between_degs(v1, v2):
    v1 = Util.make_vector(v1)
    v2 = Util.make_vector(v2)
    result = math.acos(v1 * v2 / (v1.length() * v2.length())) * 180 / math.pi
    return result


def get_distance_between(p1, p2):
    p1 = Util.make_point(p1)
    p2 = Util.make_point(p2)
    result = (p2 - p1).length()
    return result


def normalize(vector):
    vector = Util.make_vector(vector)
    return vector.normalize()


def project_on_to_plane(vector, plane_normal):
    vector = Util.make_vector(vector)
    plane_normal = Util.make_vector(plane_normal)
    if plane_normal.length() < 0:
        raise Exception("Plane normal can not be zero")

    result = vector - (vector * plane_normal) * plane_normal
    return result


def gen_perpendicular_vector_quick(vector):
    vector = Util.make_vector(vector)
    if abs(vector[1]) < 0.99:
        perp = CG3dVector(vector[2], 0, vector[0])
    else:
        perp = CG3dVector(0, vector[2], 0, -vector[1])

    return Util.normalize(perp)


def rotate_about_axis(source, angle_degs, rotation_axis):
    rotation_axis = Util.make_vector(rotation_axis)
    source = Util.make_vector(source)
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


def get_angle_limited_uv(vector_to_limit, vector_base_line, angle_limit_degs):
    vector_to_limit = Util.make_vector(vector_to_limit)
    vector_base_line = Util.make_vector(vector_base_line)
    angle_between = Util.get_angle_between_degs(vector_to_limit, vector_base_line)
    if angle_between > angle_limit_degs:
        correction_axis = (vector_to_limit ^ vector_base_line).normalize()
        return Util.rotate_about_axis(vector_base_line, angle_limit_degs, correction_axis).normalize()
    else:
        return vector_to_limit.normalize()


def create_rotation_matrix(reference_direction):
    reference_direction = Util.make_vector(reference_direction)
    if reference_direction[1] == 1.0:
        reference_direction[1] -= 0.0001
        reference_direction.normalize()

    m20 = reference_direction[0]
    m21 = reference_direction[1]
    m22 = reference_direction[2]
    x_dir = (reference_direction ^ Util.make_vector([0, 1, 0])).normalize()
    m00 = x_dir[0]
    m01 = x_dir[1]
    m02 = x_dir[2]

    y_dir = (Util.make_vector([m00, m01, m02]) ^ Util.make_vector([m20, m21, m22])).normalize()

    m10 = y_dir[0]
    m11 = y_dir[1]
    m12 = y_dir[2]

    rotation_matrix = [[m00, m01, m02],
                       [m10, m11, m12],
                       [m20, m21, m22]]
    return rotation_matrix


def times(matrix, vector):
    vector = Util.make_vector(vector)
    return CG3dVector(
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2])


def get_signed_angle_between_degs(reference_vector, other_vector, normal_vector):
    reference_vector = Util.make_vector(reference_vector)
    other_vector = Util.make_vector(other_vector)
    normal_vector = Util.make_vector(normal_vector)
    unsigned_angle = Util.get_angle_between_degs(reference_vector,other_vector)
    sign_measure = (reference_vector^other_vector)*normal_vector
    if sign_measure>=0:
        sign = 1
    else:
        sign = -1
    return unsigned_angle * sign