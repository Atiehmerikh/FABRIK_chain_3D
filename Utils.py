from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_point import CG3dVector
import math
import Mat



def get_angle_between_degs(v1, v2):
    v1 = CG3dVector(v1[0], v1[1], v1[2])
    v2 = CG3dVector(v2[0], v2[1], v2[2])
    result = math.acos(v1 * v2 / (v1.length() * v2.length())) * 180 / math.pi
    return result


def get_distance_between(p1, p2):
    p1 = CG3dPoint(p1[0], p1[1], p1[2])
    p2 = CG3dPoint(p2[0], p2[1], p2[2])
    result = (p2 .sub(p1)).length()
    return result


def normalization(vector):
    vector = CG3dVector(vector[0], vector[1], vector[2])
    l = math.sqrt(vector[0]**2+ vector[1]**2 + vector[2]**2)
    if l == 0:
        raise Exception("target is too close")
        return
    else:
        normal_vector = CG3dVector(vector[0]/l, vector[1]/l, vector[2]/l)
        return normal_vector


def project_on_to_plane(vector, plane_normal):
    vector = CG3dVector(vector[0], vector[1], vector[2])
    plane_normal = CG3dVector(plane_normal[0], plane_normal[1], plane_normal[2])
    if plane_normal.length() < 0:
        raise Exception("Plane normal can not be zero")
    dot = vector * plane_normal
    scale = CG3dVector(plane_normal[0]*dot, plane_normal[1]*dot, plane_normal[2]*dot)
    result = vector.sub(scale)
    return result


def gen_perpendicular_vector_quick(vector):
    vector = CG3dVector(vector[0], vector[1], vector[2])
    if abs(vector[1]) < 0.99:
        perp = CG3dVector(vector[2], 0, vector[0])
    else:
        perp = CG3dVector(vector[2], 0, -vector[1])

    return perp.normalize()


def get_angle_limited_uv(vector_to_limit, vector_base_line, angle_limit_degs):
    vector_to_limit = CG3dVector(vector_to_limit[0], vector_to_limit[1], vector_to_limit[2])
    vector_base_line = CG3dVector(vector_base_line[0], vector_base_line[1], vector_base_line[2])
    angle_between = math.acos(vector_to_limit * vector_base_line / (vector_to_limit.length() * vector_base_line.length())) * 180 / math.pi
    if angle_between > angle_limit_degs:
        correction_axis = (vector_to_limit ^ vector_base_line).normalization()
        return Mat.rotate_about_axis(vector_base_line, angle_limit_degs, correction_axis).normalization()
    else:
        return vector_to_limit.normalize()


def create_rotation_matrix(reference_direction):
    reference_direction = CG3dVector(reference_direction[0], reference_direction[1], reference_direction[2])
    if reference_direction[1] == 1.0:
        reference_direction[1] -= 0.0001
        reference_direction.normalize()

    m20 = reference_direction[0]
    m21 = reference_direction[1]
    m22 = reference_direction[2]
    cross = reference_direction ^ CG3dVector(0, 1, 0)
    l = math.sqrt(cross[0]**2+cross[1]**2+cross[2]**2)
    x_dir = CG3dVector(cross[0]/l,cross[1]/l,cross[2]/l)
    m00 = x_dir[0]
    m01 = x_dir[1]
    m02 = x_dir[2]

    cross = CG3dVector(m00, m01, m02)  ^ CG3dVector(m20, m21, m22)
    l = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    y_dir = CG3dVector(cross[0] / l, cross[1] / l, cross[2] / l)

    m10 = y_dir[0]
    m11 = y_dir[1]
    m12 = y_dir[2]

    rotation_matrix = [[m00, m01, m02],
                       [m10, m11, m12],
                       [m20, m21, m22]]
    return rotation_matrix


def times(matrix, vector):
    vector = CG3dVector(vector[0], vector[1], vector[2])
    return CG3dVector(
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2])


def get_signed_angle_between_degs(reference_vector, other_vector, normal_vector):
    reference_vector = CG3dVector(reference_vector[0], reference_vector[1], reference_vector[2])
    other_vector = CG3dVector(other_vector[0], other_vector[1], other_vector[2])
    normal_vector = CG3dVector(normal_vector[0], normal_vector[1], normal_vector[2])
    unsigned_angle = math.acos(reference_vector * other_vector / (reference_vector.length() * other_vector.length())) * 180 / math.pi
    sign_measure = (reference_vector^other_vector)*normal_vector
    if sign_measure>=0:
        sign = 1
    else:
        sign = -1
    return unsigned_angle * sign


def negated(vector):
    vector=CG3dVector(vector[0], vector[1], vector[2])
    return CG3dVector(-vector[0],-vector[1],-vector[2])


def approximately_equal(a,b,tolerance):
    if abs(a-b)<tolerance:
        return 1
    else: return 0





