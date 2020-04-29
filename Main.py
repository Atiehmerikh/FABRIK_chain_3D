from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_point import CG3dVector
import Bone as Bone
import Chain as Chain
import math


def main():
    # 4 step of information entry for your chain

    # Step1 : Specify the target position which this IK solve the chain for that
    default_target_position = [0.9, 0.8, 0.6]

    # Step2: Specify base bone of the chain
    base_bone_start_location = [0, 0, 0]
    base_bone_direction = [0, 0, 1]
    base_bone_length = 0.333

    # Step3: define base bone specification:
    # joint type,
    # constraints and axises
    # joint type may vary between
    # BALL
    # GLOBAL_HINGE
    # LOCAL_HINGE

    # for Example GLOBAL_HINGE JointType:
    base_bone_constrain_type = "GLOBAL_HINGE"
    # clockwise and counter clockwise joints angle constraint in radian
    cw_base_bone_constraint_rads = 2.8973
    cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
    acw_base_bone_constraint_rads = 2.8973
    acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
    base_bone_hinge_rotation_axis = [0, 0, 1]
    base_bone_hinge_reference_axis = [0, 0, 1]  # TODO
    # here you can specify that the base bone is fixed or can rotate(yes = 1/ No =0)
    is_base_bone_fixed = 1

    # Step4: adding consecutive bone to the chain
    # number of bones excluding base bone!

    # P2
    number_bone_2 = 2
    bone_direction_2 = [0, 0, 1]
    bone_length_2 = 0.316
    joint_type_2 = "LOCAL_HINGE"
    hinge_rotation_axis_2 = [0, 1, 0]
    hinge_constraint_reference_axis_2 = [0, 1, 1]  # TODO
    cw_rad_2 = 1.7628
    cw_deg_2 = cw_rad_2 * 180 / math.pi
    acw_rad_2 = 1.7628
    acw_deg_2 = acw_rad_2 * 180 / math.pi

    # P3
    number_bone_3 = 3
    bone_direction_3 = [1, 0, 0]
    bone_length_3 = 0.088
    joint_type_3 = "LOCAL_HINGE"
    hinge_rotation_axis_3 = [0, 0, 1]
    hinge_constraint_reference_axis_3 = [1, 0, 1]  # TODO
    cw_rad_3 = 2.8972
    cw_deg_3 = cw_rad_3 * 180 / math.pi
    acw_rad_3 = 2.8973
    acw_deg_3 = acw_rad_3 * 180 / math.pi

    # P4
    number_bone_4 = 4
    bone_direction_4 = [0, 0, -1]
    bone_length_4 = 0.088
    joint_type_4 = "LOCAL_HINGE"
    hinge_rotation_axis_4 = [0, 1, 0]
    hinge_constraint_reference_axis_4 = [0, 1, 1]  # TODO
    cw_rad_4 = -0.0698
    cw_deg_4 = cw_rad_4 * 180 / math.pi
    acw_rad_4 = 3.0718
    acw_deg_4 = acw_rad_4 * 180 / math.pi

    # P5
    number_bone_5 = 5
    bone_direction_5 = [1, 0, 0]
    bone_length_5 = 0.384
    joint_type_5 = "LOCAL_HINGE"
    hinge_rotation_axis_5 = [1, 0, 0]
    hinge_constraint_reference_axis_5 = [1, 0, 0]  # TODO
    cw_rad_5 = 2.8973
    cw_deg_5 = cw_rad_5 * 180 / math.pi
    acw_rad_5 = 2.8973
    acw_deg_5 = acw_rad_5 * 180 / math.pi

    # P6
    number_bone_6 = 6
    bone_direction_6 = [1, 0, 0]
    bone_length_6 = 0.088
    joint_type_6 = "LOCAL_HINGE"
    hinge_rotation_axis_6 = [0, 1, 0]
    hinge_constraint_reference_axis_6 = [1, 1, 0]  # TODO
    cw_rad_6 = 3.7525
    cw_deg_6 = cw_rad_6 * 180 / math.pi
    acw_rad_6 = 0.0175
    acw_deg_6 = acw_rad_6 * 180 / math.pi

    # P7
    number_bone_7 = 7
    bone_direction_7 = [1, 0, 0]
    bone_length_7 = 0.107
    joint_type_7 = "LOCAL_HINGE"
    hinge_rotation_axis_7 = [1, 0, 0]
    hinge_constraint_reference_axis_7 = [1, 0, 0]  # TODO
    cw_rad_7 = 2.8973
    cw_deg_7 = cw_rad_7 * 180 / math.pi
    acw_rad_7 = 2.8973
    acw_deg_7 = acw_rad_7 * 180 / math.pi

    ###### Solving!

    target = CG3dPoint(default_target_position[0], default_target_position[1], default_target_position[2])

    base_bone_start_location = CG3dPoint(base_bone_start_location[0], base_bone_start_location[1],
                                         base_bone_start_location[2])
    base_bone_direction = CG3dVector(base_bone_direction[0], base_bone_direction[1],
                                     base_bone_direction[2])

    m_bone = Bone.Bone3D(base_bone_start_location, base_bone_direction, base_bone_length)

    # create a new chain
    m_chain = Chain.Chain3d(is_base_bone_fixed)
    # adding the new base bone to the chain
    m_chain.add_bone(m_bone)

    base_bone_hinge_rotation_axis = CG3dVector(base_bone_hinge_rotation_axis[0],
                                               base_bone_hinge_rotation_axis[1],
                                               base_bone_hinge_rotation_axis[2])
    base_bone_hinge_reference_axis = CG3dVector(base_bone_hinge_reference_axis[0],
                                                base_bone_hinge_reference_axis[1],
                                                base_bone_hinge_reference_axis[2])
    m_chain.set_global_hinged_base_bone(base_bone_hinge_rotation_axis, cw_base_bone_constraint_degs,
                                        acw_base_bone_constraint_degs, base_bone_hinge_reference_axis)

    bone_direction_2 = CG3dVector(bone_direction_2[0], bone_direction_2[1], bone_direction_2[2])
    bone_direction_3 = CG3dVector(bone_direction_3[0], bone_direction_3[1], bone_direction_3[2])
    bone_direction_4 = CG3dVector(bone_direction_4[0], bone_direction_4[1], bone_direction_4[2])
    bone_direction_5 = CG3dVector(bone_direction_5[0], bone_direction_5[1], bone_direction_5[2])
    bone_direction_6 = CG3dVector(bone_direction_6[0], bone_direction_6[1], bone_direction_6[2])
    bone_direction_7 = CG3dVector(bone_direction_7[0], bone_direction_7[1], bone_direction_7[2])

    hinge_rotation_axis_2 = CG3dVector(hinge_rotation_axis_2[0], hinge_rotation_axis_2[1], hinge_rotation_axis_2[2])
    hinge_rotation_axis_3 = CG3dVector(hinge_rotation_axis_3[0], hinge_rotation_axis_3[1], hinge_rotation_axis_3[2])
    hinge_rotation_axis_4 = CG3dVector(hinge_rotation_axis_4[0], hinge_rotation_axis_4[1], hinge_rotation_axis_4[2])
    hinge_rotation_axis_5 = CG3dVector(hinge_rotation_axis_5[0], hinge_rotation_axis_5[1], hinge_rotation_axis_5[2])
    hinge_rotation_axis_6 = CG3dVector(hinge_rotation_axis_6[0], hinge_rotation_axis_6[1], hinge_rotation_axis_6[2])
    hinge_rotation_axis_7 = CG3dVector(hinge_rotation_axis_7[0], hinge_rotation_axis_7[1], hinge_rotation_axis_7[2])

    hinge_constraint_reference_axis_2 = CG3dVector(hinge_constraint_reference_axis_2[0],
                                                   hinge_constraint_reference_axis_2[1],
                                                   hinge_constraint_reference_axis_2[2])
    hinge_constraint_reference_axis_3 = CG3dVector(hinge_constraint_reference_axis_3[0],
                                                   hinge_constraint_reference_axis_3[1],
                                                   hinge_constraint_reference_axis_3[2])
    hinge_constraint_reference_axis_4 = CG3dVector(hinge_constraint_reference_axis_4[0],
                                                   hinge_constraint_reference_axis_4[1],
                                                   hinge_constraint_reference_axis_4[2])
    hinge_constraint_reference_axis_5 = CG3dVector(hinge_constraint_reference_axis_5[0],
                                                   hinge_constraint_reference_axis_5[1],
                                                   hinge_constraint_reference_axis_5[2])
    hinge_constraint_reference_axis_6 = CG3dVector(hinge_constraint_reference_axis_6[0],
                                                   hinge_constraint_reference_axis_6[1],
                                                   hinge_constraint_reference_axis_6[2])
    hinge_constraint_reference_axis_7 = CG3dVector(hinge_constraint_reference_axis_7[0],
                                                   hinge_constraint_reference_axis_7[1],
                                                   hinge_constraint_reference_axis_7[2])

    # for i in range(1, number_bone + 1):
    m_chain.add_consecutive_hinged_bone(bone_direction_2, bone_length_2, joint_type_2, hinge_rotation_axis_2, cw_deg_2,
                                        acw_deg_2, hinge_constraint_reference_axis_2)
    m_chain.add_consecutive_hinged_bone(bone_direction_3, bone_length_3, joint_type_3, hinge_rotation_axis_3, cw_deg_3,
                                        acw_deg_3, hinge_constraint_reference_axis_3)
    m_chain.add_consecutive_hinged_bone(bone_direction_4, bone_length_4, joint_type_4, hinge_rotation_axis_4, cw_deg_4,
                                        acw_deg_4, hinge_constraint_reference_axis_4)
    m_chain.add_consecutive_hinged_bone(bone_direction_5, bone_length_5, joint_type_5, hinge_rotation_axis_5, cw_deg_5,
                                        acw_deg_5, hinge_constraint_reference_axis_5)
    m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6, joint_type_6, hinge_rotation_axis_6, cw_deg_6,
                                        acw_deg_6, hinge_constraint_reference_axis_6)
    m_chain.add_consecutive_hinged_bone(bone_direction_7, bone_length_7, joint_type_7, hinge_rotation_axis_7, cw_deg_7,
                                        acw_deg_7, hinge_constraint_reference_axis_7)

    m_chain.set_target(target)
    m_chain.solve_fabrik_ik()


if __name__ == "__main__":
    main()
