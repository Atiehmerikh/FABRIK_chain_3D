import Bone as Bone
import Chain as Chain
import math


def main():
    # 4 step of information entry for your chain

    # Step1 : Specify the target position which this IK solve the chain for that
    # default_target_position = [0.5, 0.5, 0.5]
    default_target_position = [0.21700440072005056, 0.21700440072005023, 0.5902820523028393]
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

    #  bone direction: is bone direction in global axis
    # bone length: length of bone
    # is_bone_i_fixed: if there is a constraint to fix the bone and no rotation is allowed for this bone enter "1"
    #  otherwise: "0"
    # hinge_rotation_Axis: The unit vector axis about which a hinged joint may rotate
    # cw_rad_i : The clockwise constraint angle in radian.
    # acw_rad_i: The anti-clockwise constraint angle in degrees.
    # hinge_constraint_reference_axis_i : The initial axis around the HingeRotationAxis
    # which we will enforce rotational constraints For a hinged joint, this is the axis used as
    # a point of reference for rotation (it is NOT the axis about which the hinge rotates)

    # P2
    number_bone_2 = 2
    bone_direction_2 = [0, 0, 1]
    bone_length_2 = 0.316
    is_bone_2_fixed = 0
    joint_type_2 = "LOCAL_HINGE"
    hinge_rotation_axis_2 = [0, -1, 0]
    hinge_constraint_reference_axis_2 = [0, 1, 0]  # TODO
    cw_rad_2 = 1.7628
    cw_deg_2 = cw_rad_2 * 180 / math.pi
    acw_rad_2 = 1.7628
    acw_deg_2 = acw_rad_2 * 180 / math.pi

    # P3
    number_bone_3 = 3
    bone_direction_3 = [0, 0, 1]
    bone_length_3 = 0.044
    is_bone_3_fixed = 1
    joint_type_3 = "LOCAL_HINGE"
    hinge_rotation_axis_3 = [1, 0, 0]
    hinge_constraint_reference_axis_3 = [1, 0, 0]  # TODO
    cw_rad_3 = 2.8972
    cw_deg_3 = cw_rad_3 * 180 / math.pi
    acw_rad_3 = 2.8973
    acw_deg_3 = acw_rad_3 * 180 / math.pi

    # # P4
    number_bone_4 = 4
    bone_direction_4 = [1, 0, 0]
    bone_length_4 = 0.044
    is_bone_4_fixed = 1
    joint_type_4 = "LOCAL_HINGE"
    hinge_rotation_axis_4 = [0, -1, 0]
    hinge_constraint_reference_axis_4 = [0, 1, 0]  # TODO
    cw_rad_4 = -0.0698
    cw_deg_4 = cw_rad_4 * 180 / math.pi
    acw_rad_4 = 3.0718

    acw_deg_4 = acw_rad_4 * 180 / math.pi

    # P5
    number_bone_5 = 5
    bone_direction_5 = [1, 0, 0]
    bone_length_5 = 0.384
    is_bone_5_fixed = 0
    joint_type_5 = "LOCAL_HINGE"
    hinge_rotation_axis_5 = [-1, 0, 0]
    hinge_constraint_reference_axis_5 = [1, 0, 0]  # TODO
    cw_rad_5 = 2.8973
    cw_deg_5 = cw_rad_5 * 180 / math.pi
    acw_rad_5 = 2.8973
    acw_deg_5 = acw_rad_5 * 180 / math.pi

    # P6
    number_bone_6 = 6
    bone_direction_6 = [1, 0, 0]
    bone_length_6 = 0.088
    is_bone_6_fixed = 1
    joint_type_6 = "LOCAL_HINGE"
    hinge_rotation_axis_6 = [0, -1, 0]
    hinge_constraint_reference_axis_6 = [0, 1, 0]  # TODO
    cw_rad_6 = 3.7525
    cw_deg_6 = cw_rad_6 * 180 / math.pi
    acw_rad_6 = 0.0175
    acw_deg_6 = acw_rad_6 * 180 / math.pi

    # P7
    number_bone_7 = 7
    bone_direction_7 = [0, 0, -1]
    bone_length_7 = 0.107
    is_bone_7_fixed = 0
    joint_type_7 = "LOCAL_HINGE"
    hinge_rotation_axis_7 = [-1, 0, 0]
    hinge_constraint_reference_axis_7 = [1, 0, 0]  # TODO
    cw_rad_7 = 2.8973
    cw_deg_7 = cw_rad_7 * 180 / math.pi
    acw_rad_7 = 2.8973
    acw_deg_7 = acw_rad_7 * 180 / math.pi

    ###### Solving!

    scale_direction = [i * base_bone_length for i in base_bone_direction]
    base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
    m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction, base_bone_length,
                         is_base_bone_fixed)

    # create a new chain
    m_chain = Chain.Chain3d(is_base_bone_fixed)
    # adding the new base bone to the chain
    m_chain.add_bone(m_bone)

    m_chain.set_global_hinged_base_bone(base_bone_hinge_rotation_axis, cw_base_bone_constraint_degs,
                                        acw_base_bone_constraint_degs, base_bone_hinge_reference_axis)

    # for i in range(1, number_bone + 1):
    m_chain.add_consecutive_hinged_bone(bone_direction_2, bone_length_2, joint_type_2, hinge_rotation_axis_2, cw_deg_2,
                                        acw_deg_2, hinge_constraint_reference_axis_2, is_bone_2_fixed)
    m_chain.add_consecutive_hinged_bone(bone_direction_3, bone_length_3, joint_type_3, hinge_rotation_axis_3, cw_deg_3,
                                        acw_deg_3, hinge_constraint_reference_axis_3, is_bone_3_fixed)
    m_chain.add_consecutive_hinged_bone(bone_direction_4, bone_length_4, joint_type_4, hinge_rotation_axis_4, cw_deg_4,
                                        acw_deg_4, hinge_constraint_reference_axis_4, is_bone_4_fixed)
    m_chain.add_consecutive_hinged_bone(bone_direction_5, bone_length_5, joint_type_5, hinge_rotation_axis_5, cw_deg_5,
                                        acw_deg_5, hinge_constraint_reference_axis_5, is_bone_5_fixed)
    m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6, joint_type_6, hinge_rotation_axis_6, cw_deg_6,
                                        acw_deg_6, hinge_constraint_reference_axis_6, is_bone_6_fixed)
    m_chain.add_consecutive_hinged_bone(bone_direction_7, bone_length_7, joint_type_7, hinge_rotation_axis_7, cw_deg_7,
                                        acw_deg_7, hinge_constraint_reference_axis_7, is_bone_7_fixed)

    m_chain.set_target(default_target_position)
    m_chain.solve_fabrik_ik()


if __name__ == "__main__":
    main()