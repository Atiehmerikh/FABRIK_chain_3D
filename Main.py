import Bone as Bone
import Chain as Chain
import math
import Utils as Util


def main():
    # default_target_position = [0.5,0.5,0]
    default_target_position = [0.21700440072005056, 0.21700440072005023, 0.55902820523028393]
    # default_target_position =[0.0994258838609295, -0.09942588386092957, 1.0867820523028393]
    default_target_orientation = [0.707,0.707,-0.707,0]

    # Bone number 1 (Base bone)
    base_bone_start_location = [0, 0, 0]
    base_bone_direction = [0, 0, 1]
    base_bone_length = 0.333
    joint_type_1 = "twist_only"
    base_bone_rotation_axis = [0, 0, 1]
    cw_base_bone_constraint_rads = 2.8973
    cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
    acw_base_bone_constraint_rads = 2.8973
    acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
    base_bone_orientation = [1, 0, 0, 0] # this means no orientational frame rotation happened from global coordinate.

    # Bone number 2
    bone_direction_2 = [0, 0, 1]
    bone_length_2 = 0.316
    joint_type_2 = "LOCAL_HINGE"
    hinge_rotation_axis_2 = [0, 1, 0]
    hinge_constraint_reference_axis_2 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_2)
    cw_rad_2 = 1.7628
    cw_deg_2 = cw_rad_2 * 180 / math.pi
    acw_rad_2 = 1.7628
    acw_deg_2 = acw_rad_2 * 180 / math.pi
    bone_2_orientation = [1, 0, 0, 0]
    is_bone_2_fixed =0

    # Bone number 3
    bone_direction_3 = [1, 0, 0]
    bone_length_3 = 0.088
    is_bone_3_fixed = 0
    joint_type_3 = "twist_only"
    hinge_rotation_axis_3 = [0, 0, 1]
    hinge_constraint_reference_axis_3 = [0,0,1]
    cw_rad_3 = 2.8972
    cw_deg_3 = cw_rad_3 * 180 / math.pi
    acw_rad_3 = 2.8973
    acw_deg_3 = acw_rad_3 * 180 / math.pi
    bone_3_orientation = [1, 0, 0, 0]

    # Bone number 4
    bone_direction_4 = [0, 0, 1]
    bone_length_4 = 0.088
    joint_type_4 = "LOCAL_HINGE"
    hinge_rotation_axis_4 = [0, 1, 0]
    hinge_constraint_reference_axis_4 = [0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_4)  # TODO
    cw_rad_4 = 3.0718
    cw_deg_4 = cw_rad_4 * 180 / math.pi
    acw_rad_4 = 0.0698
    acw_deg_4 = acw_rad_4 * 180 / math.pi
    bone_4_orientation = [1, 0, 0, 0]
    is_bone_4_fixed =0

    # Bone number 5
    bone_direction_5 = [0, 0, 1]
    bone_length_5 = 0.384
    is_bone_5_fixed = 0
    joint_type_5 = "twist_only"
    hinge_rotation_axis_5 = [0, 0, 1]
    hinge_constraint_reference_axis_5 = [0,0,1]
    cw_rad_5 = 2.8973
    cw_deg_5 = cw_rad_5 * 180 / math.pi
    acw_rad_5 = 2.8973
    acw_deg_5 = acw_rad_5 * 180 / math.pi
    bone_5_orientation = [1, 0, 0, 0]

    # Bone number 6
    bone_direction_6 = [1, 0, 0]
    bone_length_6 = 0.088
    joint_type_6 = "LOCAL_HINGE"
    hinge_rotation_axis_6 = [0, 1, 0]
    hinge_constraint_reference_axis_6 = [0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_6)  # TODO
    cw_rad_6 = 0.0175#
    cw_deg_6 = cw_rad_6 * 180 / math.pi
    acw_rad_6 = 3.7525#
    acw_deg_6 = acw_rad_6 * 180 / math.pi
    bone_6_orientation = [0.707, 0, -0.707, 0]
    is_bone_6_fixed = 0

    # Bone number 7
    bone_direction_7 = [0, 0, -1]
    bone_length_7 = 0.107
    is_bone_7_fixed = 0
    joint_type_7 = "twist_only"
    hinge_rotation_axis_7 = [0, 0, 1]
    hinge_constraint_reference_axis_7 =[0,0,1]
    cw_rad_7 = 2.8973
    cw_deg_7 = cw_rad_7 * 180 / math.pi
    acw_rad_7 = 2.8973
    acw_deg_7 = acw_rad_7 * 180 / math.pi
    bone_7_orientation = [0.707, 0, -0.707, 0]

    ###### Solving!

    # The FRANKA consist of four main part that in each part there is a joint responsible for hinge duties and
    # a consecutive bone responsible for twisting In below these four part being made
    # by the above information about joints and bones

    # the First part: create a chain by defining one bone that is fixed in its place and only able to twist
    is_base_bone_fixed = 0
    m_chain = Chain.Chain3d(is_base_bone_fixed)

    # Defining second part that consist of bone 2(able to work as a local hinge) and bone 3 that only
    # rotate around itself and responsible for twists.
    scale_direction = [i * base_bone_length for i in base_bone_direction]
    bone_2_start_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
    scale_direction = [i * (bone_length_2+bone_length_3) for i in bone_direction_2]
    bone_2_end_location = [x + y for x, y in zip(bone_2_start_location, scale_direction)]
    m_bone = Bone.Bone3D(bone_2_start_location, bone_2_end_location, bone_direction_2, bone_length_2+bone_length_3,
                         is_bone_2_fixed,bone_2_orientation)
    m_chain.add_bone(m_bone)
    m_chain.set_rotor_base_bone_constraint("BALL",base_bone_rotation_axis,cw_deg_2)
    # Third part belongs to bone 4(able to work as a local hinge) and bone 5 that only
    # rotate around itself and responsible for twists.
    m_chain.add_consecutive_hinged_bone(bone_direction_4, bone_length_4+bone_length_5, joint_type_4,
                                        hinge_rotation_axis_4, cw_deg_4,
                                        acw_deg_4, hinge_constraint_reference_axis_4, is_bone_4_fixed,bone_4_orientation)

    # Fourth part belongs to bone 6(able to work as a local hinge) and bone 7 that only
    # rotate around itself and responsible for twists.
    m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6+bone_length_7, joint_type_6, hinge_rotation_axis_6, cw_deg_6,
                                        acw_deg_6, hinge_constraint_reference_axis_6,is_bone_6_fixed,bone_6_orientation)

    # In this part the target is set for the chain and whole chain is going to be solved
    m_chain.set_target(default_target_position,default_target_orientation)
    m_chain.solve_fabrik_ik()



# this is the one with hinges perpendicular to rotary bones

# import Bone as Bone
# import Chain as Chain
# import math
# import Utils as Util
#
# # def main():
# #     # 4 step of information entry for your chain
# #
# #     # Step1 : Specify the target position which this IK solve the chain for that
# #     # default_target_position = [0.5, 0.5, 0.5]
# #     # default_target_position = [0.21700440072005056, 0.21700440072005023, 0.5902820523028393]
# #     # default_target_position =[0.5,0.5,0]
# #     # default_target_position = [-0.040882729136984236, 0.35804755544399547, 0.5076608851454391]
# #     # default_target_position =[-0.0994258838609295, -0.09942588386092957, 1.0867820523028393]
# #     # default_target_position =[0.3330044007200504, 0.5330044007200505, 0.4736094334070591]
# #     default_target_position = [0.07566042558696057, 0.20756604255869606, 1.121]
# #     # default_target_position = [0.5,0.5,0]
# #
# #     # target orientation in quaternion
# #     default_target_orientation = [1, 0, 0, 0]
# #
# #     # Step2: Specify base bone of the chain
# #     # base_bone_start_location = [0, 0, 0]
# #     # base_bone_direction = [0, 0, 1]
# #     # base_bone_length = 0.333
# #     #
# #     #
# #     #
# #     # # for Example GLOBAL_HINGE JointType:
# #     # base_bone_constrain_type = "GLOBAL_HINGE"
# #     # # clockwise and counter clockwise joints angle constraint in radian
# #     # cw_base_bone_constraint_rads = 2.8973
# #     # cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
# #     # acw_base_bone_constraint_rads = 2.8973
# #     # acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
# #     # base_bone_hinge_rotation_axis = [0, 0, 1]
# #     # base_bone_hinge_reference_axis = Util.gen_perpendicular_vector_quick(base_bone_hinge_rotation_axis)  # TODO
# #     # # here you can specify that the base bone is fixed or can rotate(yes = 1/ No =0)
# #     # is_base_bone_fixed = 0
# #     # Step4: adding consecutive bone to the chain
# #     # p2- prime as base bone
# #     base_bone_start_location = [0, 0, 0]
# #     base_bone_direction = [0, 0, 1]
# #     base_bone_length = 0.33
# #
# #     # # for Example GLOBAL_HINGE JointType:
# #     # base_bone_constrain_type = "GLOBAL_HINGE"
# #     # # clockwise and counter clockwise joints angle constraint in radian
# #     # cw_base_bone_constraint_rads = 2.8973
# #     # cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
# #     # acw_base_bone_constraint_rads = 2.8973
# #     # acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
# #     # base_bone_hinge_rotation_axis = [0, 1, 0]
# #     # # base_bone_hinge_reference_axis = Util[0, 0, 1]  # TODO
# #     # # here you can specify that the base bone is fixed or can rotate(yes = 1/ No =0)
# #     is_base_bone_fixed = 0
# #
# #
# #
# #
# #
# #     # # P2
# #     # number_bone_2 = 2
# #     bone_direction_2 = [0, 0, 1]
# #     bone_length_2 = 0.316
# #     is_bone_2_fixed = 0
# #     joint_type_2 = "LOCAL_HINGE"
# #     hinge_rotation_axis_2 = [0, 1, 0]
# #     hinge_constraint_reference_axis_2 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_2)   # TODO
# #     cw_rad_2 =2.8972 #1.7628
# #     cw_deg_2 = cw_rad_2 * 180 / math.pi
# #     acw_rad_2 =2.8972# 1.7628
# #     acw_deg_2 = acw_rad_2 * 180 / math.pi
# #
# #     # P3
# #     number_bone_3 = 3
# #     bone_direction_3 = [0, 0, 1]
# #     bone_length_3 = 0.088
# #     is_bone_3_fixed = 0
# #     # joint_type_3 = "LOCAL_HINGE"
# #     # hinge_rotation_axis_3 = [0, 0, 1]
# #     # hinge_constraint_reference_axis_3 = [0,0,1]#TODO Util.gen_perpendicular_vector_quick(hinge_rotation_axis_3)
# #     # cw_rad_3 = 2.8972
# #     # cw_deg_3 = cw_rad_3 * 180 / math.pi
# #     # acw_rad_3 = 2.8973
# #     # acw_deg_3 = acw_rad_3 * 180 / math.pi
# #
# #     # # P4
# #     number_bone_4 = 4
# #     bone_direction_4 = [0, 0, 1]
# #     bone_length_4 = 0.088
# #     is_bone_4_fixed = 0
# #     joint_type_4 = "LOCAL_HINGE"
# #     hinge_rotation_axis_4 = [0, 1, 0]
# #     hinge_constraint_reference_axis_4 =Util.gen_perpendicular_vector_quick(hinge_rotation_axis_4)   # TODO
# #     cw_rad_4 = 3.0718
# #     cw_deg_4 = cw_rad_4 * 180 / math.pi
# #     acw_rad_4 = 0.0698
# #     acw_deg_4 = acw_rad_4 * 180 / math.pi
# #
# #     # P5
# #     number_bone_5 = 5
# #     bone_direction_5 = [0, 0, 1]
# #     bone_length_5 = 0.384
# #     is_bone_5_fixed = 0
# #     # joint_type_5 = "LOCAL_HINGE"
# #     # hinge_rotation_axis_5 = [0, 0, 1]
# #     # hinge_constraint_reference_axis_5 = [0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_5)   # TODO
# #     # cw_rad_5 = 2.8973
# #     # cw_deg_5 = cw_rad_5 * 180 / math.pi
# #     # acw_rad_5 = 2.8973
# #     # acw_deg_5 = acw_rad_5 * 180 / math.pi
# #
# #     # P6
# #     number_bone_6 = 6
# #     bone_direction_6 = [0, 0,1]
# #     bone_length_6 = 0.088
# #     is_bone_6_fixed = 0
# #     joint_type_6 = "LOCAL_HINGE"
# #     hinge_rotation_axis_6 = [0, 1, 0]
# #     hinge_constraint_reference_axis_6 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_6)   # TODO
# #     cw_rad_6 =2.8973#0.0175#2.8973#3.7525
# #     cw_deg_6 = cw_rad_6 * 180 / math.pi
# #     acw_rad_6 =3.7525#0.0175
# #     acw_deg_6 = acw_rad_6 * 180 / math.pi
# #
# #     # P7
# #     number_bone_7 = 7
# #     bone_direction_7 = [0, 0, 1]
# #     bone_length_7 = 0.107
# #     is_bone_7_fixed = 0
# #     # joint_type_7 = "LOCAL_HINGE"
# #     # hinge_rotation_axis_7 = [0, 0, 1]
# #     # hinge_constraint_reference_axis_7 =[0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_7)   # TODO
# #     # cw_rad_7 = 2.8973
# #     # cw_deg_7 = cw_rad_7 * 180 / math.pi
# #     # acw_rad_7 = 2.8973
# #     # acw_deg_7 = acw_rad_7 * 180 / math.pi
# #
# #     ###### Solving!
# #
# #     orient1 = [1,0,0,0]
# #     scale_direction = [i * base_bone_length for i in base_bone_direction]
# #     base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
# #     m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction, base_bone_length,
# #                          is_bone_3_fixed,orient1)
# #
# #     # create a new chain
# #     m_chain = Chain.Chain3d(is_bone_3_fixed)
# #     # adding the new base bone to the chain
# #     m_chain.add_bone(m_bone)
# #
# #     # 1
# #     # m_chain.set_rotor_base_bone_constraint("No",[0,0,1],180)
# #     # m_chain.set_freely_rotating_global_hinged_base_bone(base_bone_hinge_rotation_axis)
# #     # m_chain.set_global_hinged_base_bone(base_bone_hinge_rotation_axis, cw_base_bone_constraint_degs,
# #     #                                     acw_base_bone_constraint_degs, base_bone_hinge_reference_axis)
# #
# #     # 2
# #     start_loation_2 = [0, 0, 0.33 ]
# #     scale_direction = [i * (bone_length_2) for i in bone_direction_2]
# #     bone_2_end_location = [x + y for x, y in zip(start_loation_2, scale_direction)]
# #     m_bone = Bone.Bone3D(start_loation_2, bone_2_end_location, bone_direction_2, bone_length_2,
# #                          is_bone_2_fixed, orient1)
# #     m_chain.add_bone(m_bone)
# #
# #     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_2,bone_length_2,180,is_bone_2_fixed,orient1)
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_2,bone_length_2,joint_type_2,hinge_rotation_axis_6,cw_deg_6,acw_deg_6,
# #     #                                     hinge_constraint_reference_axis_6,is_bone_2_fixed,orient1)
# #     # 3
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_3,bone_length_3,0,is_bone_3_fixed,orient1)
# #
# #     # 4
# #     # m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction, base_bone_length,
# #     #                      is_bone_3_fixed, orient1)
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_4,bone_length_4,joint_type_4,hinge_rotation_axis_6,cw_deg_4,acw_deg_4,
# #     #                                     hinge_constraint_reference_axis_4,is_bone_4_fixed,orient1)
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_4,bone_length_4,180,is_bone_4_fixed,orient1)
# #
# #     # 5
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_5,bone_length_5,0,is_bone_5_fixed,orient1)
# #
# #     # 6 and 7 are end effectors bone
# #     start_loation_6 = [0,0,0.33+0.316+0.088+0.088+0.384]
# #     scale_direction = [i * (bone_length_6+bone_length_7) for i in bone_direction_6]
# #     bone_6_end_location = [x + y for x, y in zip(start_loation_6, scale_direction)]
# #     m_bone = Bone.Bone3D(start_loation_6, bone_6_end_location, bone_direction_6, bone_length_6,
# #                          is_bone_6_fixed, orient1)
# #     m_chain.add_bone(m_bone)
# #
# #     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_7,bone_length_7,0,is_bone_7_fixed,orient1)
# #
# #
# #
# #     # 6
# #     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_6,bone_length_6,180,is_bone_6_fixed,orient1)
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6, joint_type_6, hinge_rotation_axis_6, cw_deg_6,
# #     #                                     acw_deg_6,
# #     #                                     hinge_constraint_reference_axis_6, is_bone_6_fixed, orient1)
# #     # #
# #     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_7,bone_length_7,0,is_bone_7_fixed,orient1)
# #
# #     m_chain.set_target(default_target_position,default_target_orientation)
# #     m_chain.solve_fabrik_ik()
#
# # def main():
# #     # 4 step of information entry for your chain
# #
# #     # Step1 : Specify the target position which this IK solve the chain for that
# #     # default_target_position = [0.5, 0.5, 0.5]
# #     default_target_position = [0.21700440072005056, 0.21700440072005023, 0.5902820523028393]
# #     # default_target_position =[0.5,0.5,0]
# #     # default_target_position = [-0.040882729136984236, 0.35804755544399547, 0.5076608851454391]
# #     # default_target_position =[-0.20994258838609295, -0.209942588386092957, 0.0867820523028393]
# #     # default_target_position =[0.5330044007200504, 0.5330044007200505, 0.4736094334070591]
# #     # default_target_position = [0.07566042558696057, 0.20756604255869606, 1.121]
# #     # default_target_position = [0.5,0.5,0]
# #
# #     # target orientation in quaternion
# #     default_target_orientation = [1, 0, 0, 0]
# #
# #     # Step2: Specify base bone of the chain
# #     base_bone_start_location = [0, 0, 0]
# #     base_bone_direction = [0, 0, 1]
# #     base_bone_length = 0.333
# #
# #     # Step3: define base bone specification:
# #     # joint type,
# #     # constraints and axises
# #     # joint type may vary between
# #     # BALL
# #     # GLOBAL_HINGE
# #     # LOCAL_HINGE
# #
# #     # for Example GLOBAL_HINGE JointType:
# #     base_bone_constrain_type = "GLOBAL_HINGE"
# #     # clockwise and counter clockwise joints angle constraint in radian
# #     cw_base_bone_constraint_rads = 2.8973
# #     cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
# #     acw_base_bone_constraint_rads = 2.8973
# #     acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
# #     base_bone_hinge_rotation_axis = [0, 0, 1]
# #     base_bone_hinge_reference_axis = Util.gen_perpendicular_vector_quick(base_bone_hinge_rotation_axis)  # TODO
# #     # here you can specify that the base bone is fixed or can rotate(yes = 1/ No =0)
# #     is_base_bone_fixed = 1
# #     # Step4: adding consecutive bone to the chain
# #
# #     #  bone direction: is bone direction in global axis
# #     # bone length: length of bone
# #     # is_bone_i_fixed: if there is a constraint to fix the bone and no rotation is allowed for this bone enter "1"
# #     #  otherwise: "0"
# #     # hinge_rotation_Axis: The unit vector axis about which a hinged joint may rotate
# #     # cw_rad_i : The clockwise constraint angle in radian.
# #     # acw_rad_i: The anti-clockwise constraint angle in degrees.
# #     # hinge_constraint_reference_axis_i : The initial axis around the HingeRotationAxis
# #     # which we will enforce rotational constraints For a hinged joint, this is the axis used as
# #     # a point of reference for rotation (it is NOT the axis about which the hinge rotates)
# #
# #     # P2
# #     number_bone_2 = 2
# #     bone_direction_2 = [0, 0, 1]
# #     bone_length_2 = 0.316
# #     is_bone_2_fixed = 0
# #     joint_type_2 = "LOCAL_HINGE"
# #     hinge_rotation_axis_2 = [0, 1, 0]
# #     hinge_constraint_reference_axis_2 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_2)   # TODO
# #     cw_rad_2 =2.8972 #1.7628
# #     cw_deg_2 = cw_rad_2 * 180 / math.pi
# #     acw_rad_2 =2.8972# 1.7628
# #     acw_deg_2 = acw_rad_2 * 180 / math.pi
# #
# #     # P3
# #     number_bone_3 = 3
# #     bone_direction_3 = [0, 0, 1]
# #     bone_length_3 = 0.088
# #     is_bone_3_fixed = 0
# #     joint_type_3 = "LOCAL_HINGE"
# #     hinge_rotation_axis_3 = [0, 0, 1]
# #     hinge_constraint_reference_axis_3 = [0,0,1]#TODO Util.gen_perpendicular_vector_quick(hinge_rotation_axis_3)
# #     cw_rad_3 = 2.8972
# #     cw_deg_3 = cw_rad_3 * 180 / math.pi
# #     acw_rad_3 = 2.8973
# #     acw_deg_3 = acw_rad_3 * 180 / math.pi
# #
# #     # # P4
# #     number_bone_4 = 4
# #     bone_direction_4 = [0, 0, 1]
# #     bone_length_4 = 0.088
# #     is_bone_4_fixed = 0
# #     joint_type_4 = "LOCAL_HINGE"
# #     hinge_rotation_axis_4 = [0, 1, 0]
# #     hinge_constraint_reference_axis_4 =Util.gen_perpendicular_vector_quick(hinge_rotation_axis_4)   # TODO
# #     cw_rad_4 = 3.0718
# #     cw_deg_4 = cw_rad_4 * 180 / math.pi
# #     acw_rad_4 = 0.0698
# #
# #     acw_deg_4 = acw_rad_4 * 180 / math.pi
# #
# #     # P5
# #     number_bone_5 = 5
# #     bone_direction_5 = [0, 0, 1]
# #     bone_length_5 = 0.384
# #     is_bone_5_fixed = 0
# #     joint_type_5 = "LOCAL_HINGE"
# #     hinge_rotation_axis_5 = [0, 0, 1]
# #     hinge_constraint_reference_axis_5 = [0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_5)   # TODO
# #     cw_rad_5 = 2.8973
# #     cw_deg_5 = cw_rad_5 * 180 / math.pi
# #     acw_rad_5 = 2.8973
# #     acw_deg_5 = acw_rad_5 * 180 / math.pi
# #
# #     # P6
# #     number_bone_6 = 6
# #     bone_direction_6 = [0, 0,1]
# #     bone_length_6 = 0.088
# #     is_bone_6_fixed = 0
# #     joint_type_6 = "LOCAL_HINGE"
# #     hinge_rotation_axis_6 = [0, 1, 0]
# #     hinge_constraint_reference_axis_6 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_6)   # TODO
# #     cw_rad_6 = 3.7525#0.0175#
# #     cw_deg_6 = cw_rad_6 * 180 / math.pi
# #     acw_rad_6 =0.0175#3.7525#
# #     acw_deg_6 = acw_rad_6 * 180 / math.pi
# #
# #     # P7
# #     number_bone_7 = 7
# #     bone_direction_7 = [0, 0, 1]
# #     bone_length_7 = 0.107
# #     is_bone_7_fixed = 0
# #     joint_type_7 = "LOCAL_HINGE"
# #     hinge_rotation_axis_7 = [0, 0, 1]
# #     hinge_constraint_reference_axis_7 =[0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_7)   # TODO
# #     cw_rad_7 = 2.8973
# #     cw_deg_7 = cw_rad_7 * 180 / math.pi
# #     acw_rad_7 = 2.8973
# #     acw_deg_7 = acw_rad_7 * 180 / math.pi
# #
# #     ###### Solving!
# #
# #     orient1 = [1,0,0,0]
# #     scale_direction = [i * base_bone_length for i in base_bone_direction]
# #     base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
# #     m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction, base_bone_length,
# #                          is_base_bone_fixed,orient1)
# #
# #     # create a new chain
# #     m_chain = Chain.Chain3d(is_base_bone_fixed)
# #     # adding the new base bone to the chain
# #     m_chain.add_bone(m_bone)
# #
# #     m_chain.set_global_hinged_base_bone(base_bone_hinge_rotation_axis, cw_base_bone_constraint_degs,
# #                                         acw_base_bone_constraint_degs, base_bone_hinge_reference_axis)
# #
# #     # for i in range(1, number_bone + 1):
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_2, bone_length_2, joint_type_2, hinge_rotation_axis_2, cw_deg_2,
# #     #                                     acw_deg_2, hinge_constraint_reference_axis_2, is_bone_2_fixed,orient1)
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_2, bone_length_2, 180, is_bone_2_fixed, orient1)
# #
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_3, bone_length_3, joint_type_3, hinge_rotation_axis_3, cw_deg_3,
# #     #                                     acw_deg_3, hinge_constraint_reference_axis_3, is_bone_3_fixed,orient1)
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_3, bone_length_3, 0, is_bone_3_fixed, orient1)
# #
# #     # m_chain.add_consecutive_freely_rotating_hinged_bone(bone_direction_3,bone_length_3,joint_type_3,hinge_rotation_axis_3,is_bone_3_fixed,orient1)
# #     m_chain.add_consecutive_hinged_bone(bone_direction_4, bone_length_4, joint_type_4, hinge_rotation_axis_4, cw_deg_4,
# #                                         acw_deg_4, hinge_constraint_reference_axis_4, is_bone_4_fixed,orient1)
# #
# #     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_4, bone_length_4, 180, is_bone_4_fixed, orient1)
# #
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_5, bone_length_5, 0, is_bone_3_fixed, orient1)
# #
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_5, bone_length_5, joint_type_5, hinge_rotation_axis_5, cw_deg_5,
# #     #                                     acw_deg_5, hinge_constraint_reference_axis_5, is_bone_5_fixed,orient1)
# #     # m_chain.add_consecutive_freely_rotating_hinged_bone(bone_direction_5, bone_length_5, joint_type_5,
# #     #                                                     hinge_rotation_axis_5, is_bone_5_fixed, orient1)
# #     m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6, joint_type_6, hinge_rotation_axis_6, cw_deg_6,
# #                                         acw_deg_6, hinge_constraint_reference_axis_6, is_bone_6_fixed,orient1)
# #     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_6, bone_length_6, 180, is_bone_7_fixed, orient1)
# #
# #     # m_chain.add_consecutive_freely_rotating_hinged_bone(bone_direction_7, bone_length_7, joint_type_7,
# #     #                                                     hinge_rotation_axis_7, is_bone_7_fixed, orient1)
# #
# #     m_chain.add_consecutive_rotor_constrained_bone(bone_direction_7, bone_length_7, 0, is_bone_7_fixed, orient1)
# #     # m_chain.add_consecutive_hinged_bone(bone_direction_7, bone_length_7, joint_type_7, hinge_rotation_axis_7, cw_deg_7,
# #     #                                     acw_deg_7, hinge_constraint_reference_axis_7, is_bone_7_fixed,orient1)
# #
# #     m_chain.set_target(default_target_position,default_target_orientation)
# #     m_chain.solve_fabrik_ik()
#
# def main():
#     # 4 step of information entry for your chain
#
#     # Step1 : Specify the target position which this IK solve the chain for that
#     # default_target_position = [0.5, 0.5, 0.5]
#     # default_target_position = [0.21700440072005056, 0.21700440072005023, 0.5902820523028393]
#     # default_target_position =[0.5,0.5,0]
#
#     default_target_position = [-0.040882729136984236, 0.35804755544399547, 0.5076608851454391]
#     # default_target_position =[-0.20994258838609295, -0.209942588386092957, 0.0867820523028393]
#     # default_target_position =[0.5330044007200504, 0.5330044007200505, 0.4736094334070591]
#     # default_target_position = [0.07566042558696057, 0.20756604255869606, 1.121]
#     # default_target_position = [0.5,0.5,0]
#
#     # target orientation in quaternion
#     default_target_orientation = [1, 0, 0, 0]
#
#     # Step2: Specify base bone of the chain
#     base_bone_start_location = [0, 0, 0]
#     base_bone_direction = [0, 0, 1]
#     base_bone_length = 0.333
#
#     # Step3: define base bone specification:
#     # joint type,
#     # constraints and axises
#     # joint type may vary between
#     # BALL
#     # GLOBAL_HINGE
#     # LOCAL_HINGE
#
#     # for Example GLOBAL_HINGE JointType:
#     base_bone_constrain_type = "GLOBAL_HINGE"
#     # clockwise and counter clockwise joints angle constraint in radian
#     cw_base_bone_constraint_rads = 2.8973
#     cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
#     acw_base_bone_constraint_rads = 2.8973
#     acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
#     base_bone_hinge_rotation_axis = [0, 0, 1]
#     base_bone_hinge_reference_axis = Util.gen_perpendicular_vector_quick(base_bone_hinge_rotation_axis)  # TODO
#     # here you can specify that the base bone is fixed or can rotate(yes = 1/ No =0)
#     is_base_bone_fixed = 1
#     # Step4: adding consecutive bone to the chain
#
#     #  bone direction: is bone direction in global axis
#     # bone length: length of bone
#     # is_bone_i_fixed: if there is a constraint to fix the bone and no rotation is allowed for this bone enter "1"
#     #  otherwise: "0"
#     # hinge_rotation_Axis: The unit vector axis about which a hinged joint may rotate
#     # cw_rad_i : The clockwise constraint angle in radian.
#     # acw_rad_i: The anti-clockwise constraint angle in degrees.
#     # hinge_constraint_reference_axis_i : The initial axis around the HingeRotationAxis
#     # which we will enforce rotational constraints For a hinged joint, this is the axis used as
#     # a point of reference for rotation (it is NOT the axis about which the hinge rotates)
#
#     # P2
#     number_bone_2 = 2
#     bone_direction_2 = [0, 0, 1]
#     bone_length_2 = 0.316
#     is_bone_2_fixed = 0
#     joint_type_2 = "LOCAL_HINGE"
#     hinge_rotation_axis_2 = [0, 1, 0]
#     hinge_constraint_reference_axis_2 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_2)   # TODO
#     cw_rad_2 =1.7628
#     cw_deg_2 = cw_rad_2 * 180 / math.pi
#     acw_rad_2 =1.7628
#     acw_deg_2 = acw_rad_2 * 180 / math.pi
#
#     # P3
#     number_bone_3 = 3
#     bone_direction_3 = [1, 0, 0]
#     bone_length_3 = 0.088
#     is_bone_3_fixed = 0
#     joint_type_3 = "LOCAL_HINGE"
#     hinge_rotation_axis_3 = [0, 0, 1]
#     hinge_constraint_reference_axis_3 = [0,0,1]#TODO Util.gen_perpendicular_vector_quick(hinge_rotation_axis_3)
#     cw_rad_3 = 2.8972
#     cw_deg_3 = cw_rad_3 * 180 / math.pi
#     acw_rad_3 = 2.8973
#     acw_deg_3 = acw_rad_3 * 180 / math.pi
#
#     # # P4
#     number_bone_4 = 4
#     bone_direction_4 = [-1, 0, 0]
#     bone_length_4 = 0.088
#     is_bone_4_fixed = 0
#     joint_type_4 = "LOCAL_HINGE"
#     hinge_rotation_axis_4 = [0, 1, 0]
#     hinge_constraint_reference_axis_4 =Util.gen_perpendicular_vector_quick(hinge_rotation_axis_4)   # TODO
#     cw_rad_4 =-0.0698
#     cw_deg_4 = cw_rad_4 * 180 / math.pi
#     acw_rad_4 =-3.0718
#
#     acw_deg_4 = acw_rad_4 * 180 / math.pi
#
#     # P5
#     number_bone_5 = 5
#     bone_direction_5 = [0, 0, 1]
#     bone_length_5 = 0.384
#     is_bone_5_fixed = 0
#     joint_type_5 = "LOCAL_HINGE"
#     hinge_rotation_axis_5 = [0, 0, 1]
#     hinge_constraint_reference_axis_5 = [0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_5)   # TODO
#     cw_rad_5 = 2.8973
#     cw_deg_5 = cw_rad_5 * 180 / math.pi
#     acw_rad_5 = 2.8973
#     acw_deg_5 = acw_rad_5 * 180 / math.pi
#
#     # P6
#     number_bone_6 = 6
#     bone_direction_6 = [1, 0,0]
#     bone_length_6 = 0.088
#     is_bone_6_fixed = 0
#     joint_type_6 = "LOCAL_HINGE"
#     hinge_rotation_axis_6 = [0, 1, 0]
#     hinge_constraint_reference_axis_6 = Util.gen_perpendicular_vector_quick(hinge_rotation_axis_6)   # TODO
#     cw_rad_6 = 3.7525#0.0175#
#     cw_deg_6 = cw_rad_6 * 180 / math.pi
#     acw_rad_6 =0.0175#3.7525#
#     acw_deg_6 = acw_rad_6 * 180 / math.pi
#
#     # P7
#     number_bone_7 = 7
#     bone_direction_7 = [0, 0, -1]
#     bone_length_7 = 0.107
#     is_bone_7_fixed = 0
#     joint_type_7 = "LOCAL_HINGE"
#     hinge_rotation_axis_7 = [0, 0, 1]
#     hinge_constraint_reference_axis_7 =[0,0,1]#Util.gen_perpendicular_vector_quick(hinge_rotation_axis_7)   # TODO
#     cw_rad_7 = 2.8973
#     cw_deg_7 = cw_rad_7 * 180 / math.pi
#     acw_rad_7 = 2.8973
#     acw_deg_7 = acw_rad_7 * 180 / math.pi
#
#     ###### Solving!
#
#     orient1 = [1,0,0,0]
#     scale_direction = [i * base_bone_length for i in base_bone_direction]
#     base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
#     m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction, base_bone_length,
#                          is_base_bone_fixed,orient1)
#
#     # create a new chain
#     m_chain = Chain.Chain3d(is_base_bone_fixed)
#     # adding the new base bone to the chain
#     m_chain.add_bone(m_bone)
#
#     m_chain.set_global_hinged_base_bone(base_bone_hinge_rotation_axis, cw_base_bone_constraint_degs,
#                                         acw_base_bone_constraint_degs, base_bone_hinge_reference_axis)
#
#     # for i in range(1, number_bone + 1):
#     m_chain.add_consecutive_hinged_bone(bone_direction_2, bone_length_2, joint_type_2, hinge_rotation_axis_2, cw_deg_2,
#                                         acw_deg_2, hinge_constraint_reference_axis_2, is_bone_2_fixed,orient1)
#     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_2, bone_length_2, 180, is_bone_2_fixed, orient1)
#
#     m_chain.add_consecutive_hinged_bone(bone_direction_3, bone_length_3, joint_type_3, hinge_rotation_axis_3, cw_deg_3,
#                                         acw_deg_3, hinge_constraint_reference_axis_3, is_bone_3_fixed,orient1)
#     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_3, bone_length_3, 0, is_bone_3_fixed, orient1)
#
#     # m_chain.add_consecutive_freely_rotating_hinged_bone(bone_direction_3,bone_length_3,joint_type_3,hinge_rotation_axis_3,is_bone_3_fixed,orient1)
#     m_chain.add_consecutive_hinged_bone(bone_direction_4, bone_length_4, joint_type_4, hinge_rotation_axis_4, cw_deg_4,
#                                         acw_deg_4, hinge_constraint_reference_axis_4, is_bone_4_fixed,orient1)
#
#     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_4, bone_length_4, 180, is_bone_4_fixed, orient1)
#
#     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_5, bone_length_5, 0, is_bone_3_fixed, orient1)
#
#     m_chain.add_consecutive_hinged_bone(bone_direction_5, bone_length_5, joint_type_5, hinge_rotation_axis_5, cw_deg_5,
#                                         acw_deg_5, hinge_constraint_reference_axis_5, is_bone_5_fixed,orient1)
#     # m_chain.add_consecutive_freely_rotating_hinged_bone(bone_direction_5, bone_length_5, joint_type_5,
#     #                                                     hinge_rotation_axis_5, is_bone_5_fixed, orient1)
#     m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6, joint_type_6, hinge_rotation_axis_6, cw_deg_6,
#                                         acw_deg_6, hinge_constraint_reference_axis_6, is_bone_6_fixed,orient1)
#     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_6, bone_length_6, 180, is_bone_7_fixed, orient1)
#
#     # m_chain.add_consecutive_freely_rotating_hinged_bone(bone_direction_7, bone_length_7, joint_type_7,
#     #                                                     hinge_rotation_axis_7, is_bone_7_fixed, orient1)
#
#     # m_chain.add_consecutive_rotor_constrained_bone(bone_direction_7, bone_length_7, 0, is_bone_7_fixed, orient1)
#     m_chain.add_consecutive_hinged_bone(bone_direction_7, bone_length_7, joint_type_7, hinge_rotation_axis_7, cw_deg_7,
#                                         acw_deg_7, hinge_constraint_reference_axis_7, is_bone_7_fixed,orient1)
#
#     m_chain.set_target(default_target_position,default_target_orientation)
#     m_chain.solve_fabrik_ik()



if __name__ == "__main__":
    main()


