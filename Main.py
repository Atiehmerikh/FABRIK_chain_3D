import Bone as Bone
import Chain as Chain
import math
import Utils as Util


def main():

    # This is an example of using this code for solving inverse kinematic of FRANKA robot

    # Step 1 : specify the target position and orientation(in quaternion)
    # default_target_position = [0.21700440072005056, 0.31700440072005023, 0.55902820523028393]
    # default_target_position =[0.0994258838609295, -0.09942588386092957, 1.0867820523028393]
    default_target_position = [0.2, 0.2, 0.5]
    # default_target_orientation = [0.707,0.707,-0.707,0]
    default_target_orientation = [0,1,0,0,0]
    # default_target_position = [0.0, 0.2, 0.5]
    # default_target_orientation = [0.707, 0.707, -0.707, 0]



    # Define the specification of Base-bone in this case it only twist and rotate around itself.
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

    # Define the specification of consecutive bones in this case they are two group the one
    # rotate around themselves(bone 3, 5 7)
    # and the one working as a hinge (bone 2,4,6)

    # Bone number 2
    bone_direction_2 = [0, 0, 1]
    bone_length_2 = 0.316
    joint_type_2 = "LOCAL_HINGE"
    hinge_rotation_axis_2 = [0, 1, 0]
    hinge_constraint_reference_axis_2 =Util.Utils().gen_perpendicular_vector_quick(hinge_rotation_axis_2)
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
    hinge_constraint_reference_axis_4 = [0,0,1]
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
    hinge_constraint_reference_axis_6 = [0,0,1]
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


if __name__ == "__main__":
    main()


