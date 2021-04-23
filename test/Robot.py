from fabrik_chain_3d import Chain as Chain, Bone as Bone, Utils as Util, FABRIK as fabrik
import math
import sys
import fabrik_chain_3d.RobotVisualization as draw_chain

sys.path.append('..')


def Franka_robot_definition(default_target_position, default_target_orientation):
    # This is an example of using this code for solving inverse kinematic of FRANKA robot

    # Step 1: Define the specification of Base-bone: In this case it only twist and rotate around itself.
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
    base_bone_orientation = [1, 0, 0, 0]  # this means no orientational frame rotation happened from global coordinate.

    # Define the specification of consecutive bones in this case they are two group the one
    # rotate around themselves(bone 3, 5 7)
    # and the one working as a hinge (bone 2,4,6)

    # Shoulder:
    # Bone number 2
    bone_direction_2 = [0, 0, 1]
    bone_length_2 = 0.316
    joint_type_2 = "LOCAL_HINGE"
    hinge_rotation_axis_2 = [0, 1, 0]
    hinge_constraint_reference_axis_2 = Util.Utils().gen_perpendicular_vector_quick(hinge_rotation_axis_2)
    cw_rad_2 = 1.7628
    cw_deg_2 = cw_rad_2 * 180 / math.pi
    acw_rad_2 = 1.7628
    acw_deg_2 = acw_rad_2 * 180 / math.pi
    bone_2_orientation = [1, 0, 0, 0]
    is_bone_2_fixed = 0

    # Bone number 3
    bone_direction_3 = [1, 0, 0]
    bone_length_3 = 0.088
    is_bone_3_fixed = 0
    joint_type_3 = "twist_only"
    hinge_rotation_axis_3 = [0, 0, 1]
    hinge_constraint_reference_axis_3 = [0, 0, 1]
    cw_rad_3 = 2.8972
    cw_deg_3 = cw_rad_3 * 180 / math.pi
    acw_rad_3 = 2.8973
    acw_deg_3 = acw_rad_3 * 180 / math.pi
    bone_3_orientation = [1, 0, 0, 0]

    # elbow
    # Bone number 4
    bone_direction_4 = [0, 0, 1]
    bone_length_4 = 0.088
    joint_type_4 = "LOCAL_HINGE"
    hinge_rotation_axis_4 = [0, 1, 0]
    hinge_constraint_reference_axis_4 = [0, 0, 1]
    cw_rad_4 = 3.0718
    cw_deg_4 = cw_rad_4 * 180 / math.pi
    acw_rad_4 = 0.0698
    acw_deg_4 = acw_rad_4 * 180 / math.pi
    bone_4_orientation = [1, 0, 0, 0]
    is_bone_4_fixed = 0

    # Bone number 5
    bone_direction_5 = [0, 0, 1]
    bone_length_5 = 0.384
    is_bone_5_fixed = 0
    joint_type_5 = "twist_only"
    hinge_rotation_axis_5 = [0, 0, 1]
    hinge_constraint_reference_axis_5 = [0, 0, 1]
    cw_rad_5 = 2.8973
    cw_deg_5 = cw_rad_5 * 180 / math.pi
    acw_rad_5 = 2.8973
    acw_deg_5 = acw_rad_5 * 180 / math.pi
    bone_5_orientation = [1, 0, 0, 0]

    # wrist
    # Bone number 6
    bone_direction_6 = [1, 0, 0]
    bone_length_6 = 0.088
    joint_type_6 = "LOCAL_HINGE"
    hinge_rotation_axis_6 = [0, 1, 0]
    hinge_constraint_reference_axis_6 = [0, 0, 1]
    cw_rad_6 = 0.0175  #
    cw_deg_6 = cw_rad_6 * 180 / math.pi
    acw_rad_6 = 3.7525  #
    acw_deg_6 = acw_rad_6 * 180 / math.pi
    bone_6_orientation = [0.707, 0, -0.707, 0]
    is_bone_6_fixed = 0

    # Bone number 7
    bone_direction_7 = [0, 0, -1]
    bone_length_7 = 0.107
    is_bone_7_fixed = 0
    joint_type_7 = "twist_only"
    hinge_rotation_axis_7 = [0, 0, 1]
    hinge_constraint_reference_axis_7 = [0, 0, 1]
    cw_rad_7 = 2.8973
    cw_deg_7 = cw_rad_7 * 180 / math.pi
    acw_rad_7 = 2.8973
    acw_deg_7 = acw_rad_7 * 180 / math.pi
    bone_7_orientation = [0.707, 0, -0.707, 0]

    ###### Make the Robot Chain

    # The FRANKA consist of four main part that in each part there is a joint responsible for hinge duties and
    # a consecutive bone responsible for twisting In below these four part being made
    # by the above information about joints and bones
    # the First part:review create a chain by defining one bone that is fixed in its place and only able to twist(base bone)

    is_base_bone_fixed = 0
    m_chain = Chain.Chain3d(is_base_bone_fixed, base_address="./output")
    # scale_direction_base = [i * (base_bone_length) for i in base_bone_direction]
    # base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction_base)]
    # m_bone = Bone.Bone3D(base_bone_start_location,base_bone_end_location,base_bone_direction,
    #                      base_bone_length,is_base_bone_fixed,base_bone_orientation)
    #
    # m_chain.add_bone(m_bone)

    # Defining second part that consist of bone 2(able to work as a local hinge) and bone 3 that only
    # rotate around itself and responsible for rotations.
    scale_direction = [i * base_bone_length for i in base_bone_direction]
    bone_2_start_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
    scale_direction = [i * (bone_length_2 + bone_length_3) for i in bone_direction_2]
    bone_2_end_location = [x + y for x, y in zip(bone_2_start_location, scale_direction)]
    m_bone = Bone.Bone3D(bone_2_start_location, bone_2_end_location, bone_direction_2, bone_length_2 + bone_length_3,
                         is_bone_2_fixed, bone_2_orientation)
    m_chain.add_bone(m_bone)
    m_chain.set_rotor_base_bone_constraint("BALL", base_bone_rotation_axis, cw_deg_2)

    # Third part belongs to bone 4(able to work as a local hinge) and bone 5 that only
    # rotate around itself and responsible for twists.
    m_chain.add_consecutive_hinged_bone(bone_direction_4, bone_length_4 + bone_length_5, joint_type_4,
                                        hinge_rotation_axis_4, cw_deg_4,
                                        acw_deg_4, hinge_constraint_reference_axis_4, is_bone_4_fixed,
                                        bone_4_orientation)

    # Fourth part belongs to bone 6(able to work as a local hinge) and bone 7 that only
    # rotate around itself and responsible for twists.
    m_chain.add_consecutive_hinged_bone(bone_direction_6, bone_length_6 + bone_length_7, joint_type_6,
                                        hinge_rotation_axis_6, cw_deg_6,
                                        acw_deg_6, hinge_constraint_reference_axis_6, is_bone_6_fixed,
                                        bone_6_orientation)

    # In this part the target is set for the chain and whole chain is going to be solved
    m_chain.set_target(default_target_position, default_target_orientation)
    return m_chain


def FRANKA_base_bone():
    base_bone_start_location = [0, 0, 0]
    base_bone_direction = [0, 0, 1]
    base_bone_length = 0.333
    cw_base_bone_constraint_rads = 2.8973
    cw_base_bone_constraint_degs = cw_base_bone_constraint_rads * 180 / math.pi
    acw_base_bone_constraint_rads = 2.8973
    acw_base_bone_constraint_degs = acw_base_bone_constraint_rads * 180 / math.pi
    base_bone_orientation = [1, 0, 0, 0]

    scale_direction = [i * base_bone_length for i in base_bone_direction]
    base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction)]
    m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction,
                         base_bone_length,
                         1, base_bone_orientation)

    return m_bone


def solve_fabrik_for_Robot(base_bone, chain, target_position, target_orientation):
    dist_base_to_target = Util.Utils().get_distance_between(chain.get_bone(0).get_start_point_position(),
                                                            target_position)
    total_chain_length = 0
    for i in range(0, chain.get_chain_length()):
        total_chain_length += chain.get_bone(i).get_length()
    if dist_base_to_target <= total_chain_length:
        if chain.get_chain_length() == 0:
            raise Exception("It makes no sense to solve an IK chain with zero bones.")

        dist_to_target = Util.Utils().get_distance_between(
            chain.get_bone(chain.get_chain_length() - 1).get_end_point_position(), target_position)

        counter = 0
        m_FABRIK = fabrik.FABRIK(chain.get_chain_length(), target_position,
                                 target_orientation
                                 , chain.get_bone(0).is_fix_bone(), chain.get_base_bone_constraint_uv(),
                                 chain.get_base_location())
        while dist_to_target > chain.get_solve_distance_threshold() and counter < 10000:
            chain = m_FABRIK.forward(chain)
            chain = m_FABRIK.backward(chain)

            dist_to_target = Util.Utils().get_distance_between(
                chain.get_bone(chain.get_chain_length() - 1).get_end_point_position(),
                target_position)
            counter += 1

        # To add the fixed base bone here!! from left of list of bones
        chain.add_bone(base_bone)
        # The new bone is added from Left to the chains of bones so needs reordering
        new_chain = [chain.get_chain()[i] for i in [3,0,1,2]]
        m_draw = draw_chain.RobotVisualization(target_position, target_orientation, new_chain, m_FABRIK.get_deg(),
                                               m_FABRIK.get_rotations())
        m_draw.draw_chain()

    else:
        print("Target is so far! can't be reached")
        return


if __name__ == "__main__":
    # x = float(sys.argv[1])
    # y = float(sys.argv[2])
    # z = float(sys.argv[3])
    # default_target_position = [x, y, z]
    # default_target_position = [0.3,0.2,0.5]
    default_target_position = [0.28, -0.199904, 0.6]
    # default_target_position = [0.41, 0.09, 0.82]

    # default_target_orientation = [0, 1, 0, 0, 0]
    # rotation_matrix = [[1,-0.000308427, 0.000562747],
    #                    [0.000308962,1,-0.000950381],
    #                    [-0.000562454, 0.000950554,0.999999]]
    rotation_matrix = [[0.998752, 0.0479043, -0.0141034],
                       [-0.0126917, -0.029647, -0.99948],
                       [-0.0482975, 0.998412, -0.029002]]

    default_target_orientation = Util.Utils().quaternion_from_rotation_matrix(rotation_matrix)

    chain = Franka_robot_definition(default_target_position, default_target_orientation)

    base_bone = FRANKA_base_bone()
    solve_fabrik_for_Robot(base_bone, chain, default_target_position, default_target_orientation)


