from fabrik_chain_3d import Chain as Chain, Bone as Bone, Utils as Util, FABRIK as fabrik
import math
import sys
import fabrik_chain_3d.RobotVisualization as draw_chain
sys.path.append('..')


def right_hand_chain(joints,orientation,constraints_angle, core_index, shoulder_index, elbow_index):
    # core to shoulder bone
    base_bone_start_location = joints[core_index]
    base_bone_direction = joints[shoulder_index] - joints[core_index]
    base_bone_length = math.sqrt((joints[shoulder_index][0] - joints[core_index][0]) ** 2 +
                                 (joints[shoulder_index][1] - joints[core_index][1]) ** 2 +
                                 (joints[shoulder_index][2] - joints[core_index][2]) ** 2)
    base_bone_rotation_axis = [0, 0, 1]
    base_bone_constraint_rads = constraints_angle[core_index][0]* math.pi/180
    base_bone_constraint_degs = base_bone_constraint_rads * 180 / math.pi
    base_bone_orientation = orientation[core_index]  # this means no orientational frame rotation happened from global coordinate.

    is_base_bone_fixed = 0
    m_chain = Chain.Chain3d(is_base_bone_fixed, base_address="./output")
    scale_direction_base = [i * (base_bone_length) for i in base_bone_direction]
    base_bone_end_location = [x + y for x, y in zip(base_bone_start_location, scale_direction_base)]

    m_bone = Bone.Bone3D(base_bone_start_location, base_bone_end_location, base_bone_direction,
                         base_bone_length, is_base_bone_fixed, base_bone_orientation)
    m_chain.add_bone(m_bone)
    m_chain.set_rotor_base_bone_constraint("BALL",base_bone_rotation_axis,base_bone_constraint_degs)

    # Shoulder bone:
    shoulder_bone_direction = joints[elbow_index] - joints[shoulder_index]
    shoulder_bone_length = math.sqrt((joints[elbow_index][0] - joints[shoulder_index][0]) ** 2 +
                                 (joints[elbow_index][1] - joints[shoulder_index][1]) ** 2 +
                                 (joints[elbow_index][2] - joints[shoulder_index][2]) ** 2)

    shoulder_constraint_deg= constraints_angle[shoulder_index][0]* math.pi/180
    shoulder_bone_orientation =orientation[shoulder_index]
    is_bone_shoulder_fixed = 0

    m_chain.add_consecutive_rotor_constrained_bone(shoulder_bone_direction,shoulder_bone_length,shoulder_constraint_deg,
                                                   is_bone_shoulder_fixed,shoulder_bone_orientation)

    # In this part the target is set for the chain and whole chain is going to be solved
    m_chain.set_target(default_target_position, default_target_orientation)
    # m_chain.solve_fabrik_ik()
    # FRANKA = m_chain.get_chain()
    return m_chain



if __name__ == "__main__":
    reader = input_reader.InputReader()

    manipulator = fabrik.FABRIK(reader.joints(),
                                reader.initial_joints_position(),
                                reader.orientation(),
                                reader.target_position(),
                                reader.target_orientation(),
                                reader.joints_constraints(),
                                reader.constraint_type(),
                                reader.bone_orientation_limit())

    manipulator.solve()