from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_point import CG3dVector
import Bone as Bone
import Chain as Chain
import math


def main():
    # 4 step of information entry for your chain

    # Step1 : Specify the target position which this IK solve the chain for that
    default_target_position = [1, 2, 3]

    # Step2: Specify base bone of the chain
    base_bone_start_location = [0, 0, -20]
    base_bone_direction = [0, 0, 1]
    base_bone_length = 2

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
    base_bone_hinge_rotation_axis = [1, 0, 0]
    base_bone_hinge_reference_axis = [2, 1, 0]

    # Step4: adding consecutive bone to the chain
    # number of bones excluding base bone!
    number_bone = 2
    bone_direction = [0, 0, 1]
    bone_length = 2
    joint_type = "GLOBAL_HINGE"
    hinge_rotation_axis = [1, 0, 0]
    hinge_constraint_reference_axis = [2, 1, 0]
    cw_rad = 1.7628
    cw_deg = cw_rad * 180 / math.pi
    acw_rad = 1.7628
    acw_deg = acw_rad * 180 / math.pi

    ###### Solving!

    target = CG3dPoint(default_target_position[0], default_target_position[1], default_target_position[2])

    base_bone_start_location = CG3dPoint(base_bone_start_location[0], base_bone_start_location[1],
                                         base_bone_start_location[2])
    base_bone_direction = CG3dVector(base_bone_direction[0], base_bone_direction[1],
                                     base_bone_direction[2])

    m_bone = Bone.Bone3D(base_bone_start_location, base_bone_direction, base_bone_length)

    # create a new chain
    m_chain = Chain.Chain3d()
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

    bone_direction = CG3dVector(bone_direction[0], bone_direction[1], bone_direction[2])
    hinge_rotation_axis = CG3dVector(hinge_rotation_axis[0], hinge_rotation_axis[1], hinge_rotation_axis[2])
    hinge_constraint_reference_axis = CG3dVector(hinge_constraint_reference_axis[0],
                                                 hinge_constraint_reference_axis[1],
                                                 hinge_constraint_reference_axis[2])
    for i in range(1, number_bone + 1):
        m_chain.add_consecutive_hinged_bone(bone_direction, bone_length, joint_type, hinge_rotation_axis, cw_deg,
                                            acw_deg
                                            , hinge_constraint_reference_axis)

    m_chain.set_target(target)
    m_chain.solve_fabrik_ik()




if __name__ == "__main__":
    main()
