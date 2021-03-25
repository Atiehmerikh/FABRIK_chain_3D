from fabrik_chain_3d import Bone as Bone, Joint as Joint, Mat as Mat, Utils as Util
import FABRIK as fabrik
import numpy as np
import math
import Visualization as draw_chain


class Chain3d:
    def __init__(self, is_base_bone_fixed, base_address="./test/output/", joints_file_address="joints-position.txt"):
        self.is_base_bone_fixed = is_base_bone_fixed
        self.target_position = [1, 0, 0]
        self.target_orientation = [1, 0, 0, 0]
        self.chain = []
        self.solve_distance_threshold = 0.001
        self.max_iteration_attempts = 20
        self.min_iteration_change = 0.01
        self.fixed_base_location = [0, 0, 0]
        self.fixed_base_location_2 = [0, 0, 0]
        self.chain_length = 0
        self.base_bone_constraint_type = "None"
        self.base_bone_constraint_uv = [0, 0, 0]
        self.current_solve_distance = 100000000
        self.bone_twist_limit = 2.8973 * 180 / math.pi
        # the last one belongs to the base bone which is fixed!
        self.fixed_base_orientation = [1, 0, 0, 0]
        self.base_address = base_address
        self.joints_file_address = joints_file_address

    def update_chain_length(self):
        self.chain_length = 0
        for i in self.chain:
            self.chain_length += 1

    def add_bone(self, bone):
        self.chain.append(bone)
        # if it is the base bone:
        if len(self.chain) == 1:
            self.fixed_base_location = bone.get_start_point_position()
            if self.is_base_bone_fixed == 1:
                self.fixed_base_location_2 = bone.get_end_point_position()
            self.base_bone_constraint_uv = bone.get_direction_uv()

        self.update_chain_length()

    def add_consecutive_bone(self, bone):
        direction = bone.get_direction_uv()
        length = bone.get_length()
        # If we have at least one bone already in the chain...
        if self.get_chain_length() != 0:
            prev_bone_end = self.get_bone(self.chain_length - 1).get_end_point_position()
            bone.set_start_point_position(prev_bone_end)
            scale = np.dot(direction, length)
            end_point = [x + y for x, y in zip(prev_bone_end, scale)]
            bone.set_end_point_position(end_point)
            self.add_bone(bone)
        else:
            raise Exception(
                "You cannot add the base bone to a chain using this method as it does not provide a start location.")

    def add_consecutive_freely_rotating_hinged_bone(self, bone_direction_uv, bone_length, joint_type,
                                                    hinge_rotation_axis, is_fixed, bone_orientation):
        self.add_consecutive_hinged_bone(bone_direction_uv, bone_length, joint_type, hinge_rotation_axis, 180, 180,
                                         Util.Utils().gen_perpendicular_vector_quick(hinge_rotation_axis), is_fixed,
                                         bone_orientation)

    def add_consecutive_hinged_bone(self, bone_direction_uv, bone_length, joint_type, hinge_rotation_axis,
                                    clockwise_degs,
                                    anticlockwise_deg, hinge_reference_axis, is_bone_fixed, bone_orientation):
        Util.Utils().validate_direction(bone_direction_uv)
        Util.Utils().validate_direction(hinge_rotation_axis)
        Util.Utils().validate_length(bone_length)

        if self.chain_length == 0:
            raise Exception("You must add a base bone before adding a consectutive bone.")

        bone_direction_uv = (bone_direction_uv)
        hinge_rotation_axis = Util.Utils().normalization(hinge_rotation_axis)
        prev_bone_end = self.get_bone(self.chain_length - 1).get_end_point_position()
        scale_direction = [i * bone_length for i in bone_direction_uv]
        bone_end_location = [x + y for x, y in zip(prev_bone_end, scale_direction)]
        m_bone = Bone.Bone3D(prev_bone_end, bone_end_location, bone_direction_uv, bone_length, is_bone_fixed,
                             bone_orientation)
        m_joint = Joint.Joint3D()
        if joint_type == "GLOBAL_HINGE":
            m_joint.set_as_global_hinge(hinge_rotation_axis, clockwise_degs, anticlockwise_deg, hinge_reference_axis)
        elif joint_type == "LOCAL_HINGE":
            m_joint.set_as_local_hinge(hinge_rotation_axis, clockwise_degs, anticlockwise_deg, hinge_reference_axis)
        else:
            raise Exception("Hinge joint types may be only JointType.GLOBAL_HINGE or JointType.LOCAL_HINGE.")

        m_bone.set_joint(m_joint)
        self.add_bone(m_bone)

    def add_consecutive_rotor_constrained_bone(self, bone_direction_uv, bone_length, constraint_angle_degs, is_fixed,
                                               bone_orientation):
        if self.get_chain_length() == 0:
            raise Exception("Add a basebone before attempting to add consectuive bones.")
        prev_bone_end = self.get_bone(self.get_chain_length() - 1).get_end_point_position()
        scale_direction = [i * bone_length for i in bone_direction_uv]
        bone_end_location = [x + y for x, y in zip(prev_bone_end, scale_direction)]

        m_bone = Bone.Bone3D(prev_bone_end, bone_end_location, bone_direction_uv, bone_length, is_fixed,
                             bone_orientation)
        rotor_joint = Joint.Joint3D()
        rotor_joint.set_as_ball_joint(constraint_angle_degs)
        rotor_joint.set_as_ball_joint(constraint_angle_degs)
        m_bone.set_joint(rotor_joint)
        m_bone.set_ball_joint_constraint_degs(constraint_angle_degs)
        self.add_bone(m_bone)

    def get_base_bone_constraint_uv(self):
        return self.base_bone_constraint_uv

    def get_base_bone_constraint_type(self):
        if self.base_bone_constraint_type is None:
            raise Exception('Cannot return the basebone constraint when the basebone constraint type is NONE.')
        return self.base_bone_constraint_type

    def get_base_location(self):
        return self.get_bone(0).get_start_point_position()

    def get_bone(self, bone_number):
        return self.chain[bone_number]

    def get_chain(self):
        return self.chain

    def get_chain_length(self):
        return self.chain_length

    def get_end_effector_location(self):
        return self.get_bone(self.chain_length - 1).get_end_point_position()

    def remove_bone(self, bone_number):
        if bone_number < self.chain_length:
            self.chain.remove(self.get_bone(bone_number))
            self.update_chain_length()
        else:
            raise Exception(
                "Bone " + bone_number + " does not exist to be removed from the chain. Bones are zero indexed.")

    def set_rotor_base_bone_constraint(self, rotor_type, constraint_axis, angle_degs):
        # Sanity checking
        if len(self.chain) == 0:
            raise Exception("Chain must contain a basebone before we can specify the basebone constraint type.")

        if len(constraint_axis) <= 0.0:
            raise Exception("Constraint axis cannot be zero.")

        if angle_degs < 0.0:
            angle_degs = 0.0

        if angle_degs > 180.0:
            angle_degs = 180.0

        # if rotor_type != "GLOBAL_ROTOR" or rotor_type != "LOCAL_ROTOR":
        #     raise Exception("The only valid rotor types for this method are GLOBAL_ROTOR and LOCAL_ROTOR.")

        #  Set the constraint type, axis and angle
        self.base_bone_constraint_type = rotor_type
        self.base_bone_constraint_uv = Util.Utils().normalization(constraint_axis)
        rotor_joint = Joint.Joint3D()
        rotor_joint.set_as_ball_joint(angle_degs)
        self.get_bone(0).set_joint(rotor_joint)

    def set_hinge_base_bone_constraint(self, hinge_type, hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                       hinge_reference_axis, ):
        # Sanity checking
        if len(self.chain) == 0:
            raise Exception("Chain must contain a base bone before we can specify the base bone constraint type.")

        if Util.Utils.length_calc(hinge_rotation_axis) <= 0.0:
            raise Exception("Constraint axis cannot be zero.")

        if Util.length_calc(hinge_reference_axis) <= 0.0:
            raise Exception("Constraint axis cannot be zero.")

        # if np.inner(hinge_rotation_axis, hinge_reference_axis) == 0:
        #     # raise Exception(
        #     "The hinge reference-axis must be in the plane of the hinge rotation axis, i.e"
        #     ", they must not be perpendicular")

        # if hinge_type != "GLOBAL_HINGE" and hinge_type != "LOCAL_HINGE":
        #     raise Exception("The only valid rotor types for this method are GLOBAL_HINGE and LOCAL_HINGE.")

        #  Set the constraint type, axis and angle
        self.base_bone_constraint_type = hinge_type
        self.base_bone_constraint_uv = Util.normalization(hinge_rotation_axis)

        hinge_joint = Joint.Joint3D()

        if hinge_type == "GLOBAL_HINGE":
            hinge_joint.set_hinge("GLOBAL_HINGE", hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                  hinge_reference_axis)

        else:
            hinge_joint.set_hinge("LOCAL_HINGE", hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                  hinge_reference_axis)

        self.get_bone(0).set_joint(hinge_joint)

    def set_freely_rotating_global_hinged_base_bone(self, hinge_rotation_axis):
        self.set_hinge_base_bone_constraint("GLOBAL_HINGE", hinge_rotation_axis, 180, 180,
                                            Util.Utils().gen_perpendicular_vector_quick(hinge_rotation_axis))

    def set_freely_rotating_local_hinged_base_bone(self, hinge_rotation_axis):
        self.set_hinge_base_bone_constraint("LOCAL_HINGE", hinge_rotation_axis, 180, 180,
                                            Util.Utils().gen_perpendicular_vector_quick(hinge_rotation_axis))

    def set_local_hinged_base_bone(self, hinge_rotation_axis, cw_constraint_degs, acw_constraint_degs,
                                   hinge_reference_axis):
        self.set_hinge_base_bone_constraint("LOCAL_HINGE", hinge_rotation_axis, cw_constraint_degs, acw_constraint_degs,
                                            hinge_reference_axis)

    def set_global_hinged_base_bone(self, hinge_rotation_axis, cw_constraint_degs, acw_constraint_degs,
                                    hinge_reference_axis):
        self.set_hinge_base_bone_constraint("GLOBAL_HINGE", hinge_rotation_axis, cw_constraint_degs,
                                            acw_constraint_degs,
                                            hinge_reference_axis)

    def set_base_bone_constraint_uv(self, constraint_uv):
        if len(constraint_uv) == 0:
            raise Exception("direction unit vector cannot be zero")
        self.base_bone_constraint_uv = Util.Utils().normalization(constraint_uv)

    def set_base_location(self, base_location):
        self.fixed_base_location = base_location

    def set_max_iteration_attempt(self, max_iteration):
        if max_iteration < 1:
            raise Exception("Minimum number of iteration is 1")
        self.max_iteration_attempts = max_iteration

    def get_max_iteration_attempt(self):
        return self.max_iteration_attempts

    def set_min_iteration_change(self, min_iteration_change):
        if min_iteration_change < 0:
            raise Exception("Minimum iteration change greater than zero")
        self.min_iteration_change = min_iteration_change

    def get_min_iteration_change(self):
        return self.min_iteration_change

    def set_solve_distance_threshold(self, solve_distance):
        if solve_distance < 0:
            raise Exception("The solve distance threshold must be greater than or equal to zero.")
        self.solve_distance_threshold = solve_distance

    def get_solve_distance_threshold(self):
        return self.solve_distance_threshold

    def set_target(self, target_position, target_orientation):
        self.target_position = target_position
        self.target_orientation = target_orientation

    def solve_fabrik_ik(self):
        dist_base_to_target = Util.Utils().get_distance_between(self.get_bone(0).get_start_point_position(),
                                                                self.target_position)
        total_chain_length = 0
        for i in range(0, self.chain_length):
            total_chain_length += self.get_bone(i).get_length()
        if dist_base_to_target <= total_chain_length:
            if self.get_chain_length == 0:
                raise Exception("It makes no sense to solve an IK chain with zero bones.")

            dist_to_target = Util.Utils().get_distance_between(
                self.get_bone(self.chain_length - 1).get_end_point_position(),
                self.target_position)

            counter = 0
            m_FABRIK = fabrik.FABRIK(self.get_chain(), self.get_chain_length(), self.target_position,
                                     self.target_orientation
                                     , self.is_base_bone_fixed, self.get_base_bone_constraint_uv(),
                                     self.fixed_base_location)
            while dist_to_target > self.get_solve_distance_threshold() and counter < 10000:

                self.chain = m_FABRIK.forward()
                self.chain = m_FABRIK.backward()

                dist_to_target = Util.Utils().get_distance_between(
                    self.get_bone(self.get_chain_length() - 1).get_end_point_position(),
                    self.target_position)
                counter += 1

            # after finding these joint position we can do anything with them.
            # Here I calculate the joints_angle:
            # print("Final joint angles:\n")
            m_draw = draw_chain.Visualization(self.target_position, self.get_chain(), m_FABRIK.get_deg(), m_FABRIK.get_rotations())
            m_draw.draw_chain()

            # self.output_joint_angles()
        else:
            print("Target is so far! can't be reached")
            return
