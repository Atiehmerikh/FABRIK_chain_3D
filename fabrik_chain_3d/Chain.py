from mpl_toolkits.mplot3d import Axes3D

from fabrik_chain_3d import Bone as Bone, Joint as Joint, Mat as Mat, Utils as Util
from fabrik_chain_3d.output_writer import *
import numpy as np
import matplotlib.pyplot as plt
import math



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
        self.bone_twist_limit = 2.8973*180/math.pi
        self.deg =[0]*4
        self. rotations = [0]*4 # the last one belongs to the base bone which is fixed!
        self.fixed_base_orientation = [1,0,0,0]
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
                                                    hinge_rotation_axis, is_fixed,bone_orientation):
        self.add_consecutive_hinged_bone(bone_direction_uv, bone_length, joint_type, hinge_rotation_axis, 180, 180,
                                         Util.Utils().gen_perpendicular_vector_quick(hinge_rotation_axis), is_fixed,bone_orientation)

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

    def add_consecutive_rotor_constrained_bone(self, bone_direction_uv, bone_length, constraint_angle_degs, is_fixed,bone_orientation):
        if self.get_chain_length() == 0:
            raise Exception("Add a basebone before attempting to add consectuive bones.")
        prev_bone_end =self.get_bone(self.get_chain_length() - 1).get_end_point_position()
        scale_direction = [i * bone_length for i in bone_direction_uv]
        bone_end_location = [x + y for x, y in zip(prev_bone_end, scale_direction)]

        m_bone = Bone.Bone3D(prev_bone_end, bone_end_location, bone_direction_uv, bone_length, is_fixed, bone_orientation)
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

        if Util.length_calc(hinge_rotation_axis) <= 0.0:
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

    def solve_for_orientation(self, outer_joint_orientation, inner_joint_orientation,loop):
        q1 = outer_joint_orientation
        q2 = inner_joint_orientation
        # finding the rotor that express rotation between two orientational frame(between outer and inner joint)
        rotor = Util.Utils().find_rotation_quaternion(q1, q2)
        if rotor[0]>1:
            rotor[0] =0.99
        if rotor[0] < -1:
            rotor[0] = -0.99
        needed_rotation = math.acos(rotor[0]) * 2 * (180 / np.pi)
        self.rotations[loop] = needed_rotation*(np.pi/180)
        if needed_rotation <= self.bone_twist_limit:
            # if the rotation is inside the limited
            return Mat.Mat().multiply_two_quaternion(rotor, outer_joint_orientation)
        else:
            # the maximum allowed rotation angle
            theta = (self.bone_twist_limit) * (np.pi / 180)
            self.rotations[loop] = theta
            # the rotation axis
            if abs(rotor[0]) ==1:
                return rotor
            v1 = np.dot(rotor[1:], (1 / math.sqrt(1 - rotor[0] ** 2)))
            w = math.cos(theta / 2)
            x = v1[0] * math.sin(theta / 2)
            y = v1[1] * math.sin(theta / 2)
            z = v1[2] * math.sin(theta / 2)
            return [w, x, y, z]

    def forward(self, target_position, target_orientation):
        for loop in range(self.chain_length - 1, -1, -1):
            #  Get the length of the bone we're working on
            this_bone = self.get_bone(loop)
            this_bone_length = this_bone.get_length()
            this_bone_joint = this_bone.get_joint()
            this_bone_joint_type = this_bone_joint.get_joint_type()

            # 	If we are NOT working on the end effector bone
            if loop != (self.chain_length - 1):
                if this_bone.is_fix_bone() == 1:
                    this_bone_outer_to_inner_uv = Util.Utils().negated(this_bone.get_fixed_bone_direction_uv())
                else:
                    # Get the outer-to-inner unit vector of the bone further out
                    outer_bone_outer_to_inner_uv = Util.Utils().negated(self.get_bone(loop + 1).get_direction_uv())
                    # Get the outer-to-inner unit vector of this bone
                    this_bone_outer_to_inner_uv = Util.Utils().negated(self.get_bone(loop).get_direction_uv())
                    next_bone_orientation = self.get_bone(loop+1).get_bone_orientation()
                    this_bone_orientation = self.get_bone(loop).get_bone_orientation()
                    this_bone.set_bone_orientation(self.solve_for_orientation(next_bone_orientation,this_bone_orientation,loop))
                    # Get the joint type for this bone and handle constraints on thisBoneOuterToInnerUV
                    if this_bone_joint_type == "BALL":
                        # Constrain to relative angle between this bone and the outer bone if required
                        angle_between_degs = Util.Utils().get_angle_between_degs(outer_bone_outer_to_inner_uv,
                                                                         this_bone_outer_to_inner_uv)
                        constrain_angle_degs = this_bone_joint.get_ball_joint_constraint_degs()
                        if angle_between_degs > constrain_angle_degs:
                            this_bone_outer_to_inner_uv = Util.Utils().get_angle_limited_uv(this_bone_outer_to_inner_uv,
                                                                                    outer_bone_outer_to_inner_uv,
                                                                                    constrain_angle_degs)
                    elif this_bone_joint_type == "GLOBAL_HINGE":
                        # Project this bone outer-to-inner direction onto the hinge rotation axis
                        this_bone_outer_to_inner_uv = Util.Utils().project_on_to_plane(this_bone_outer_to_inner_uv,
                                                                               this_bone_joint.get_hinge_rotation_axis())
                    elif this_bone_joint_type == "LOCAL_HINGE":
                        # Not a base bone? Then construct a rotation matrix based on the previous bones
                        # inner-to-outer direction...
                        if loop > 0:
                            m = Util.Utils().create_rotation_matrix(self.get_bone(loop - 1).get_direction_uv())
                            relative_hinge_rotation_axis = Util.Utils().normalization(
                                Util.Utils().times(m, this_bone_joint.get_hinge_rotation_axis()))

                            # transform the hinge rotation axis into the previous bones frame of reference.
                            # Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
                            this_bone_outer_to_inner_uv = Util.Utils().project_on_to_plane(this_bone_outer_to_inner_uv,
                                                                                   relative_hinge_rotation_axis)
                        else:
                            raise Exception("The base bone joint can't be LOCAL HINGE")

                scale = [i * this_bone_length for i in this_bone_outer_to_inner_uv]
                end_location = this_bone.get_end_point_position()
                new_start_location = [x + y for x, y in zip(end_location, scale)]

                this_bone.set_start_point_position(new_start_location)

                # If we are not working on the basebone, then we also set the end joint location of
                # the previous bone in the chain
                if loop > 0:
                    self.get_bone(loop - 1).set_end_point_position(new_start_location)
            # If we ARE working on the end effector bone..
            else:
                # put end effector end location to the target
                this_bone.set_end_point_position(target_position)
                this_bone.set_bone_orientation(
                    self.solve_for_orientation(target_orientation, this_bone.get_bone_orientation(), loop))
                if this_bone.is_fix_bone() == 1:
                    this_bone_outer_to_inner_uv = Util.Utils().negated(this_bone.get_fixed_bone_direction_uv())
                else:
                    this_bone_outer_to_inner_uv = Util.Utils().negated(this_bone.get_direction_uv())

                    if this_bone_joint_type == "BALL":
                        i=0
                    elif this_bone_joint_type == "GLOBAL_HINGE":
                        this_bone_outer_to_inner_uv = Util.Utils().project_on_to_plane(this_bone_outer_to_inner_uv,
                                                                               this_bone_joint.get_hinge_rotation_axis())
                    elif this_bone_joint_type == "LOCAL_HINGE":
                        m = Util.Utils().create_rotation_matrix(self.get_bone(loop - 1).get_direction_uv())
                        relative_hinge_rotation_axis = Util.Utils().normalization(
                            Util.Utils().times(m, this_bone_joint.get_hinge_rotation_axis()))
                        # Project this bone's outer-to-inner direction onto the plane described by the relative hinge
                        # rotation axis
                        this_bone_outer_to_inner_uv = Util.Utils().project_on_to_plane(this_bone_outer_to_inner_uv,
                                                                               relative_hinge_rotation_axis)

                scale = [i * this_bone_length for i in this_bone_outer_to_inner_uv]
                end_location = this_bone.get_end_point_position()
                new_start_location = [x + y for x, y in zip(end_location, scale)]

                this_bone.set_start_point_position(new_start_location)
                # If we are not working on the base bone, then we also set the end joint location of
                # the previous bone in the chain
                if loop > 0:
                    self.get_bone(loop - 1).set_end_point_position(new_start_location)
        # self.draw_chain()

    def backward(self):
        for loop in range(self.chain_length):
            this_bone = self.get_bone(loop)
            this_bone_length = self.get_bone(loop).get_length()
            # If we are not working on the base bone
            if loop != 0:
                if this_bone.is_fix_bone() == 1:
                    this_bone_inner_to_outer_uv = this_bone.get_fixed_bone_direction_uv()
                else:
                    this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                    prev_bone_inner_to_outer_uv = self.get_bone(loop - 1).get_direction_uv()
                    this_bone_joint = this_bone.get_joint()
                    this_bone_joint_type = this_bone_joint.get_joint_type()
                    if this_bone_joint_type == "BALL":
                        angle_between_degs = Util.Utils().get_angle_between_degs(prev_bone_inner_to_outer_uv,
                                                                         this_bone_inner_to_outer_uv)
                        constraint_angle_degs = this_bone_joint.get_ball_joint_constraint_degs()
                        self.deg[loop] =angle_between_degs
                        if angle_between_degs > constraint_angle_degs:
                            this_bone_inner_to_outer_uv = Util.Utils().get_angle_limited_uv(this_bone_inner_to_outer_uv,
                                                                                    prev_bone_inner_to_outer_uv,
                                                                                    constraint_angle_degs)
                            self.deg[loop] =constraint_angle_degs
                    elif this_bone_joint_type == "GLOBAL_HINGE":
                        # Get the hinge rotation axis and project our inner-to-outer UV onto it
                        this_bone_inner_to_outer_uv = Util.Utils().project_on_to_plane(this_bone_inner_to_outer_uv,
                                                                               this_bone_joint.get_hinge_rotation_axis())
                        # If there are joint constraints, then we must honour them...
                        cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                        acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                        if not Util.Utils().approximately_equal(cw_constraint_degs,
                                                        -this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                        0.001) and not Util.Utils().approximately_equal(acw_constraint_degs,
                                                                                                this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                                                                0.001):
                            hinge_reference_axis = this_bone_joint.get_reference_axis()
                            hinge_rotation_axis = this_bone_joint.get_hinge_rotation_axis()
                            # Get the signed angle (about the hinge rotation axis) between the hinge reference axis and the hinge-rotation aligned bone UV
                            signed_angle_degs = Util.Utils().get_signed_angle_between_degs(hinge_reference_axis,
                                                                                   this_bone_inner_to_outer_uv,
                                                                                   hinge_rotation_axis)
                            self.deg[loop] = signed_angle_degs*math.pi/180
                            # Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
                            if signed_angle_degs > acw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(hinge_reference_axis, acw_constraint_degs,
                                                          hinge_rotation_axis))
                                self.deg[loop] = acw_constraint_degs*math.pi/180
                            elif signed_angle_degs < cw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(hinge_reference_axis, cw_constraint_degs,
                                                          hinge_rotation_axis))
                                self.deg[loop] = cw_constraint_degs*math.pi/180
                    elif this_bone_joint_type == "LOCAL_HINGE":
                        # Transform the hinge rotation axis to be relative to the previous bone in the chain
                        hinge_rotation_axis = this_bone_joint.get_hinge_rotation_axis()
                        m = Util.Utils().create_rotation_matrix(prev_bone_inner_to_outer_uv)
                        relative_hinge_rotation_axis = Util.Utils().normalization(Util.Utils().times(m, hinge_rotation_axis))
                        this_bone_inner_to_outer_uv = Util.Utils().project_on_to_plane(this_bone_inner_to_outer_uv,
                                                                               relative_hinge_rotation_axis)
                        # Constrain rotation about reference axis if required
                        cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                        acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                        if not Util.Utils().approximately_equal(cw_constraint_degs,
                                                        -this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                        0.001) and not Util.Utils().approximately_equal(acw_constraint_degs,
                                                                                                this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                                                                0.001):
                            relative_hinge_reference_axis = Util.Utils().normalization(
                                Util.Utils().times(m, this_bone_joint.get_reference_axis()))

                            signed_angle_degs = Util.Utils().get_signed_angle_between_degs(relative_hinge_reference_axis,
                                                                                   this_bone_inner_to_outer_uv,
                                                                                   relative_hinge_rotation_axis)
                            self.deg[loop] = signed_angle_degs*math.pi/180
                            if signed_angle_degs > acw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(relative_hinge_reference_axis, acw_constraint_degs,
                                                          relative_hinge_rotation_axis))
                                self.deg[loop] = acw_constraint_degs*math.pi/180
                            elif signed_angle_degs < cw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(relative_hinge_reference_axis, cw_constraint_degs,
                                                          relative_hinge_rotation_axis))
                                self.deg[loop] = cw_constraint_degs*math.pi/180
                        # twisted = np.cross(this_bone_inner_to_outer_uv,relative_hinge_rotation_axis)
                        # print("bone"+str(loop))
                        # print(twisted)

                scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                start_location = this_bone.get_start_point_position()
                new_end_location = [x + y for x, y in zip(start_location, scale)]
                this_bone.set_end_point_position(new_end_location)
                if loop < self.chain_length - 1:
                    self.get_bone(loop + 1).set_start_point_position(new_end_location)

            #  If we ARE working on the basebone...
            else:
                self.get_bone(0).set_start_point_position(self.fixed_base_location)
                if self.is_base_bone_fixed == 1:
                    self.get_bone(0).set_end_point_position(self.fixed_base_location_2)
                    if self.chain_length > 1:
                        self.get_bone(1).set_start_point_position(self.fixed_base_location_2)
                else:
                    this_bone_joint = this_bone.get_joint()
                    this_bone_joint_type = this_bone_joint.get_joint_type()
                    if this_bone_joint_type == "GLOBAL_HINGE":
                        hinge_rotation_axis = this_bone_joint.get_hinge_rotation_axis()
                        cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                        acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                        this_bone_inner_to_outer_uv = Util.Utils().project_on_to_plane(this_bone.get_direction_uv(),
                                                                               hinge_rotation_axis)
                        # If we have a global hinge which is not freely rotating then we must constrain about the reference axis
                        if not Util.Utils().approximately_equal(cw_constraint_degs,
                                                        -this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                        0.001) and not Util.Utils().approximately_equal(acw_constraint_degs,
                                                                                                this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                                                                0.001):
                            hinge_reference_axis = this_bone_joint.get_reference_axis()
                            signed_angle_degs = Util.Utils().get_signed_angle_between_degs(hinge_reference_axis,
                                                                                   this_bone_inner_to_outer_uv,
                                                                                   hinge_rotation_axis)
                            self.deg[loop] = signed_angle_degs * math.pi / 180
                            if signed_angle_degs > acw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(hinge_reference_axis, acw_constraint_degs,
                                                          hinge_rotation_axis))
                                self.deg[loop] = acw_constraint_degs * math.pi / 180
                            elif signed_angle_degs < cw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(hinge_reference_axis, cw_constraint_degs,
                                                          hinge_rotation_axis))
                                self.deg[loop] = cw_constraint_degs * math.pi / 180

                        # twisted = np.cross(this_bone_inner_to_outer_uv, hinge_rotation_axis)
                        # print("bone" + str(loop))
                        # print(twisted)
                        scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                        start_location = this_bone.get_start_point_position()
                        new_end_location = [x + y for x, y in zip(start_location, scale)]
                        this_bone.set_end_point_position(new_end_location)
                        if self.chain_length > 1:
                            self.get_bone(1).set_start_point_position(new_end_location)
                    if this_bone_joint_type=="BALL":
                        this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                        angle_between_degs = Util.Utils().get_angle_between_degs(self.base_bone_constraint_uv,this_bone_inner_to_outer_uv)
                        constraint_angle_degs = this_bone.get_ball_joint_constraint_degs()
                        self.deg[loop] = angle_between_degs * math.pi / 180
                        if angle_between_degs>constraint_angle_degs:
                            this_bone_inner_to_outer_uv = Util.Utils().get_angle_limited_uv(this_bone_inner_to_outer_uv,self.base_bone_constraint_uv,
                                                                                    constraint_angle_degs)
                            self.deg[loop] = constraint_angle_degs * math.pi / 180

                        scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                        start_location = this_bone.get_start_point_position()
                        new_end_location = [x + y for x, y in zip(start_location, scale)]
                        this_bone.set_end_point_position(new_end_location)
                        if self.chain_length > 1:
                            self.get_bone(1).set_start_point_position(new_end_location)
                    else:
                        this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                        scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                        start_location = this_bone.get_start_point_position()
                        new_end_location = [x + y for x, y in zip(start_location, scale)]
                        this_bone.set_end_point_position(new_end_location)
                        if self.chain_length > 1:
                            self.get_bone(1).set_start_point_position(new_end_location)
        # self.draw_chain()

    def solve_fabrik_ik(self):
        dist_base_to_target = Util.Utils().get_distance_between(self.get_bone(0).get_start_point_position(),
                                                        self.target_position)
        total_chain_length = 0
        for i in range(0, self.chain_length):
            total_chain_length += self.get_bone(i).get_length()
        # print("The initial joint angles and diagram. close the figure to see the final configuration\n")
        # self.draw_chain()
        if dist_base_to_target <= total_chain_length:
            if self.get_chain_length == 0:
                raise Exception("It makes no sense to solve an IK chain with zero bones.")

            dist_to_target = Util.Utils().get_distance_between(self.get_bone(self.chain_length - 1).get_end_point_position(),
                                                       self.target_position)

            counter = 0
            while dist_to_target > self.get_solve_distance_threshold() and counter < 10000:
                self.forward(self.target_position,self.target_orientation)
                self.backward()

                dist_to_target = Util.Utils().get_distance_between(
                    self.get_bone(self.get_chain_length() - 1).get_end_point_position(),
                    self.target_position)
                counter += 1
                # self.draw_chain()

            # after finding these joint position we can do anything with them.
            # Here I calculate the joints_angle:
            # print("Final joint angles:\n")
            self.draw_chain()

            # self.output_joint_angles()
        else:
            print("Target is so far! can't be reached")
            return

    def angles(self):
        angles =[]
        # for base bone twist
        base_bone = self.get_bone(0)
        base_bone_orientation = base_bone.get_bone_orientation()
        # number 3 belongs to the rotation matrix which is 0 for bone 3 rotation, 1 for bone 5 rotation,
        # 2 for bone 7 rotation, 3 for bone 0 rotation
        self.solve_for_orientation(base_bone_orientation,self.fixed_base_orientation, 3)
        angles.append(self.rotations[3])
        # for bone 2(in schematic)
        angles.append(self.deg[0])
        angles.append(self.rotations[0])
        # for bone 3(in schematic)
        angles.append(self.deg[1])
        angles.append(self.rotations[1])
        # for bone 4(in schematic)
        angles.append(self.deg[2])
        angles.append(self.rotations[2])

        s = ','.join([str(n) for n in angles])
        print(s)

    def draw_chain(self):
        self.angles()
        coordinate = self.fill_array(self.points_retrieval())
        x_prime = coordinate[0]
        y_prime = coordinate[1]
        z_prime = coordinate[2]

        f = OutputWriter(self.base_address, self.joints_file_address).joint_writer()
        for i in range(0, len(x_prime)):
            f.write(str(x_prime[i]))
            f.write(' ')
            f.write(str(y_prime[i]))
            f.write(' ')
            f.write(str(z_prime[i]))

            f.write("\n")
        f.close()
        fig = plt.figure()
        # ax = fig.gca(projection='3d')
        ax = Axes3D(fig)
        ax.plot3D(x_prime, y_prime, z_prime, color='red')
        ax.scatter3D(self.target_position[0], self.target_position[1], self.target_position[2])
        ax.scatter3D(x_prime, y_prime, z_prime)

        for i in range(len(x_prime) - 1):
            print('length bone ' + str(i + 1))
            x = x_prime[i] - x_prime[i + 1]
            y = y_prime[i] - y_prime[i + 1]
            z = z_prime[i] - z_prime[i + 1]
            print("%.2f" % round(math.sqrt(x * x + y * y + z * z), 2))
        plt.show()

    def set_axes_radius(self, ax, origin, radius):
        ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
        ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
        ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

    def set_axes_equal(self, ax):
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ])
        origin = np.mean(limits, axis=1)
        radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
        self.set_axes_radius(ax, origin, radius)

    def fill_array(self, start_locations):
        w, h = 3, len(start_locations)
        coordinate = [[0 for x in range(w)] for y in range(h)]
        # coordinate =[][len(body_part_index)]
        x = [0]
        y = [0]
        z = [0]
        length = [0.316, 0.088,0.088]
        for i in range(len(start_locations)):
            if i>=1:
                uv = [(start_locations[i][0]-start_locations[i-1][0]),
                      (start_locations[i][1]-start_locations[i-1][1]),
                      (start_locations[i][2]-start_locations[i-1][2])]
                uv = Mat.Mat().normalization(uv)
                scale = np.dot(uv, length[i-1])
                middle_points = [a + b for a, b in zip(start_locations[i-1], scale)]
                x.append(middle_points[0])
                y.append(middle_points[1])
                z.append(middle_points[2])

            x.append(start_locations[i][0])
            y.append(start_locations[i][1])
            z.append(start_locations[i][2])

        coordinate[0][:] = x
        coordinate[1][:] = y
        coordinate[2][:] = z
        return coordinate

    def points_retrieval(self):
        start_locations = []
        end_locations = []
        for i in range(0, self.get_chain_length()):
            start_loc = self.get_bone(i).get_start_point_position()
            # end_loc = self.chain.get_bone(i).set_end_point()
            start_locations.append(start_loc)
            # end_locations.append(end_loc)
        end_effector_bone = self.chain[self.get_chain_length() - 1].end_point
        start_locations.append(end_effector_bone)
        return start_locations