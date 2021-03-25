from fabrik_chain_3d import Bone as Bone, Joint as Joint, Mat as Mat, Utils as Util
import math
import numpy as np


class FABRIK():
    def __init__(self, chain, chain_length, target_position, target_orientation,is_base_bone_fixed,base_bone_constraint_uv,fixed_base_location):
        self.chain = chain
        self.target_position = target_position
        self.target_orientation = target_orientation
        self.chain_length = chain_length
        self.bone_twist_limit = 2.8973 * 180 / math.pi
        self.deg = [0] * 4
        self.rotations = [0] * 4  # the last one belongs to the base bone which is fixed!
        self.fixed_base_location = fixed_base_location
        self.fixed_base_location_2 = [0, 0, 0]
        self.is_base_bone_fixed = is_base_bone_fixed
        self.base_bone_constraint_uv = base_bone_constraint_uv

    def solve_for_orientation(self, outer_joint_orientation, inner_joint_orientation, loop):
        q1 = outer_joint_orientation
        q2 = inner_joint_orientation
        # finding the rotor that express rotation between two orientational frame(between outer and inner joint)
        rotor = Util.Utils().find_rotation_quaternion(q1, q2)
        if rotor[0] > 1:
            rotor[0] = 0.99
        if rotor[0] < -1:
            rotor[0] = -0.99
        needed_rotation = math.acos(rotor[0]) * 2 * (180 / np.pi)
        self.rotations[loop] = needed_rotation * (np.pi / 180)
        if needed_rotation <= self.bone_twist_limit:
            # if the rotation is inside the limited
            return Mat.Mat().multiply_two_quaternion(rotor, outer_joint_orientation)
        else:
            # the maximum allowed rotation angle
            theta = (self.bone_twist_limit) * (np.pi / 180)
            self.rotations[loop] = theta
            # the rotation axis
            if abs(rotor[0]) == 1:
                return rotor
            v1 = np.dot(rotor[1:], (1 / math.sqrt(1 - rotor[0] ** 2)))
            w = math.cos(theta / 2)
            x = v1[0] * math.sin(theta / 2)
            y = v1[1] * math.sin(theta / 2)
            z = v1[2] * math.sin(theta / 2)
            return [w, x, y, z]

    def forward(self):
        for loop in range(self.chain_length - 1, -1, -1):
            #  Get the length of the bone we're working on
            this_bone = self.chain[loop]
            this_bone_length = this_bone.get_length()
            this_bone_joint = this_bone.get_joint()
            this_bone_joint_type = this_bone_joint.get_joint_type()

            # 	If we are NOT working on the end effector bone
            if loop != (self.chain_length - 1):
                if this_bone.is_fix_bone() == 1:
                    this_bone_outer_to_inner_uv = Util.Utils().negated(this_bone.get_fixed_bone_direction_uv())
                else:
                    # Get the outer-to-inner unit vector of the bone further out
                    outer_bone_outer_to_inner_uv = Util.Utils().negated(
                        self.chain[loop + 1].get_direction_uv())
                    # Get the outer-to-inner unit vector of this bone
                    this_bone_outer_to_inner_uv = Util.Utils().negated(
                        self.chain[loop].get_direction_uv())
                    next_bone_orientation = self.chain[loop+1].get_bone_orientation()
                    this_bone_orientation = self.chain[loop+1].get_bone_orientation()
                    this_bone.set_bone_orientation(
                        self.solve_for_orientation(next_bone_orientation, this_bone_orientation, loop))
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
                            m = Util.Utils().create_rotation_matrix(self.chain[loop - 1].get_direction_uv())
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
                    self.chain[loop - 1].set_end_point_position(new_start_location)
            # If we ARE working on the end effector bone..
            else:
                # put end effector end location to the target
                this_bone.set_end_point_position(self.target_position)
                this_bone.set_bone_orientation(
                    self.solve_for_orientation(self.target_orientation, this_bone.get_bone_orientation(), loop))
                if this_bone.is_fix_bone() == 1:
                    this_bone_outer_to_inner_uv = Util.Utils().negated(this_bone.get_fixed_bone_direction_uv())
                else:
                    this_bone_outer_to_inner_uv = Util.Utils().negated(this_bone.get_direction_uv())

                    if this_bone_joint_type == "BALL":
                        i = 0
                    elif this_bone_joint_type == "GLOBAL_HINGE":
                        this_bone_outer_to_inner_uv = Util.Utils().project_on_to_plane(this_bone_outer_to_inner_uv,
                                                                                       this_bone_joint.get_hinge_rotation_axis())
                    elif this_bone_joint_type == "LOCAL_HINGE":
                        m = Util.Utils().create_rotation_matrix(self.chain[loop - 1].get_direction_uv())
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
                    self.chain[loop - 1].set_end_point_position(new_start_location)
        return self.chain

    def backward(self):
        for loop in range(self.chain_length):
            this_bone = self.chain[loop]
            this_bone_length = self.chain[loop].get_length()
            # If we are not working on the base bone
            if loop != 0:
                if this_bone.is_fix_bone() == 1:
                    this_bone_inner_to_outer_uv = this_bone.get_fixed_bone_direction_uv()
                else:
                    this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                    prev_bone_inner_to_outer_uv = self.chain[loop - 1].get_direction_uv()
                    this_bone_joint = this_bone.get_joint()
                    this_bone_joint_type = this_bone_joint.get_joint_type()
                    if this_bone_joint_type == "BALL":
                        angle_between_degs = Util.Utils().get_angle_between_degs(prev_bone_inner_to_outer_uv,
                                                                                 this_bone_inner_to_outer_uv)
                        constraint_angle_degs = this_bone_joint.get_ball_joint_constraint_degs()
                        self.deg[loop] = angle_between_degs
                        if angle_between_degs > constraint_angle_degs:
                            this_bone_inner_to_outer_uv = Util.Utils().get_angle_limited_uv(this_bone_inner_to_outer_uv,
                                                                                            prev_bone_inner_to_outer_uv,
                                                                                            constraint_angle_degs)
                            self.deg[loop] = constraint_angle_degs
                    elif this_bone_joint_type == "GLOBAL_HINGE":
                        # Get the hinge rotation axis and project our inner-to-outer UV onto it
                        this_bone_inner_to_outer_uv = Util.Utils().project_on_to_plane(this_bone_inner_to_outer_uv,
                                                                                       this_bone_joint.get_hinge_rotation_axis())
                        # If there are joint constraints, then we must honour them...
                        cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                        acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                        if not Util.Utils().approximately_equal(cw_constraint_degs,
                                                                -this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                                0.001) and not Util.Utils().approximately_equal(
                            acw_constraint_degs,
                            this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                            0.001):
                            hinge_reference_axis = this_bone_joint.get_reference_axis()
                            hinge_rotation_axis = this_bone_joint.get_hinge_rotation_axis()
                            # Get the signed angle (about the hinge rotation axis) between the hinge reference axis and the hinge-rotation aligned bone UV
                            signed_angle_degs = Util.Utils().get_signed_angle_between_degs(hinge_reference_axis,
                                                                                           this_bone_inner_to_outer_uv,
                                                                                           hinge_rotation_axis)
                            self.deg[loop] = signed_angle_degs * math.pi / 180
                            # Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
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
                    elif this_bone_joint_type == "LOCAL_HINGE":
                        # Transform the hinge rotation axis to be relative to the previous bone in the chain
                        hinge_rotation_axis = this_bone_joint.get_hinge_rotation_axis()
                        m = Util.Utils().create_rotation_matrix(prev_bone_inner_to_outer_uv)
                        relative_hinge_rotation_axis = Util.Utils().normalization(
                            Util.Utils().times(m, hinge_rotation_axis))
                        this_bone_inner_to_outer_uv = Util.Utils().project_on_to_plane(this_bone_inner_to_outer_uv,
                                                                                       relative_hinge_rotation_axis)
                        # Constrain rotation about reference axis if required
                        cw_constraint_degs = -this_bone_joint.get_hinge_clockwise_constraint_degs()
                        acw_constraint_degs = this_bone_joint.get_hinge_anticlockwise_constraint_degs()
                        if not Util.Utils().approximately_equal(cw_constraint_degs,
                                                                -this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                                                                0.001) and not Util.Utils().approximately_equal(
                            acw_constraint_degs,
                            this_bone_joint.get_MAX_CONSTRAINT_ANGLE_DEGS(),
                            0.001):
                            relative_hinge_reference_axis = Util.Utils().normalization(
                                Util.Utils().times(m, this_bone_joint.get_reference_axis()))

                            signed_angle_degs = Util.Utils().get_signed_angle_between_degs(
                                relative_hinge_reference_axis,
                                this_bone_inner_to_outer_uv,
                                relative_hinge_rotation_axis)
                            self.deg[loop] = signed_angle_degs * math.pi / 180
                            if signed_angle_degs > acw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(relative_hinge_reference_axis, acw_constraint_degs,
                                                                relative_hinge_rotation_axis))
                                self.deg[loop] = acw_constraint_degs * math.pi / 180
                            elif signed_angle_degs < cw_constraint_degs:
                                this_bone_inner_to_outer_uv = Util.Utils().normalization(
                                    Mat.Mat().rotate_about_axis(relative_hinge_reference_axis, cw_constraint_degs,
                                                                relative_hinge_rotation_axis))
                                self.deg[loop] = cw_constraint_degs * math.pi / 180
                        # twisted = np.cross(this_bone_inner_to_outer_uv,relative_hinge_rotation_axis)
                        # print("bone"+str(loop))
                        # print(twisted)

                scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                start_location = this_bone.get_start_point_position()
                new_end_location = [x + y for x, y in zip(start_location, scale)]
                this_bone.set_end_point_position(new_end_location)
                if loop < self.chain_length - 1:
                    self.chain[loop + 1].set_start_point_position(new_end_location)

            #  If we ARE working on the basebone...
            else:
                self.chain[0].set_start_point_position(self.fixed_base_location)
                if self.is_base_bone_fixed == 1:
                    self.chain[0].set_end_point_position(self.fixed_base_location_2)
                    if self.chain_length > 1:
                        self.chain[1].set_start_point_position(self.fixed_base_location_2)
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
                                                                0.001) and not Util.Utils().approximately_equal(
                            acw_constraint_degs,
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
                            self.chain[1].set_start_point_position(new_end_location)
                    if this_bone_joint_type == "BALL":
                        this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                        angle_between_degs = Util.Utils().get_angle_between_degs(self.base_bone_constraint_uv,
                                                                                 this_bone_inner_to_outer_uv)
                        constraint_angle_degs = this_bone.get_ball_joint_constraint_degs()
                        self.deg[loop] = angle_between_degs * math.pi / 180
                        if angle_between_degs > constraint_angle_degs:
                            this_bone_inner_to_outer_uv = Util.Utils().get_angle_limited_uv(this_bone_inner_to_outer_uv,
                                                                                            self.base_bone_constraint_uv,
                                                                                            constraint_angle_degs)
                            self.deg[loop] = constraint_angle_degs * math.pi / 180

                        scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                        start_location = this_bone.get_start_point_position()
                        new_end_location = [x + y for x, y in zip(start_location, scale)]
                        this_bone.set_end_point_position(new_end_location)
                        if self.chain_length > 1:
                            self.chain[1].set_start_point_position(new_end_location)
                    else:
                        this_bone_inner_to_outer_uv = this_bone.get_direction_uv()
                        scale = [i * this_bone_length for i in this_bone_inner_to_outer_uv]
                        start_location = this_bone.get_start_point_position()
                        new_end_location = [x + y for x, y in zip(start_location, scale)]
                        this_bone.set_end_point_position(new_end_location)
                        if self.chain_length > 1:
                            self.chain[1].set_start_point_position(new_end_location)
        return self.chain

        # self.draw_chain()
