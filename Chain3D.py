import math
from pycg3d.cg3d_point import CG3dPoint
import Joint3D as Joint
import Bone as bone


class Chain3d:
    def __init__(self, target_position, base_bone_constraint_type, base_bone_constraint_uv):
        self.target_position = target_position
        self.chain = []
        self.solve_distance_threshold = 0.01
        self.max_iteration_attempts = 20
        self.min_iteration_change = 0.01
        self.fixed_base_location = CG3dPoint(0, 0, 0)
        self.chain_length = 0
        self.base_bone_constraint_type = base_bone_constraint_type
        self.base_bone_constraint_uv = base_bone_constraint_uv
        self.current_solve_distance = 100000000

    def update_chain_length(self):
        self.chain_length = 0
        for i in self.chain:
            self.chain_length += 1

    def add_bone(self, bone):
        self.chain.append(bone)
        # if it is the base bone:
        if len(self.chain) == 1:
            self.fixed_base_location = bone.get_start_point
            self.base_bone_constraint_uv = bone.get_direction_uv

        self.update_chain_length()

    def add_consecutive_bone(self, bone):
        dir = bone.get_direction_uv()
        len = bone.get_live_length()
        if self.chain.__len__()!=0:
            prev_bone_end = self.chain[self.chain_length-1].get_end_point()
            bone.set_start_point(prev_bone_end)
            bone.set_end_point(prev_bone_end+dir*len)
            self.add_bone(bone)
        else:
            raise Exception("You cannot add the base bone to a chain using this method as it does not provide a start location.")

    def add_consecutive_freely_rotating_hinged_bone(self, bone_direction_uv, bone_length, joint_type,
                                                    hinge_rotation_axis):
        self.add_consecutive_hinged_bone(bone_direction_uv,bone_length,joint_type,180,180,genPerpendicularVectorQuick(hinge_rotation_axis))

    def add_consecutive_hinged_bone(self, bone_direction_uv, bone_length, joint_type, hinge_rotation_axis,
                                    clockwise_degs,
                                    anticlockwise_deg, hinge_reference_axis):
        if self.chain_length==0:
            raise Exception("You must add a basebone before adding a consectutive bone.")

        bone_direction_uv = bone_direction_uv.normalize()
        hinge_rotation_axis = hinge_rotation_axis.normalize()
        prev_bone_end = self.chain[self.chain_length - 1].get_end_point()
        m_bone = bone.Bone3D(prev_bone_end,bone_direction_uv,bone_length)
        m_joint = Joint.Joint3D()
        if joint_type == "GLOBAL_HINGE":
            m_joint.set_as_global_hinge(hinge_rotation_axis,clockwise_degs,anticlockwise_deg,hinge_reference_axis)
        elif joint_type == "LOCAL_HINGE":
            m_joint.set_as_local_hinge(hinge_rotation_axis,clockwise_degs,anticlockwise_deg,hinge_reference_axis)
        else:
            raise Exception("Hinge joint types may be only JointType.GLOBAL_HINGE or JointType.LOCAL_HINGE.")

        m_bone.set_joint(m_joint)
        self.add_bone(m_bone)

    def add_consecutive_rotor_constrained_bone(self, bone_direction_uv, bone_length, constraint_angle_degs):
        if self.chain.__len__()==0:
            raise Exception("Add a basebone before attempting to add consectuive bones.")
        m_bone = bone.Bone3D(self.chain[self.chain_length-1].get_end_point(),bone_direction_uv,bone_length)
        m_bone.set_ball_joint_constraint_degs(constraint_angle_degs)
        self.add_bone(m_bone)

    def get_base_bone_constraint_uv(self):
        return self.base_bone_constraint_uv

    def get_base_bone_constraint_type(self):
        if self.base_bone_constraint_type == None:
            raise Exception('Cannot return the basebone constraint when the basebone constraint type is NONE.')
        return self.base_bone_constraint_type

    def get_base_location(self):
        return self.chain[0].get_start_point()

    def get_bone(self, bone_number):
        return self.chain[bone_number]

    def get_chain(self):
        return self.chain

    def get_chain_length(self):
        return self.chain_length

    def get_end_effector_location(self):
        return self.chain[self.chain_length - 1].get_end_point()

    def remove_bone(self, bone_number):
        if bone_number < self.chain_length:
            self.chain.remove(self.chain[bone_number])
            self.update_chain_length()
        else:
            raise Exception(
                "Bone " + bone_number + " does not exist to be removed from the chain. Bones are zero indexed.")

    def set_rotor_basebone_constraint(self, rotor_type, constraint_axis, angle_degs):
        # Sanity checking
        if len(self.chain) == 0:
            raise Exception("Chain must contain a basebone before we can specify the basebone constraint type.")

        if len(constraint_axis) <= 0.0:
            raise Exception("Constraint axis cannot be zero.")

        if angle_degs < 0.0:
            angle_degs = 0.0

        if angle_degs > 180.0:
            angle_degs = 180.0

        if rotor_type != "GLOBAL_ROTOR" or rotor_type != "LOCAL_ROTOR":
            raise Exception("The only valid rotor types for this method are GLOBAL_ROTOR and LOCAL_ROTOR.")

        #  Set the constraint type, axis and angle
        self.base_bone_constraint_type = rotor_type
        self.base_bone_constraint_uv = constraint_axis.normalised()
        self.chain[0].getJoint().setAsBallJoint(angle_degs)

    def set_hinge_basebone_constraint(self, hinge_type, hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                      hinge_reference_axis):
        # Sanity checking
        if len(self.chain) == 0:
            raise Exception("Chain must contain a basebone before we can specify the basebone constraint type.")

        if len(hinge_rotation_axis) <= 0.0:
            raise Exception("Constraint axis cannot be zero.")

        if len(hinge_reference_axis) <= 0.0:
            raise Exception("Constraint axis cannot be zero.")

        if (hinge_rotation_axis * hinge_reference_axis) == 0:
            raise Exception(
                "The hinge reference axis must be in the plane of the hinge rotation axis, that is, they must be perpendicular")

        if hinge_type != "GLOBAL_HINGE" or hinge_type != "LOCAL_HINGE":
            raise Exception("The only valid rotor types for this method are GLOBAL_HINGE and LOCAL_HINGE.")

        #  Set the constraint type, axis and angle
        self.base_bone_constraint_type = hinge_type
        self.base_bone_constraint_uv = hinge_rotation_axis.normalised()

        if hinge_type == "GLOBAL_HINGE":
            self.chain[0].getJoint().setHinge(hinge_type, hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                              hinge_reference_axis)
        else:
            self.chain[0].getJoint().setHinge(hinge_type, hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                              hinge_reference_axis)

    def set_freely_rotating_global_hinged_basebone(self, hinge_rotation_axis):
        self.chain[0].getJoint().setHinge("GLOBAL_HINGE", hinge_rotation_axis, 180, 180,
                                          genPerpendicularVectorQuick(hinge_rotation_axis))

    def set_freely_rotating_local_hinged_basebone(self, hinge_rotation_axis):
        self.chain[0].getJoint().setHinge("LOCAL_HINGE", hinge_rotation_axis, 180, 180,
                                          genPerpendicularVectorQuick(hinge_rotation_axis))

    def set_local_hinged_basebone(self, hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                  hinge_reference_axis):
        self.chain[0].getJoint().setHinge("LOCAL_HINGE", hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                          hinge_reference_axis)

    def set_global_hinged_basebone(self, hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                   hinge_reference_axis):
        self.chain[0].getJoint().setHinge("GLOBAL_HINGE", hinge_rotation_axis, cw_constraint_degs, acw_constraint_axis,
                                          hinge_reference_axis)

    def set_basebone_constraint_uv(self, constraint_uv):
        if len(constraint_uv) == 0:
            raise Exception("direction unit vector cannot be zero")
        self.base_bone_constraint_uv = constraint_uv.normalize()

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

    def forward(self):
        # Forward Pass: from end effector to basebone
        for count in range(self.chain_length - 1, 0, -1):
            this_bone_length = self.chain[count].length()
            this_bone_joint_type = self.chain[count].get_joint_type()

            # if it is NOT end-effector:
            if count != self.chain_length - 1:
                outer_bone_outer_to_inner_uv = self.chain[count + 1].get_direction_uv()
                this_bone_outer_to_inner_uv = self.chain[count].get_direction_uv()

                if this_bone_joint_type == "BALL":
                    # Constrain to relative angle between this bone and the outer bone if required!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!(here needs improve by cone)
                    angle_between_bones = Utils.get_angle_between_degs(this_bone_outer_to_inner_uv,
                                                                       outer_bone_outer_to_inner_uv)
                    constraint_angle_deg = self.chain[count].get_ball_joint_constraint_degs()
                    if angle_between_bones > constraint_angle_deg:
                        this_bone_outer_to_inner_uv = Utils.get_angle_limited_uv(this_bone_outer_to_inner_uv,
                                                                                 outer_bone_outer_to_inner_uv,
                                                                                 constraint_angle_deg)

                elif this_bone_joint_type == "GLOBAL_HINGE":
                    # Project this bone outer to inner direction onto the hinge rotation axis
                    this_bone_outer_to_inner_uv = Utils.projectOntoPlane(this_bone_outer_to_inner_uv,
                                                                         self.chain[count].get_hinge_rotation_axis)

                elif this_bone_joint_type == "LOCAL_HINGE":

                    if count > 0:
                        m = Utils.create_rotation_matrix(self.chain[count - 1].getDirectionUV())
                        relative_hinge_rotation_axis = Utils.times(
                            m, self.chain[count].get_hinge_rotation_axis().normalise())
                    else:
                        raise Exception("no Local hinge accepted for base bone")
                        # base bone! no relative hinge is accepted:

                    this_bone_outer_to_inner_uv = Utils.project_onto_plane(this_bone_outer_to_inner_uv,
                                                                           relative_hinge_rotation_axis)

                # At this stage we have an outer to inner unit vector for this bone which is whithin this constraint
                # so we can find the start location of this joint
                new_start_location = self.chain[count].get_end_point() + this_bone_outer_to_inner_uv * this_bone_length
                self.chain[count].set_start_point(new_start_location)

                # if it isn't the base bone we set end point of the previous bone to new start location of this bone
                if count > 0:
                    self.chain[count - 1].set_end_point(new_start_location)

            # IF it is end effector
            else:
                # if it is the end effector bone:
                self.chain[count].set_end_point(target)
                this_bone_outer_to_inner_uv = self.chain[count].get_direction_uv()
                if this_bone_joint_type == "BALL":
                    break
                elif this_bone_joint_type == "GLOBAL_HINGE":
                    this_bone_outer_to_inner_uv = Utils.projectOntoPlane(this_bone_outer_to_inner_uv,
                                                                         self.chain[count].get_hinge_rotation_axis)
                elif this_bone_joint_type == "LOCAL_HINGE":
                    # Construct a rotation matrix based on previous bones
                    m = Utils.createRotationMatrix(self.chain[count - 1].getDirectionUV())
                    # transform the hinge rotation axis into the previous bones frame of reference
                    relative_hinge_rotation_axis = Utils.times(m,
                                                               self.chain[count].get_hinge_rotation_axis().normalise())
                    # project this bone outer to inner vector to the plane described by relative hinge rotation axis
                    this_bone_outer_to_inner_uv = Utils.projectOntoPlane(this_bone_outer_to_inner_uv,
                                                                         relative_hinge_rotation_axis)

                # At this stage we have an outer to inner unit vector for this bone which is whithin this constraint
                # so we can find the start location of this joint
                new_start_location = target + this_bone_outer_to_inner_uv * this_bone_length
                self.chain[count].set_start_point(new_start_location)

                # if it isn't the base bone we set end point of the previous bone to new start location of this bone
                if count > 0:
                    self.chain[count - 1].set_end_point(new_start_location)

    def backward(self):
        for count in range(0, 1, self.chain_length):
            this_bone_length = self.chain[count].length()
            # if it is NOT base bone:
            if count != 0:
                this_bone_inner_to_outer_uv = self.chain[count].get_direction_uv()
                previous_bone_inner_to_outer_uv = self.chain[count - 1].get_direction_uv()
                this_bone_joint_type = self.chain[count].get_joint_type()

                if this_bone_joint_type == "BALL":
                    # Constrain to relative angle between this bone and the outer bone if required!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!(here needs improve by cone)
                    angle_between_bones = math.degrees(math.acos(
                        (this_bone_inner_to_outer_uv * previous_bone_inner_to_outer_uv) / (
                                this_bone_length * self.chain[count - 1].length())))
                    constraint_angle_deg = self.chain[count].get_ball_joint_constraint_degs()
                    if angle_between_bones > constraint_angle_deg:
                        this_bone_outer_to_inner_uv = Utils.get_angle_limited_uv(this_bone_inner_to_outer_uv,
                                                                                 previous_bone_inner_to_outer_uv,
                                                                                 constraint_angle_deg)

                elif this_bone_joint_type == "GLOBAL_HINGE":
                    # Project this bone inner to outer direction onto the hinge rotation axis
                    hinge_rotation_axis = self.chain[count].get_hinge_rotation_axis()
                    this_bone_inner_to_outer_uv = Utils.projectOntoPlane(this_bone_inner_to_outer_uv,
                                                                         hinge_rotation_axis)

                    cw_constraint_degs = -self.chain[count].get_hinge_clockwise_constraint_degs()
                    acw_constraint_degs = self.chain[count].get_hinge_anticlockwise_constraint_degs()

                    if abs(cw_constraint_degs - (-FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS)) > 0.001 and abs(
                            acw_constraint_degs - (-FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS)) > 0.001:
                        hinge_reference_axis = self.chain[count].get_hinge_reference_axis()
                        # get the signed angle(about the hinge rotation axis) between the hinge reference axis and hinge rotation aligned bone uv
                        # ACW rotation is positive and cw rotation is negative

                        signed_angle_degs = Utils.get_signed_angle_between_degs(hinge_reference_axis,
                                                                                this_bone_inner_to_outer_uv,
                                                                                hinge_rotation_axis)
                        # make our bone inner to outer uv the hinge reference axis rotated by its maximum clockwise or axclockwise rotation as required

                        if signed_angle_degs > acw_constraint_degs:
                            this_bone_inner_to_outer_uv = Utils.rotate_about_axis_degs(hinge_reference_axis,
                                                                                       acw_constraint_degs,
                                                                                       hinge_rotation_axis).normalised()

                        elif signed_angle_degs < cw_constraint_degs:
                            this_bone_inner_to_outer_uv = Utils.rotate_about_axis_degs(hinge_reference_axis,
                                                                                       cw_constraint_degs,
                                                                                       hinge_rotation_axis).normalised()

                elif this_bone_joint_type == "LOCAL_HINGE":
                    # transform hinge rotation axis to be relative to the previous bone in chain
                    hinge_rotation_axis = self.chain[count].get_hinge_rotation_axis()
                    # constructing a rotation matrix based on the previous bone's direction
                    m = Utils.create_rotation_matrix(previous_bone_inner_to_outer_uv)
                    # transform the hinge rotation axis into the previous bone's frame of reference
                    relative_hinge_rotation_axis = Utils.times(m, (hinge_rotation_axis).normalise())
                    # project this bone direction onto the plane described onto the plane described by rotation axis
                    # the returned vector is normalised
                    this_bone_inner_to_outer_uv = Utils.project_onto_plane(this_bone_inner_to_outer_uv,
                                                                           relative_hinge_rotation_axis);
                    # constrain rotation about reference axis if required
                    cw_constraint_degs = -self.chain[count].get_hinge_clockwise_constraint_degs()
                    acw_constraint_degs = self.chain[count].get_hinge_anticlockwise_constraint_degs()

                    if abs(cw_constraint_degs - (-FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS)) > 0.001 and abs(
                            acw_constraint_degs - (-FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS)) > 0.001:
                        # calc the reference axis in local space
                        relative_hinge_reference_axis = Utils.times(m, self.chain[
                            count].get_hinge_reference_axis.normalise())
                        # get the signed angle(about the hinge rotation axis) between the hinge reference axis and hinge rotation aligned bone uv
                        # ACW rotation is positive and cw rotation is negative

                        signed_angle_degs = Utils.get_signed_angle_between_degs(relative_hinge_reference_axis,
                                                                                this_bone_inner_to_outer_uv,
                                                                                relative_hinge_rotation_axis)
                        # make our bone inner to outer uv the hinge reference axis rotated by its maximum clockwise or axclockwise rotation as required

                        if signed_angle_degs > acw_constraint_degs:
                            this_bone_inner_to_outer_uv = Utils.rotate_about_axis_degs(relative_hinge_reference_axis,
                                                                                       acw_constraint_degs,
                                                                                       relative_hinge_rotation_axis).normalised()

                        elif signed_angle_degs < cw_constraint_degs:
                            this_bone_inner_to_outer_uv = Utils.rotate_about_axis_degs(relative_hinge_reference_axis,
                                                                                       cw_constraint_degs,
                                                                                       relative_hinge_rotation_axis).normalised()

                new_end_location = self.chain[count].get_start_point() + this_bone_inner_to_outer_uv * this_bone_length
                self.chain[count].set_end_point(new_end_location)

                # if it is not end effector bone
                if count < self.chain_length - 1:
                    self.chain[count + 1].set_start_point(new_end_location)
            # if it is base bone
            else:
              self.chain[count].set_start_point = self.fixed_base_location


              if self.base_bone_constraint_type == "NONE":
                #
                new_end_location = self.chain[count].get_start_point() + this_bone_inner_to_outer_uv * this_bone_length
                self.chain[count].set_end_point(new_end_location)

                # if more bone exists
                if self.chain_length >1:
                  self.chain[count + 1].set_start_point(new_end_location)

              elif self.base_bone_constraint_type == "GLOBAL_ROTOR":
                this_bone_inner_to_outer_uv = self.chain[count].get_direction_uv()
                angle_between_degs = Utils.get_angle_between_degs(self.base_bone_constraint_uv,this_bone_inner_to_outer_uv)
                constraint_angle_degs = self.chain[count].get_ball_joint_constraint_degs()
                if angle_between_degs>constraint_angle_degs:
                  Utils.get_angle_limited_uv(this_bone_inner_to_outer_uv,self.base_bone_constraint_uv,constraint_angle_degs)

                new_end_location = self.chain[count].get_start_point() + this_bone_inner_to_outer_uv * this_bone_length
                self.chain[count].set_end_point(new_end_location)

                # if more bone exists
                if self.chain_length > 1:
                  self.chain[count + 1].set_start_point(new_end_location)

              elif self.base_bone_constraint_type == "GLOBAL_HINGE":
                hinge_rotation_axis  =  self.chain[count].get_hinge_rotation_axis();
                cw_constraint_degs   = -self.chain[count].get_hinge_clockwise_constraint_degs()
                acw_constraint_degs  =  self.chain[count].get_hinge_aclockwise_constraint_degs()
                # get the inner to outer direction of this bone and project it onto global hinge rotation axis
                this_bone_inner_to_outer_uv = Utils.project_onto_plane(self.chain[count].get_direction_uv(),hinge_rotation_axis);

                if abs(cw_constraint_degs - (-FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS)) > 0.01 and abs(
                        acw_constraint_degs - (-FabrikJoint3D.MAX_CONSTRAINT_ANGLE_DEGS)) > 0.01:
                    hinge_reference_axis = self.chain[count].get_hinge_reference_axis()
                    signed_angle_degs = Utils.get_signed_angle_between_degs(hinge_reference_axis,this_bone_inner_to_outer_uv,hinge_rotation_axis)

                    if signed_angle_degs > acw_constraint_degs:
                      this_bone_inner_to_outer_uv = Utils.rotate_about_axis_degs(hinge_reference_axis,acw_constraint_degs,hinge_rotation_axis).normalise()

                    elif signed_angle_degs < cw_constraint_degs:
                      this_bone_inner_to_outer_uv = Utils.rotate_about_axis_degs(hinge_reference_axis,cw_constraint_degs,hinge_rotation_axis).normalise()

                new_end_location = self.chain[count].get_start_point() + this_bone_inner_to_outer_uv * this_bone_length
                self.chain[count].set_end_point(new_end_location)

                # if more bone exists
                if self.chain_length > 1:
                  self.chain[count + 1].set_start_point(new_end_location)

    def solve_fabrik_ik(self, target):

        if self.chain.__len__() == 0:
            raise Exception("It makes no sense to solve an IK chain with zero bones.")

        dist_to_target = Util.distance_between(self.chain[self.chain_length - 1].get_end_point, target)
        while dist_to_target>self.get_solve_distance_threshold():
            self.forward()
            self.backward()
            dist_to_target = Util.distance_between(self.chain[self.chain_length - 1].get_end_point, target)

