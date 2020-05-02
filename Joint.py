import Utils as Util
import numpy as np


class Joint3D:
    def __init__(self):
        self.MAX_CONSTRAINT_ANGLE_DEGS = 360
        self.MIN_CONSTRAINT_ANGLE_DEGS = -360
        # the default values
        self.rotor_constraint_degs = self.MAX_CONSTRAINT_ANGLE_DEGS
        self.hinge_clockwise_constraint_degs = self.MAX_CONSTRAINT_ANGLE_DEGS
        self.hinge_anticlockwise_constraint_degs = self.MAX_CONSTRAINT_ANGLE_DEGS
        self.joint_type = "BALL"
        self.rotation_axis_uv = [0, 0, 0]
        self.reference_axis_uv = [0, 0, 0]

    def get_MAX_CONSTRAINT_ANGLE_DEGS(self):
        return self.MAX_CONSTRAINT_ANGLE_DEGS

    def get_MIN_CONSTRAINT_ANGLE_DEGS(self):
        return self.MIN_CONSTRAINT_ANGLE_DEGS

    def validate_constraint_angle_degs(self, angle_degs):
        if angle_degs < self.MIN_CONSTRAINT_ANGLE_DEGS or angle_degs > self.MAX_CONSTRAINT_ANGLE_DEGS:
            raise Exception(
                "Constraint angles must be within the range " + str(self.MIN_CONSTRAINT_ANGLE_DEGS) + " to " + str(
                    self.MAX_CONSTRAINT_ANGLE_DEGS) + " inclusive.")

    def validate_axis(self, axis):
        if Util.length_calc(axis) <= 0:
            raise Exception("Provided axis is illegal - it has a magnitude of zero.")

    def set_as_ball_joint(self, constraint_angle_degs):
        self.validate_constraint_angle_degs(constraint_angle_degs)
        self.rotor_constraint_degs = constraint_angle_degs
        self.joint_type = "BALL"

    def set_hinge(self, joint_type, rotation_axis, clockwise_constraint_degs, anticlockwise_constraint_degs,
                  reference_axis):
        dot = Util.dot_product(rotation_axis, reference_axis)
        if Util.approximately_equal(dot, 0.0, 0.01) == 1:
            angle_degs = Util.get_angle_between_degs(rotation_axis, reference_axis)
            raise Exception(
                "The reference axis must be in the plane of the hinge rotation axis - angle between them is currently: " + str(
                    angle_degs))

        self.validate_constraint_angle_degs(clockwise_constraint_degs)
        self.validate_constraint_angle_degs(anticlockwise_constraint_degs)
        self.validate_axis(rotation_axis)
        self.validate_axis(reference_axis)

        self.hinge_clockwise_constraint_degs = clockwise_constraint_degs
        self.hinge_anticlockwise_constraint_degs = anticlockwise_constraint_degs
        self.joint_type = joint_type
        self.rotation_axis_uv = Util.normalization(rotation_axis)
        self.reference_axis_uv = Util.normalization(reference_axis)

    def set_as_global_hinge(self, global_rotation_axis, cw_constraint_degs, acw_constraint_degs, global_reference_axis):
        self.set_hinge("GLOBAL_HINGE", global_rotation_axis, cw_constraint_degs, acw_constraint_degs,
                       global_reference_axis)

    def set_as_local_hinge(self, local_rotation_axis, cw_constraint_degs, acw_constraint_degs, local_reference_axis):
        self.set_hinge("LOCAL_HINGE", local_rotation_axis, cw_constraint_degs, acw_constraint_degs,
                       local_reference_axis)

    def set_hinge_clockwise_constraint_degs(self, angle_degs):
        self.validate_constraint_angle_degs(angle_degs)
        self.hinge_clockwise_constraint_degs = angle_degs

    def get_hinge_clockwise_constraint_degs(self):
        return self.hinge_clockwise_constraint_degs

    def set_hinge_anticlockwise_constraint_degs(self, angle_degs):
        self.validate_constraint_angle_degs(angle_degs)
        self.hinge_anticlockwise_constraint_degs = angle_degs

    def get_hinge_anticlockwise_constraint_degs(self):
        return self.hinge_anticlockwise_constraint_degs

    def set_ball_joint_constraint_degs(self, angle_degs):
        self.validate_constraint_angle_degs(angle_degs)
        if self.joint_type == "BALL":
            self.rotor_constraint_degs = angle_degs
        else:
            raise Exception(
                "This joint is of type: " + self.joint_type + " - only joints of type JointType.BALL have a ball joint constraint angle.")

    def get_ball_joint_constraint_degs(self):
        return self.rotor_constraint_degs

    def set_hinge_rotation_axis(self, axis):
        self.validate_axis(axis)
        self.rotation_axis_uv = axis

    def get_hinge_rotation_axis(self):
        return self.rotation_axis_uv

    def set_hinge_reference_axis(self, reference_axis):
        self.validate_axis(reference_axis)
        self.reference_axis_uv = reference_axis

    def get_reference_axis(self):
        return self.reference_axis_uv

    def get_joint_type(self):
        return self.joint_type
