from pycg3d.cg3d_point import CG3dPoint
from pycg3d.cg3d_point import CG3dVector
import Joint as Joint
import Utils as Util

X_AXIS = CG3dVector(1.0, 0.0, 0.0)
Y_AXIS = CG3dVector(0.0, 1.0, 0.0)
Z_AXIS = CG3dVector(0.0, 0.0, 1.0)

class Bone3D:
    def __init__(self, start,direction_uv ,length,is_fixed_bone):
        self.start_point = start
        self.is_fixed_bone = is_fixed_bone
        self.end_point = CG3dPoint(0, 0, 0)
        self.length = length
        self.calculate_end(direction_uv)
        m_joint = Joint.Joint3D()
        self.joint = m_joint
        self.direction_uv = direction_uv

    def set_length(self,length):
        self.length = length

    def get_live_length(self):
        return Util.get_distance_between(self.start_point,self.end_point)

    def get_length(self):
        return self.length

    def set_start_point(self,start):
        self.start_point = start

    def get_start_point(self):
        return self.start_point

    def calculate_end(self,direction_uv):

        scale_direction = CG3dVector(direction_uv[0]*self.length,
                                     direction_uv[1]*self.length,
                                     direction_uv[2]*self.length)
        self.end_point = self.start_point.add(scale_direction)

    def set_end_point(self,end):
        self.end_point = end

    def get_end_point(self):
        return self.end_point

    def set_joint(self,joint):
        self.joint = joint

    def get_joint(self):
        return self.joint

    def get_joint_type(self):
        return self.joint.get_joint_type()

    def set_hinge_joint_clockwise_constraint_degs(self,angle_degs):
        self.joint.set_hinge_clockwise_constraint_degs(angle_degs)

    def get_hinge_joint_clockwise_constraint_degs(self):
        return self.joint.get_hinge_clockwise_constraint_degs()

    def set_hinge_joint_anticlockwise_constraint_degs(self,angle_degs):
        self.joint.set_hinge_anticlockwise_constraint_degs(angle_degs)

    def get_hinge_joint_anticlockwise_constraint_degs(self):
        return self.joint.get_hinge_anticlockwise_constraint_degs()

    def set_ball_joint_constraint_degs(self,angle_degs):
        if angle_degs<0 or angle_degs>180:
            raise Exception("Rotor constraints for ball joints must be in the range 0.0f to 180.0f degrees inclusive.")
        self.joint.set_ball_joint_constraint_degs(angle_degs)

    def get_ball_joint_constraint_degs(self):
        return self.joint.get_ball_joint_constraint_degs()

    def get_direction_uv(self):
        return self.end_point.sub(self.start_point)

    def get_global_pitch_degs(self):
        bone_uv = self.get_direction_uv()
        x_projected = Util.project_on_to_plane(bone_uv,X_AXIS)
        pitch = Util.get_angle_between_degs(Z_AXIS, x_projected)
        if x_projected[1] < 0.0:
            return -pitch
        else:
            return pitch

    def get_global_yaw_degs(self):
        bone_uv = self.get_direction_uv()
        y_projected = Util.project_on_to_plane(bone_uv,Y_AXIS)
        yaw = Util.get_angle_between_degs(Z_AXIS, y_projected)
        if y_projected[0] < 0.0:
            return -yaw
        else:
            return yaw

    def is_fix_bone(self):
        return self.is_fixed_bone

    def get_fixed_bone_direction_uv(self):
        return self.direction_uv



