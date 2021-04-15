from fabrik_chain_3d import Chain as Chain, Bone as Bone, Utils as Util, FABRIK as fabrik
import math
import sys
import fabrik_chain_3d.Visualization as draw_chain
sys.path.append('..')




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