from fabrik_chain_3d.singleton import Singleton


class OutputWriter(object):
    __metaclass__ =  Singleton
    def __init__(self, base_address="./output/", joints_file_address="joints-position.txt"):
        self.base_address = base_address
        self.joints_file_address = base_address + joints_file_address

    def joint_writer(self):
        return open(self.joints_file_address, "w")