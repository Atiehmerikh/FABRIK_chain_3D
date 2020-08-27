from fabrik_chain_3d.singleton import Singleton


class OutputWriter(metaclass=Singleton):
    def __init__(self, base_address="./output", joints_file_address="joints-position.txt"):
        self.base_address = base_address
        self.angles_file_address = base_address + joints_file_address

    def joint_writer(self):
        return open(self.angles_file_address, "w")