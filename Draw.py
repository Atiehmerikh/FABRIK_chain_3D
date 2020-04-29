import numpy as np
import matplotlib.pyplot as plt



class Draw:
    def __init__(self, chain):
        self.chain = chain

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
        x = []
        y = []
        z = []
        for i in range(len(start_locations)):
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
        for i in range(0, len(self.chain)):
            start_loc = self.chain[i].start_point
            # end_loc = self.chain.get_bone(i).set_end_point()
            start_locations.append(start_loc)
            # end_locations.append(end_loc)
        end_effector_bone = self.chain[len(self.chain)-1].end_point
        start_locations.append(end_effector_bone)
        return start_locations

    def drawing(self):
        coordinate = self.fill_array(self.points_retrieval())
        x_prime = coordinate[0]
        y_prime = coordinate[1]
        z_prime = coordinate[2]

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot3D(x_prime, y_prime, z_prime, color='red')
        plt.show()