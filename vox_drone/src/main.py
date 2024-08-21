# This is the main file for the project
# The purpose of this project is to test the use of voxel for 3D representation of an environment and their use in communication between agents
# One way do to an efficient communication could be the compression of the voxel and the use of AI to do so

# the voxel Map are stored in .voxmap files

# repere : x, y, z
# x : horizontal
# y : vertical
# z : depth


from voxFftConverter import testfullConversion

import numpy as np
import pyvista as pv
from pyvista import examples



# Define some helpers - ignore these and use your own data if you like!
def generate_points(subset=0.02):
    """A helper to make a 3D NumPy array of points (n_points by 3)."""
    # Columns are (X, Y, Z)
    dataset = examples.download_lidar()
    ids = np.random.randint(low=0, high=dataset.n_points - 1, size=int(dataset.n_points * subset))
    return dataset.points[ids]


def pointcloudTest():
    points = generate_points()
    # PyVista conversion
    point_cloud = pv.PolyData(points)
    print("point cloud conversion check : ",np.allclose(points, point_cloud.points))

    # Plotting
    point_cloud.plot(eye_dome_lighting=True)




if __name__ == '__main__':
    testfullConversion(True)
    #pointcloudTest()

    


    


