# This is the main file for the project
# The purpose of this project is to test the use of voxel for 3D representation of an environment and their use in communication between agents
# One way do to an efficient communication could be the compression of the voxel and the use of AI to do so

# the voxel Map are stored in .voxmap files

# repere : x, y, z
# x : horizontal
# y : vertical
# z : depth

#from voxFftConverter import testfullConversion
from mapCreator import VoxelMapCreator
import matplotlib.pyplot as plt

import numpy as np
import pyvista as pv
from pyvista import examples


def pyvista_example_points_cloud(subset=0.02): #TODO change to directly give voxel map and migrate to mapCreator.py
    """A helper to make a 3D NumPy array of points (n_points by 3)."""
    dataset = examples.download_lidar()
    ids = np.random.randint(low=0, high=dataset.n_points - 1, size=int(dataset.n_points * subset))
    return dataset.points[ids]




if __name__ == '__main__':
    points = pyvista_example_points_cloud()

    #get voxel map from pyvista example
    voxel_map = VoxelMapCreator().voxelise_3Dcloud(pyvista_example_points_cloud(),1)

    
    #get point cloud from voxel map
    pc = VoxelMapCreator().voxel_to_point_cloud(voxel_map)
    #print(pc.shape)
    #print(pc)

    pv_pc = pv.PolyData(pc)
    pv_pc.plot(eye_dome_lighting=True)






