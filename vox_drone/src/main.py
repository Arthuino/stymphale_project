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
from voxFftConverter import VoxFftConverter
import matplotlib.pyplot as plt
from complex32 import Complex32
import sys

import numpy as np
import pyvista as pv
from pyvista import examples


def pyvista_example_points_cloud(subset=0.02): 
    """A helper to make a 3D NumPy array of points (n_points by 3)."""
    dataset = examples.download_lidar()
    ids = np.random.randint(low=0, high=dataset.n_points - 1, size=int(dataset.n_points * subset))
    return dataset.points[ids]

def size_comparator(size_1:int,size_2:int)->float:
    return (size_1-size_2)*100/size_1

def rmse_pc(pc1,pc2):
    if pc1.shape != pc2.shape:
        raise ValueError("Point clouds must have the same shape")
    
    diff = pc1 - pc2
    squared_diff = np.square(diff)
    mse = np.mean(np.sum(squared_diff, axis=1))
    return np.sqrt(mse)

if __name__ == '__main__':
    #get voxel map from pyvista example
    point_cloud_1 = pyvista_example_points_cloud()
    # Compression

    point_cloud_2 = point_cloud_1.astype(np.float32)

    pc_1_size = sys.getsizeof(point_cloud_1)
    pc_2_size = sys.getsizeof(point_cloud_2)

    print("rmse ",rmse_pc(point_cloud_1,point_cloud_2))
    print("size 1 ",pc_1_size)
    print("type1 ",type(point_cloud_1))
    print("size 2 ",pc_2_size)

    print(f"compression {size_comparator(pc_2_size,pc_1_size)} %")

    pv_pc_1 = pv.PolyData(point_cloud_1)
    

    pv_pc = pv.PolyData(point_cloud_2)
    pv_pc.plot(eye_dome_lighting=True)
    pv_pc_1.plot(eye_dome_lighting=True)
