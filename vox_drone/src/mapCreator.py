import numpy as np
import matplotlib.pyplot as plt
from voxelMap import VoxelMap

class VoxelMapCreator:
    def __init__(self):
        pass

    def create_test_cloud(self, center, size, num_points):
        x = np.random.randint(center[0] - size[0], center[0] + size[0], num_points)
        y = np.random.randint(center[1] - size[1], center[1] + size[1], num_points)
        z = np.random.randint(center[2] - size[2], center[2] + size[2], num_points)
        return np.column_stack((x, y, z))
    
    def voxelise_3Dcloud(self, cloud, voxel_size):
        # create a voxel map from a cloud of points
        # cloud : 3D numpy array
        # voxel_size : int
        # return : VoxelMap
        # The voxel map is a 3D matrix where each voxel is a cube of size voxel_size
        # The value of the voxel is 0 if the voxel is empty and 1 if the voxel is full
        
        # get the size of the voxel map
        size = np.max(cloud, axis=0) + 1
        voxel_map = VoxelMap(size[0], size[1], size[2], voxel_size)
        
        # fill the voxel map
        for point in cloud:
            voxel_map.set_voxel(point, 1)
        
        return voxel_map


    def create_voxelMap_from_sinfunc(self, size, voxel_size):
        size = 20
        voxel_map = VoxelMap(size, size, size, voxel_size)
        a = np.zeros((size, size, size))
        [X, Y, Z] = np.meshgrid(2 * np.pi * np.arange(size) / 12,
                            2 * np.pi * np.arange(size) / 20 + 10,
                            2 * np.pi * np.arange(size) / 56 + 15)
        
        S = np.sin(X) + np.cos(Y) + np.tan(Z) + np.random.uniform(0, 1, X.shape)
        # binarise the 3D matrix
        S = np.where(S > 0.5, 1, 0)

        # reverse Z axis to look better
        S = np.flip(S, axis=2)

        voxel_map.set_voxelMap(S)
        return voxel_map




    
    def plot_3Dcloud(self, cloud):
        # plot a cloud of points
        # cloud : 3D numpy array
        # return : None
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2])
        return

def testCloudToVoxelMap():
    mapper = VoxelMapCreator()

    # create a cloud of points with scipy
    cloud = mapper.create_test_cloud([5, 5, 5], [2, 2, 2], 30)

    # plot the cloud of points
    mapper.plot_3Dcloud(cloud)

    # create a voxel map from the cloud of points
    voxel_map = mapper.voxelise_3Dcloud(cloud, 1)
    voxel_map.print()
    voxel_map.plot3D()
    voxel_map.save_voxelMap_to_file("data/cloud1.voxmap")
    return


if __name__ == "__main__":
    testCloudToVoxelMap()
    plt.show()