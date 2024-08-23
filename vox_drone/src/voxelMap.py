import matplotlib.pyplot as plt
import numpy as np
import pyvista as pv


class VoxelMap:
    """This class is used to store a 3D map of voxel/point cloud"""

    def __init__(self, _width = 10, _height = 10, _depth = 10, _voxel_size = 1, data = None, dtype = np.float32):
        self.voxel_size = _voxel_size
        self.dtype = dtype

        self.width, self.height, self.depth = _width, _height, _depth

        if data is not None:
            self.set_voxelMap(data)
            return
        
        placeholder = np.zeros((_width, _height, _depth),dtype=self.dtype)
        self.set_voxelMap(placeholder) #TODO adapt to large maps
        return
    

    def set_voxelMap(self, data):
        self._data_voxel_map = data

        self.shape = self._data_voxel_map.shape
        self.width, self.height, self.depth = self.shape
        return self


    


    def plot3D(self, title = "Voxel Map"):
        

        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(title)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.voxels(self._data_voxel_map, edgecolor='k')
        """
        return


    def print(self):
        for i in range(self.width):
            for j in range(self.height):
                for k in range(self.depth):
                    print(self._data_voxel_map[i][j][k], end = " ")
                print()
            print()
        return
    
    def set_voxel(self, point, value):
        x = int(point[0])
        y = int(point[1])
        z = int(point[2])
        if 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth:
            self._data_voxel_map[x][y][z] = (int)(np.ceil(value))
        else:
            raise IndexError("Coordinates out of range")
        return

    def get_voxel(self, point):
        x = int(point[0])
        y = int(point[1])
        z = int(point[2])
        if 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth:
            return self._data_voxel_map[x][y][z]
        else:
            raise IndexError("Coordinates out of range")
    
    def binariseMap(self, threshold = 0.5):
        self._data_voxel_map = np.where(self._data_voxel_map > threshold, 1, 0)
        return self
    
    def get_voxelMap(self):
        return self._data_voxel_map
    
    

    def get_memory_size(self):
        return self._data_voxel_map.nbytes
    



if __name__ == "__main__":
    print("This is a module, it should not be run alone")
  