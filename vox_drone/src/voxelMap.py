import matplotlib.pyplot as plt
import numpy as np

class VoxelMap:
    """This class is used to store a 3D map of voxel/point cloud"""

    def __init__(self, _width = 10, _height = 10, _depth = 10, _voxel_size = 1, data = None):
        self.voxel_size = _voxel_size

        if data is not None:
            self.set_voxelMap(data)
            return
        
        self.set_voxelMap(np.zeros((_width, _height, _depth)))
        return


    


    def plot3D(self, title = "Voxel Map"):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(title)
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.voxels(self._data, edgecolor='k')
        return


    def print(self):
        for i in range(self.width):
            for j in range(self.height):
                for k in range(self.depth):
                    print(self._data[i][j][k], end = " ")
                print()
            print()
        return
    
    def set_voxel(self, point, value):
        x = int(point[0])
        y = int(point[1])
        z = int(point[2])
        if 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth:
            self._data[x][y][z] = (int)(np.ceil(value))
        else:
            raise IndexError("Coordinates out of range")
        return

    def get_voxel(self, point):
        x = int(point[0])
        y = int(point[1])
        z = int(point[2])
        if 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth:
            return self._data[x][y][z]
        else:
            raise IndexError("Coordinates out of range")
        return 0
    
    def binariseMap(self, threshold = 0.5):
        self._data = np.where(self._data > threshold, 1, 0)
        return self
    
    def get_voxelMap(self) -> np.ndarray:
        return self._data
    
    def set_voxelMap(self, data):
        self._data = data
        self.width = (int)(data.shape[0])
        self.height = (int)(data.shape[1])
        self.depth = (int)(data.shape[2])
        self.shape = data.shape
        return self

    def get_memory_size(self):
        return self._data.nbytes
    



def testVoxelFile():
    voxMap1 = VoxelMap(10, 10, 10)
    voxMap1.read_voxelMap_from_file("data/test2.voxmap")
    voxMap1.print()
    voxMap1.write_voxelMap_to_file("data/test3.voxmap")
    return


if __name__ == "__main__":
    print("This is a module, it should not be run alone")
    print("testing the VoxelMap class")

    testVoxelFile()


    