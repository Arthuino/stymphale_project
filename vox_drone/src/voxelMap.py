import matplotlib.pyplot as plt
import numpy as np

class VoxelMap:
    def __init__(self, _width = 10, _height = 10, _depth = 10, _voxel_size = 1, data = None):
        self.voxel_size = _voxel_size

        if data is not None:
            self.set_voxelMap(data)
            return
        
        self.set_voxelMap(np.zeros((_width, _height, _depth)))
        return


    def read_voxelMap_from_file(self, file_path):
        "Read a 3D matrix from a .voxmap file"
        
        with open(file_path, 'r') as file:
            # Read the size of the matrix
            size = file.readline().split()
            size = [int(i) for i in size]
            
            matrix = np.zeros((size[0], size[1], size[2])).astype(int)
            # Read the values of the matrix
            for i in range(size[2]):
                for j in range(size[1]):
                    line = file.readline().split()
                    for k in range(size[0]):
                        matrix[i][j][k] = int(line[k])
                file.readline()
        size = matrix.shape*self.voxel_size
        voxel_map = np.repeat(
                            np.repeat(
                                np.repeat(matrix, self.voxel_size, axis=0),
                                         self.voxel_size, axis=1),
                                          self.voxel_size, axis=2)
        self._data = voxel_map
        return self

    
    def save_voxelMap_to_file(self, file_path):
        "Write a 3D matrix to a file, considering the voxel size."
        # Create a 3D matrix
        # .voxmap file format
        # The first line is the size of the matrix : X, Y, Z
        # The next lines are the values of the matrix
        # 0 = empty, other = full

        # step 1 : reduce the size of the matrix
        matrix = self._data[::self.voxel_size, ::self.voxel_size, ::self.voxel_size]
        with open(file_path, 'w') as file:
            file.write(str(matrix.shape[0]) + " " + str(matrix.shape[1]) + " " + str(matrix.shape[2]) + "\n")
            for i in range(matrix.shape[0]):
                for j in range(matrix.shape[1]):
                    for k in range(matrix.shape[2]):
                        file.write(str(matrix[i][j][k]) + " ")
                    file.write("\n")
                file.write("\n")
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
    
    def get_voxelMap(self):
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


    