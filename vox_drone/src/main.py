# This is the main file for the project
# The purpose of this project is to test the use of voxel for 3D representation of an environment and their use in communication between agents
# One way do to an efficient communication could be the compression of the voxel and the use of AI to do so

# the voxel Map are stored in .voxmap files

# repere : x, y, z
# x : horizontal
# y : vertical
# z : depth


import numpy as np
import matplotlib.pyplot as plt
from voxelMap import VoxelMap
from mapCreator import VoxelMapCreator
from voxCompressor import VoxCompressor


def main():
    print("VOX Drone Project")
    print("Developement test program")
    testFullCompressionProcess()

    


def testFullCompressionProcess():
    mapper = VoxelMapCreator()
    testsize = 20

    #map1.read_voxelMap_from_file("data/cloud1.voxmap")
    #map1 = mapper.create_voxelMap_from_sinfunc(testsize, 1)
    map1 = mapper.voxelise_3Dcloud(mapper.create_test_cloud([10, 10, 10], [3, 5, 8], 500), 1)

    print("fft compression")
    fftmap1 = VoxCompressor.fft_compression(map1)
    print("fft decompression")
    ifftmap1 = VoxCompressor.fft_decompression(fftmap1)

    print("plot")
    map1.plot3D("Original")
    ifftmap1.plot3D("IFFT")

    map1.save_voxelMap_to_file("data/cloud1.voxmap")
    
    VoxCompressor.layerPlotFFT3(map1, fftmap1, ifftmap1)
    plt.show()

    


if __name__ == '__main__':
    main()


