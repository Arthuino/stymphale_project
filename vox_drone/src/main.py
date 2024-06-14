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
from voxFftConverter import VoxFftConverter
from voxFftConverter import testfullConversionLoop


def main():
    testfullConversionLoop(True)

    

if __name__ == '__main__':
    main()


