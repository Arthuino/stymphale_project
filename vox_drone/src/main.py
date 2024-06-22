# This is the main file for the project
# The purpose of this project is to test the use of voxel for 3D representation of an environment and their use in communication between agents
# One way do to an efficient communication could be the compression of the voxel and the use of AI to do so

# the voxel Map are stored in .voxmap files

# repere : x, y, z
# x : horizontal
# y : vertical
# z : depth


import matplotlib
#matplotlib.use('TkAgg') # set the backend of matplotlib to TkAgg


from voxFftConverter import testfullConversionLoop
import open3d as o3d


if __name__ == '__main__':
    testfullConversionLoop(True)

    #dataset = o3d.data.EaglePointCloud()
    #pcd = o3d.io.read_point_cloud(dataset.path)
    #o3d.visualization.draw(pcd)

    


