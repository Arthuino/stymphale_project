# This is the main file for the project
# The purpose of this project is to test the use of voxel for 3D representation of an environment and their use in communication between agents
# One way do to an efficient communication could be the compression of the voxel and the use of AI to do so

# the voxel Map are stored in .voxmap files

# repere : x, y, z
# x : horizontal
# y : vertical
# z : depth


from voxFftConverter import testfullConversionLoop

from pyvista import examples

if __name__ == '__main__':
    #testfullConversionLoop(False)

    mesh = examples.download_dragon()
    mesh['scalars'] = mesh.points[:, 1]
    mesh.plot(cpos='xy', cmap='plasma')

    


