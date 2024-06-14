import numpy as np

import matplotlib.pyplot as plt

from mapCreator import VoxelMapCreator
from voxelMap import VoxelMap


class VoxCompressor:
    def __init__(self):
        pass

    @staticmethod
    def fft_compression(voxelMap: VoxelMap, compression_value: int = 3 ) -> np.array:
        fft3Dmap = np.fft.fftn(voxelMap.get_voxelMap())
        fft3DmapF = VoxCompressor.fft_lowpass_filter(fft3Dmap, compression_value)
        return fft3DmapF
    
    @staticmethod
    def fft_decompression(fft3Dmap: np.array) -> VoxelMap:
        ifft3Dmap = np.fft.ifftn(fft3Dmap)
        ifftVoxMap = VoxelMap(data = np.abs(ifft3Dmap))
        return ifftVoxMap.binariseMap()
    
    @staticmethod
    def layerPlotFFT(fft3Dmap: np.array, title = ""):
        fig, ax = plt.subplots(1, fft3Dmap.shape[2])
        fig.suptitle(title)
        for i in range(fft3Dmap.shape[2]):
            epsilon = np.finfo(float).eps # to avoid log(0)
            ax[i].imshow(np.log(np.abs(np.fft.fftshift(fft3Dmap[:, :, i]))**2 + epsilon))
            ax[i].set_title("Layer " + str(i))

    @staticmethod
    def layerPlotFFT3(voxmap : VoxelMap, fft3Dmap:np.array, ifftVoxMap:VoxelMap, title = ""):

        fig, ax = plt.subplots(3, voxmap.depth)
        fig.suptitle(title)
        for i in range(voxmap.depth):
            ax[0][i].imshow(voxmap.get_voxelMap()[:, :, i])
            ax[0][i].set_title("Layer " + str(i))
        
        for i in range(fft3Dmap.shape[2]):
            epsilon = np.finfo(float).eps # to avoid log(0)
            ax[1][i].imshow(np.log(np.abs(np.fft.fftshift(fft3Dmap[:, :, i]))**2 + epsilon))
            ax[1][i].set_title("Layer " + str(i))

        for i in range(ifftVoxMap.depth):
            ax[2][i].imshow(np.abs(ifftVoxMap.get_voxelMap()[:, :, i]))
            ax[2][i].set_title("Layer " + str(i))

        return
    
    @staticmethod
    def compression_rate(voxelMap: VoxelMap, fftMap) -> float:
        print("voxelMap memory size",voxelMap.get_memory_size())
        print("fftMap memory size",fftMap.get_memory_size())
        compression_rate = 100*(1-fftMap.get_memory_size()/voxelMap.get_memory_size())
        print("compression rate",compression_rate)
        return compression_rate
    






    @staticmethod
    def filtrage(spectre, n):
        spectre = spectre.copy()

        # Create a zero-filled array with the same shape as spectre
        zeros = np.zeros_like(spectre)
        
        # TODO: marche très bien pour un cube, mais dois être généralisé 

        # Copy the interior of spectre into the zero-filled array
        zeros[n:-n, n:-n, n:-n] = spectre[n:-n, n:-n, n:-n]

        # Replace spectre with the zero-filled array
        spectre = zeros

        
        return spectre




    @staticmethod
    def fft_lowpass_filter(fft3Dmap: np.array, compression_value:int = 4) -> np.array:
        # remove high frequency components by setting exterior values to 0
        fft3DmapFiltered = np.copy(fft3Dmap)

        fft3DmapFiltered = np.fft.fftshift(fft3DmapFiltered) # shift the zero frequency component to the center
        fft3DmapFiltered = VoxCompressor.filtrage(fft3DmapFiltered, fft3DmapFiltered.shape[0]//compression_value) # remove high frequency components
        fft3DmapFiltered = np.fft.ifftshift(fft3DmapFiltered) # shift them back


        return fft3DmapFiltered
    



def testCompressor():
    mapper = VoxelMapCreator()
    testsize = 50
    voxmap = mapper.create_voxelMap_from_sinfunc(testsize, 1)

    fftmap = VoxCompressor.fft_compression(voxmap)

    ifftmap = VoxCompressor.fft_decompression(fftmap)

    VoxCompressor.layerPlotFFT3(voxmap, fftmap, ifftmap)

    voxmap.plot3D()
    ifftmap.plot3D("IFFT Voxel Map Filtered")

    plt.show()


if __name__ == "__main__":
    testCompressor()