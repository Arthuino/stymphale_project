import numpy as np
from voxelMap import VoxelMap
from voxCompressor import VoxCompressor
import matplotlib.pyplot as plt
import sys

class FftMap:
    """This class is used to store the fft of a VoxelMap and to compress/decompress it

    TODO: Change the compression value unit to something more meaningful

    TODO: Fix the decompression function

    TODO: Refacto code architecture

    Args:
        fft3Dmap (np.array): the fft of a VoxelMap
        compression_value (int): the compression value used to compress the fft 

    Attributes:
        compression_value (int): the compression value used to compress the fft
        _size_filter (int): the size of the filter used to compress the fft
        _compressed_map (np.array): the compressed fft of the VoxelMap

    Methods:
        _compute_filter_compression: compute the compression of the fft
        _compute_filter_decompression: compute the decompression of the fft
        get_fft_map: return the decompressed fft
        get_compress_map: return the compressed fft
        get_voxel_map: return the decompressed fft
        get_memory_size: return the memory size of the compressed fft

    """
    def __init__(self, fft3Dmap, compression_value):
        self.compression_value = compression_value
        self._size_filter = 0
        self._compressed_map = self._compute_filter_compression(fft3Dmap, compression_value)


    def _compute_filter_compression(self, _fft3Dmap:np.array, compression_value:int):
        fft3Dmap = _fft3Dmap.copy()

        # create a smaller matrix with the low frequencies
        self._size_filter = (int)(fft3Dmap.shape[0]//compression_value)
        

        # Slice and append along the first axis
        fft3DmapC = np.append(
            fft3Dmap[:fft3Dmap.shape[0]//2-self._size_filter, :, :],
            fft3Dmap[fft3Dmap.shape[0]//2+self._size_filter:, :, :],
            axis=0
        )

        # Slice and append along the second axis
        fft3DmapC = np.append(
            fft3DmapC[:, :fft3DmapC.shape[1]//2-self._size_filter, :],
            fft3DmapC[:, fft3DmapC.shape[1]//2+self._size_filter:, :],
            axis=1
        )

        # Slice and append along the third axis
        fft3DmapC = np.append(
            fft3DmapC[:, :, :fft3DmapC.shape[2]//2-self._size_filter],
            fft3DmapC[:, :, fft3DmapC.shape[2]//2+self._size_filter:],
            axis=2
        )
        return fft3DmapC
        
        


    def _compute_filter_decompression(self)->np.array:
        fft3DmapC = self._compressed_map.copy()

        original_shape = ((int)(fft3DmapC.shape[0] + 2*self._size_filter),
                                (int)(fft3DmapC.shape[1]+ 2*self._size_filter),
                                (int)(fft3DmapC.shape[2]+ 2*self._size_filter))
        print("original_shape", original_shape)
        print("fft3DmapC.shape", fft3DmapC.shape)
        print("self._size_filter", self._size_filter)

        fft3Dmap = np.zeros(original_shape, dtype=fft3DmapC.dtype)
        """
        fft3Dmap[0:fft3DmapC.shape[0]//2, 
                 0:fft3DmapC.shape[1]//2, 
                 0:fft3DmapC.shape[2]//2] = fft3DmapC[0:fft3DmapC.shape[0]//2, 
                                                      0:fft3DmapC.shape[1]//2,
                                                      0:fft3DmapC.shape[2]//2,]
        
        fft3Dmap[original_shape[0]-fft3DmapC.shape[0]//2:,
                    original_shape[1]-fft3DmapC.shape[1]//2:,
                    original_shape[2]-fft3DmapC.shape[2]//2:] = fft3DmapC[fft3DmapC.shape[0]//2:,
                                                                        fft3DmapC.shape[1]//2:,
                                                                        fft3DmapC.shape[2]//2:]
        """
        fft3Dmap[0:fft3DmapC.shape[0]//2, :, :] = fft3DmapC[0:fft3DmapC.shape[0]//2, :, :]
        fft3Dmap[original_shape[0]-fft3DmapC.shape[0]//2:, :, :] = fft3DmapC[fft3DmapC.shape[0]//2:, :, :]

        fft3Dmap[:, 0:fft3DmapC.shape[1]//2, :] = fft3DmapC[:, 0:fft3DmapC.shape[1]//2, :]
        fft3Dmap[:, original_shape[1]-fft3DmapC.shape[1]//2:, :] = fft3DmapC[:, fft3DmapC.shape[1]//2:, :]

        fft3Dmap[:, :, 0:fft3DmapC.shape[2]//2] = fft3DmapC[:, :, 0:fft3DmapC.shape[2]//2]
        fft3Dmap[:, :, original_shape[2]-fft3DmapC.shape[2]//2:] = fft3DmapC[:, :, fft3DmapC.shape[2]//2:]
        

         

        print("fft3Dmap.shape", fft3Dmap.shape)

        return fft3Dmap






    def get_fft_map(self):
        return self._compute_filter_decompression()
    
    def get_compress_map(self):
        return self._compressed_map

    def get_voxel_map(self):
        """Return the decompressed fft
        """
        return self._compute_decompression()
    
    def get_memory_size(self):
        """Return the memory size of the compressed fft

        Returns:
            int: the memory size of the compressed fft
        """
        return self._compressed_map.nbytes
    
    

if __name__ == '__main__':
    map1 = VoxelMap(10, 10, 10)
    map1.read_voxelMap_from_file("data/test1.voxmap")
    
    compression_value = 3

    fft3Dmap = VoxCompressor.fft_compression(map1,compression_value)
    fftmap = FftMap(fft3Dmap, compression_value)
    ifftF = fftmap.get_fft_map()

    compression_rate = VoxCompressor.compression_rate(map1, fftmap)

    VoxCompressor.layerPlotFFT(fft3Dmap, "fft3Dmap")
    #VoxCompressor.layerPlotFFT(fftmap.get_compress_map(), "fftmapC")
    VoxCompressor.layerPlotFFT(ifftF, "ifftF")

    #VoxCompressor.fft_decompression(fft3Dmap).plot3D("fft3Dmap")
    #VoxCompressor.fft_decompression(fftmap.get_compress_map()).plot3D("fftmapC")
    #VoxCompressor.fft_decompression(ifftF).plot3D("ifftF")


    plt.show()
    