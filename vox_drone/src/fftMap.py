import numpy as np
from voxelMap import VoxelMap
import matplotlib.pyplot as plt
import sys

class FftMap:
    """This class is used to store the fft of a VoxelMap and to compress/decompress it

    Args:
        fft3Dmap (np.array): the fft of a VoxelMap
        filter_size (int): the size of the filter used to compress the fft

    Attributes:
        _size_filter (int): the size of the filter used to compress the fft
        _compressed_map (np.array): the compressed fft of the VoxelMap

    Methods:
        

    """
    def __init__(self, compressedFft3Dmap, filter_size):
        """Constructor of the FftMap class

        Args:
            compressedFft3Dmap (np.array): the compressed fft of the VoxelMap
            filter_size (int): the size of the filter used to compress the fft

        """

        self._filter_size = filter_size
        self._compressed_map = compressedFft3Dmap
        

    
    def get_compress_map(self)->np.array:
        return self._compressed_map
    
    def get_memory_size(self):
        """Return the memory size of the compressed fft

        Returns:
            int: the memory size of the compressed fft
        """
        return self._compressed_map.nbytes
    
    def get_filter_size(self):
        return self._filter_size


if __name__ == '__main__':
    pass




























"""
The Laws of Robotics:
1. A robot may not injure a human being or, through inaction, allow a human being to come to harm.
2. A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
3. A robot must protect its own existence as long as such protection does not conflict with the First or Second Laws.

- Isaac Asimov
"""