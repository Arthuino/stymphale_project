import numpy as np

import matplotlib.pyplot as plt

from mapCreator import VoxelMapCreator
from voxelMap import VoxelMap
from fftMap import FftMap


class voxFftConverter:
    def __init__(self):
        pass

    ############################
    ### Conversion functions ###
    ############################

    @staticmethod
    def vox_fft_conversion(voxelMap: VoxelMap, compression_ratio : float = 3.0) -> FftMap:
        """Convert a VoxelMap to its fft representation and apply a low pass filter to compress it
        
        Args:
            voxelMap (VoxelMap): the VoxelMap to convert
            compression_ratio :  filter_size = map_size/compression_ratio. Defaults to 3. max value : 2

        Returns:
            FftMap: the compressed fft of the VoxelMap
        """

        fft3Dmap = np.fft.fftn(voxelMap.get_voxelMap()) # compute the 3D fft of the VoxelMap

        filter_size = (int)(fft3Dmap.shape[0]//compression_ratio)

        fft3DmapFiltered = voxFftConverter.fft_lowpass_filter(fft3Dmap, filter_size) # apply a low pass filter on the fft

        fft3DmapCompressed  = voxFftConverter._compute_filter_compression(fft3DmapFiltered, filter_size) # compress the fft by removing the blank space

        # Debugging
        voxFftConverter.layerPlotFFT(fft3DmapFiltered,"Filtered FFT")
        print("fft3DmapFiltered.shape", fft3DmapFiltered.shape)

        return FftMap(fft3DmapCompressed, filter_size)
    

    @staticmethod
    def fft_vox_conversion(fft3Dmap: FftMap) -> VoxelMap:
        """Convert a compressed fft to a VoxelMap

        Args:
            fft3Dmap (np.array): the compressed fft of a VoxelMap

        Returns:
            VoxelMap: the VoxelMap reconstructed from the fft
        """

        fft3DmapF = voxFftConverter._compute_filter_decompression(fft3Dmap.get_compress_map(),
                                                                  fft3Dmap.get_filter_size()) # decompress the fft

        voxFftConverter.layerPlotFFT(fft3DmapF,"Decompressed FFT")

        ifft3Dmap = np.fft.ifftn(fft3DmapF)
        ifftVoxMap = VoxelMap(data = np.abs(ifft3Dmap))
        return ifftVoxMap.binariseMap()
    

    @staticmethod
    def filtrage(spectre, n):
        """Remove high frequency components of a 3D fft

        Args:
            spectre (np.array): the 3D fft
            n (int): the compression value

        Returns:
            np.array: the filtered 3D fft
        """
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
    def fft_lowpass_filter(fft3Dmap: np.array, filter_size:int) -> np.array :
        """Apply a low pass filter to a 3D fft

        Args:
            fft3Dmap (np.array): the 3D fft
            filter_size (int): the size of the filter


        Returns:
            np.array: the filtered 3D fft
        """
        # remove high frequency components by setting exterior values to 0
        fft3DmapFiltered = np.copy(fft3Dmap)

        fft3DmapFiltered = np.fft.fftshift(fft3DmapFiltered) # shift the zero frequency component to the center
        fft3DmapFiltered = voxFftConverter.filtrage(fft3DmapFiltered, filter_size) # remove high frequency components
        fft3DmapFiltered = np.fft.ifftshift(fft3DmapFiltered) # shift them back


        return fft3DmapFiltered
    

    #############################
    ### Compression functions ###
    #############################


    @staticmethod
    def _compute_filter_compression(fftMapFiltered: np.array, size_filter: int)->np.array:

        print("size_filter compression", size_filter)

        # Slice and append along the first axis
        fftMapCompressed = np.append(
            fftMapFiltered[:fftMapFiltered.shape[0]//2-size_filter, :, :],
            fftMapFiltered[fftMapFiltered.shape[0]//2+size_filter:, :, :],
            axis=0
        )

        # Slice and append along the second axis
        fftMapCompressed = np.append(
            fftMapCompressed[:, :fftMapCompressed.shape[1]//2-size_filter, :],
            fftMapCompressed[:, fftMapCompressed.shape[1]//2+size_filter:, :],
            axis=1
        )

        # Slice and append along the third axis
        fftMapCompressed = np.append(
            fftMapCompressed[:, :, :fftMapCompressed.shape[2]//2-size_filter],
            fftMapCompressed[:, :, fftMapCompressed.shape[2]//2+size_filter:],
            axis=2
        )

        return fftMapCompressed





    @staticmethod
    def _compute_filter_decompression(fft3DmapCompressed:np.array, filter_size:int)->np.array:

        # compute shape of the original fft
        original_shape = ((int)(fft3DmapCompressed.shape[0] + 2*filter_size),
                                (int)(fft3DmapCompressed.shape[1]+ 2*filter_size),
                                (int)(fft3DmapCompressed.shape[2]+ 2*filter_size))
        print("original_shape", original_shape)
        print("fft3DmapCompressed.shape", fft3DmapCompressed.shape)
        print("size_filter decompress", filter_size)


        fft3Dmap = np.zeros(original_shape, dtype=fft3DmapCompressed.dtype)

        fft3DmapCompressed = np.fft.fftshift(fft3DmapCompressed) # shift the zero frequency component to the center

        
        #fft3Dmap[size_filter:fft3Dmap.shape[0]-size_filter, size_filter:fft3Dmap.shape[1]-size_filter, size_filter:fft3Dmap.shape[2]-size_filter] = fft3DmapCompressed
        # put the compressed fft in the center of the new fft, exept for the third axis where it has to be outside
        fft3Dmap[filter_size:fft3Dmap.shape[0]-filter_size, filter_size:fft3Dmap.shape[1]-filter_size, filter_size:fft3Dmap.shape[2]-filter_size] = fft3DmapCompressed

        fft3Dmap = np.fft.ifftshift(fft3Dmap) # shift the zero frequency component to the center

         

        print("fft3Dmap.shape", fft3Dmap.shape)

        return fft3Dmap





    @staticmethod
    def compression_rate(voxelMap: VoxelMap, fftMap:FftMap , printing:bool=False) -> float:
        """Compute the compression rate of a VoxelMap and its fft

        Args:
            voxelMap (VoxelMap): the VoxelMap
            fftMap ([type]): the fft of the VoxelMap
        
        Returns:
            float: the compression rate
        """

        compression_rate = 100*(1-fftMap.get_memory_size()/voxelMap.get_memory_size())

        if printing:
            print("voxelMap memory size ",voxelMap.get_memory_size())
            print("fftMap memory size ",fftMap.get_memory_size())
            print("compression rate ",compression_rate)
        return compression_rate




    ##########################
    ### Plotting functions ###
    ##########################
    
    @staticmethod
    def layerPlotFFT(fft3Dmap: np.array, title = ""):
        """Plot the fft of a VoxelMap layer by layer

        Args:
            fft3Dmap (np.array): the fft of a VoxelMap
            title (str, optional): the title of the plot. Defaults to "".
        """
        fig, ax = plt.subplots(1, fft3Dmap.shape[2])
        fig.suptitle(title)
        for i in range(fft3Dmap.shape[2]):
            epsilon = np.finfo(float).eps # to avoid log(0)
            ax[i].imshow(np.log(np.abs(np.fft.fftshift(fft3Dmap[:, :, i]))**2 + epsilon))
            ax[i].set_title("Layer " + str(i))



    @staticmethod
    def layerPlotFFT3(voxmap : VoxelMap, fft3Dmap:np.array, ifftVoxMap:VoxelMap, title = ""):
        """Plot the VoxelMap, its fft and the reconstructed VoxelMap layer by layer

        Args:
            voxmap (VoxelMap): the VoxelMap
            fft3Dmap (np.array): the fft of the VoxelMap
            ifftVoxMap (VoxelMap): the reconstructed VoxelMap
            title (str, optional): the title of the plot. Defaults to "".
        """

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
    


 
    




    
def testConversion():
    mapper = VoxelMapCreator()
    testsize = 50
    voxmap = mapper.create_voxelMap_from_sinfunc(testsize, 1)

    fftmap = voxFftConverter.vox_fft_conversion(voxmap)

    voxFftConverter.compression_rate(voxmap, fftmap,True)

    voxFftConverter.layerPlotFFT(fftmap.get_compress_map())
    
    plt.show()


def testfullConversionLoop(plot):
    """Test the compression and decompression of a VoxelMap
    """
    mapper = VoxelMapCreator()
    #voxmap = mapper.create_voxelMap_from_sinfunc(50, 1)
    voxmap = mapper.read_voxelMap_from_file("data/test1.voxmap")


    fftmap = voxFftConverter.vox_fft_conversion(voxmap,3)

    ifft_vox_map = voxFftConverter.fft_vox_conversion(fftmap)

    voxFftConverter.compression_rate(voxmap, fftmap,True)


    #plotting
    if plot:
        voxFftConverter.layerPlotFFT3(voxmap, fftmap.get_compress_map(), ifft_vox_map)
        voxmap.plot3D()
        ifft_vox_map.plot3D("IFFT Voxel Map Filtered")
        plt.show()






if __name__ == "__main__":
    #testConversion() # successful ?

    testfullConversionLoop(True)