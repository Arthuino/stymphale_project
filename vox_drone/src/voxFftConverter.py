import numpy as np

import matplotlib.pyplot as plt

from mapCreator import VoxelMapCreator
from voxelMap import VoxelMap
from fftMap import FftMap
import pyvista as pv
from complex32 import Complex32
#from main import pyvista_example_points_cloud

class VoxFftConverter:
    def __init__(self):
        pass

    ############################
    ### Conversion functions ###
    ############################

    @staticmethod
    def vox_fft_conversion(voxelMap: VoxelMap, compression_ratio : float = 3.4, plotting : bool = False) -> FftMap:
        """Convert a VoxelMap to its fft representation and apply a low pass filter to compress it
        
        Args:
            voxelMap (VoxelMap): the VoxelMap to convert
            compression_ratio :  filter_size = map_size/compression_ratio. Defaults to 3. min value : 2

        Returns:
            FftMap: the compressed fft of the VoxelMap
        """
        fft3Dmap = np.fft.fftn(voxelMap.get_voxelMap()) # compute the 3D fft of the VoxelMap
        filter_size = tuple(int(dim_size // compression_ratio) for dim_size in fft3Dmap.shape)
        fft3DmapFiltered = VoxFftConverter.fft_lowpass_filter(fft3Dmap, filter_size) # apply a low pass filter on the fft
        fft3DmapCompressed  = VoxFftConverter._compute_filter_compression(fft3DmapFiltered, filter_size) # compress the fft by removing the blank space
        fft3DmapCompressed32 = VoxFftConverter.typeCompression(fft3DmapCompressed)

        # Debugging
        if plotting:
            print("compression_ratio", compression_ratio)
            print("filter_size", filter_size)
            VoxFftConverter.layerPlotFFT(fft3DmapFiltered,"Filtered FFT")
            print("fft3DmapFiltered.shape", fft3DmapFiltered.shape)

        return FftMap(fft3DmapCompressed32, filter_size)
    

    @staticmethod
    def fft_vox_conversion(fft3Dmap: FftMap, plotting : bool = False) -> VoxelMap:
        """Convert a compressed fft to a VoxelMap

        Args:
            fft3Dmap (np.array): the compressed fft of a VoxelMap

        Returns:
            VoxelMap: the VoxelMap reconstructed from the fft
        """
        fft3Dmap = VoxFftConverter.typeDecompression(fft3Dmap)
        fft3DmapF = VoxFftConverter._compute_filter_decompression(fft3Dmap.get_compress_map(),
                                                                  fft3Dmap.get_filter_size()) # decompress the fft

        ifft3Dmap = np.fft.ifftn(fft3DmapF)
        ifftVoxMap = VoxelMap(data = np.abs(ifft3Dmap))

        if plotting:
            VoxFftConverter.layerPlotFFT(fft3DmapF,"Decompressed FFT")

        return ifftVoxMap.binariseMap()
    

    @staticmethod
    def filtrage(spectre, filter_size):
        """Remove high frequency components of a 3D fft

        Args:
            spectre (np.array): the 3D fft
            

        Returns:
            np.array: the filtered 3D fft
        """
        spectre = spectre.copy()

        # Create a zero-filled array with the same shape as spectre
        zeros = np.zeros_like(spectre)
        
        

        # Copy the interior of spectre into the zero-filled array
        zeros[filter_size[0]:-filter_size[0],
                filter_size[1]:-filter_size[1],
                filter_size[2]:-filter_size[2]] = spectre[filter_size[0]:-filter_size[0],
                                                            filter_size[1]:-filter_size[1],
                                                            filter_size[2]:-filter_size[2]]
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
        fft3DmapFiltered = VoxFftConverter.filtrage(fft3DmapFiltered, filter_size) # remove high frequency components
        fft3DmapFiltered = np.fft.ifftshift(fft3DmapFiltered) # shift them back


        return fft3DmapFiltered
    

    #############################
    ### Compression functions ###
    #############################


    @staticmethod
    def _compute_filter_compression(fftMapFiltered: np.array, size_filter: int, plotting : bool = False)->np.array:
        """Compress a filtered fft by removing the blank space"""
        if plotting:
            print("size_filter compression", size_filter)

        # Slice and append along the first axis
        fftMapCompressed = np.append(
            fftMapFiltered[:fftMapFiltered.shape[0]//2-size_filter[0], :, :],
            fftMapFiltered[fftMapFiltered.shape[0]//2+size_filter[0]:, :, :],
            axis=0
        )

        # Slice and append along the second axis
        fftMapCompressed = np.append(
            fftMapCompressed[:, :fftMapCompressed.shape[1]//2-size_filter[1], :],
            fftMapCompressed[:, fftMapCompressed.shape[1]//2+size_filter[1]:, :],
            axis=1
        )

        # Slice and append along the third axis
        fftMapCompressed = np.append(
            fftMapCompressed[:, :, :fftMapCompressed.shape[2]//2-size_filter[2]],
            fftMapCompressed[:, :, fftMapCompressed.shape[2]//2+size_filter[2]:],
            axis=2
        )

        return fftMapCompressed





    @staticmethod
    def _compute_filter_decompression(fft3DmapCompressed:np.array, filter_size:int, plotting : bool = False)->np.array:

        # compute shape of the original fft
        original_shape = ((int)(fft3DmapCompressed.shape[0] + 2*filter_size[0]),
                                (int)(fft3DmapCompressed.shape[1]+ 2*filter_size[1]),
                                (int)(fft3DmapCompressed.shape[2]+ 2*filter_size[2]))

        if plotting:
            print("original_shape", original_shape)
            print("fft3DmapCompressed.shape", fft3DmapCompressed.shape)
            print("size_filter decompress", filter_size)


        fft3Dmap = np.zeros(original_shape, dtype=fft3DmapCompressed.dtype)

        fft3DmapCompressed = np.fft.fftshift(fft3DmapCompressed) # shift the zero frequency component to the center

        
        #fft3Dmap[size_filter:fft3Dmap.shape[0]-size_filter, size_filter:fft3Dmap.shape[1]-size_filter, size_filter:fft3Dmap.shape[2]-size_filter] = fft3DmapCompressed
        # put the compressed fft in the center of the new fft, exept for the third axis where it has to be outside
        fft3Dmap[filter_size[0]:fft3Dmap.shape[0]-filter_size[0], 
                 filter_size[1]:fft3Dmap.shape[1]-filter_size[1],
                   filter_size[2]:fft3Dmap.shape[2]-filter_size[2]] = fft3DmapCompressed

        fft3Dmap = np.fft.ifftshift(fft3Dmap) # shift the zero frequency component to the center

         
        if plotting:
            print("fft3Dmap.shape", fft3Dmap.shape)

        return fft3Dmap




    @staticmethod
    def typeCompression(fftMap64:FftMap):
        """FftMaps are created with complex64 type (two float32).
        Convert to custom complex32 type (two float16)"""
        return np.vectorize(Complex32.complex64_to_complex32)(fftMap64)

    @staticmethod
    def typeDecompression(fftMap32):
        return np.vectorize(Complex32.complex32_to_complex64)(fftMap32)








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
    
    @staticmethod
    def rmse_map(map1 : VoxelMap, map2: VoxelMap) -> float:
        rmse = np.sqrt(np.mean((map1.get_voxelMap() - map2.get_voxelMap()) ** 2))
        return rmse
    


 
    




    
def testConversion():
    mapper = VoxelMapCreator()
    testsize = 50
    voxmap = mapper.create_voxelMap_from_sinfunc(testsize, 1)

    fftmap = VoxFftConverter.vox_fft_conversion(voxmap)

    VoxFftConverter.compression_rate(voxmap, fftmap,True)

    VoxFftConverter.layerPlotFFT(fftmap.get_compress_map())
    
    plt.show()


def testfullConversion(plot, compression_ratio = 3.4):
    """Test the compression and decompression of a VoxelMap
    """
    mapper = VoxelMapCreator()
    voxmap = mapper.read_voxelMap_from_file("data/sinfuncmap2.voxmap")

    #voxmap = VoxelMapCreator().voxelise_3Dcloud(pyvista_example_points_cloud(),1)



    fftmap = VoxFftConverter.vox_fft_conversion(voxmap,compression_ratio)

    ifft_vox_map = VoxFftConverter.fft_vox_conversion(fftmap)

    comp_rate = VoxFftConverter.compression_rate(voxmap, fftmap,False)

    rmse = VoxFftConverter.rmse_map(voxmap, ifft_vox_map)
    



    #plotting
    if plot:
        #VoxFftConverter.layerPlotFFT3(voxmap, fftmap.get_compress_map(), ifft_vox_map)

        pc = VoxelMapCreator().voxel_to_point_cloud(voxmap)
        pv_pc = pv.PolyData(pc)
        pv_pc.plot(eye_dome_lighting=True)
        #voxmap.plot3D()

        #ifft_vox_map.plot3D("IFFT Voxel Map Filtered")
        pc = VoxelMapCreator().voxel_to_point_cloud(ifft_vox_map)
        pv_pc = pv.PolyData(pc)
        pv_pc.plot(eye_dome_lighting=True)
        plt.show()

    return comp_rate,rmse

def compressionRatioEvaluation():
    
    #compression ratios between 2 and 5 incrementing by 0.1
    compression_ratios = np.arange(1.9, 6, 0.1)
    comp_rates = []
    rmses = []
    for compression_ratio in compression_ratios:
        comp_rate,rmse = testfullConversion(False, compression_ratio)
        comp_rates.append(comp_rate)
        rmses.append(rmse*100)
    


    #plotting
    for i, (comp_rate, rmse) in enumerate(zip(comp_rates, rmses)):
        print(f"Compression ratio {compression_ratios[i]} Compression rate {comp_rate:.2f} %      RMSE {rmse:.2f}")
        print()

    plt.plot(compression_ratios, comp_rates)
    plt.plot(compression_ratios, rmses)
    plt.xlabel("Compression ratio")
    plt.ylabel("Compression rate (%) - RMSE (*100)")
    plt.title("Compression rate as a function of the compression ratio")
    plt.show()

if __name__ == "__main__":
    #testConversion() # successful ?

    #print(testfullConversion(True, 5))

    #compressionRatioEvaluation()

    mapper = VoxelMapCreator()
    testsize = 50
    voxmap = mapper.create_voxelMap_from_sinfunc(testsize, 1)

    fftmap = VoxFftConverter.vox_fft_conversion(voxmap)
