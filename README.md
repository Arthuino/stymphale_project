# stymphale_project

Project of drone swarm.

The purpose of this project is to test several algorithms, strategies and tools in the idea of creating a swarm of drones.

## Vox_drone

The purpose of the vox_drone module is to test the posibilities of using voxels instead of point cloud to represent the environment.
The voxels indeed allow to use Fast Fourrier Transform and apply a low pass filter on the environment.
Thanks to that, we can optains a representation of the environement with a very small memory footprint. That means we could possibly make the exange of the map between the drones faster or with a lower latency/communication frequency.

### Setup

```bash
source .config/setup.sh
```

### Run

```bash
python3 src/main.py
```

### Results

![voxel_map](doc/v010/voxel_map.png "A randomly generated voxel map.")

![fft_before_compression](doc/v010/fft_before_compression.png "The Fast Fourier Transform of the voxel map.")

![fftcompression_layers](doc/v010/fftcompression_layers.png " 1- The voxel map plotted layer by layer. 2- The FFT after compression. 3- The voxel map regenerated from the compressed FFT.")

![decompressedfft](doc/v010/decompressedfft.png "The FFT rebuilt from the compressed FFT.")

![voxel_map_after_transfert](doc/v010/voxel_map_after_transfert.png "The voxel map after the transfer of the compressed FFT.")

The final result depend of the compression. The compression is currently parameted by a compression ratio : ```filter_size = map_size/compression_ratio```

![terminalscreenshotCP3](doc/v010/terminalscreenshotCP3.png "The terminal screenshot for a compression ratio of 3.")

![terminalscreenshotCP4](doc/v010/terminalscreenshotCP4.png "The terminal screenshot for a compression ratio of 4.")
