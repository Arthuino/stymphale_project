#!/bin/bash

xhost +local:*

docker run -it --rm \
            --gpus all \
            --privileged \
            --net host \
            --ipc host \
            -e DISPLAY=$DISPLAY \
            -e LIBGL_ALWAYS_SOFTWARE=1 \
            -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /dev/dri/card0:/dev/dri/card0 \
            -v /home/arthuino/stymphale_project/vox_drone:/app/vox_drone \
            vox_drone:latest
            
xhost -local:*

#/bin/bash -c "cd vox_drone && python3 src/main.py"
