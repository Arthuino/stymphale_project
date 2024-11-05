#!/bin/bash

xhost +local:*

docker run -it --rm \
            --name zawardo-docker \
            --privileged \
            --net host \
            --ipc host \
            --device=/dev/dri/renderD128 \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /dev/dri/card0:/dev/dri/card0 \
            zawardo_drone:latest
            
xhost -local:*

