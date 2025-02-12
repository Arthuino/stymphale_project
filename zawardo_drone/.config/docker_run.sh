#!/bin/bash
PROJECT_DIR=/home/arthuino/stymphale_project/zawardo_drone
xhost +local:*

docker run -it --rm \
            --name zawardo-docker \
            --privileged \
            --net host \
            --ipc host \
            --device=/dev/dri/renderD128 \
            --gpus=all \
            -e DISPLAY="$DISPLAY" \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /dev/dri/card0:/dev/dri/card0 \
            -v $PROJECT_DIR/data:/home/ardupilotuser/ros2_ws/data:rw \
            arthuino/stymphale-zawardo:latest
            
xhost -local:*

