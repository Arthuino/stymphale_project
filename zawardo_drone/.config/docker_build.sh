#!/bin/bash

# Set the Docker image name and tag
IMAGE_NAME="arthuino/stymphale-zawardo"
IMAGE_TAG="latest"
CONFIG_DIR="./.config"
TARGET="$1"

# Build the Docker image
if [ $# -eq 1 ]; then
    echo "Build docker to target $TARGET"
    docker build -t "$IMAGE_NAME:$IMAGE_TAG"  -f $CONFIG_DIR/zawardo_docker $CONFIG_DIR --target $TARGET --progress=plain
else
    echo "Default docker build"
    docker build -t "$IMAGE_NAME:$IMAGE_TAG"  -f $CONFIG_DIR/zawardo_docker $CONFIG_DIR
fi
