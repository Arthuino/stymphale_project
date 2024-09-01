#!/bin/bash

# Set the Docker image name and tag
IMAGE_NAME="zawardo_drone"
IMAGE_TAG="latest"
CONFIG_DIR="./.config"

# Build the Docker image
docker build -t "$IMAGE_NAME:$IMAGE_TAG"  -f $CONFIG_DIR/zawardo_docker $CONFIG_DIR