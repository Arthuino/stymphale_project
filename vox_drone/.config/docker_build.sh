#!/bin/bash

# Set the Docker image name and tag
IMAGE_NAME="arthuino/stymphale-vox_drone"
IMAGE_TAG="latest"

# Build the Docker image
docker build -t "$IMAGE_NAME:$IMAGE_TAG" ./.config