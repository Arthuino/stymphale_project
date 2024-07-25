# This could also be another Ubuntu or Debian based distribution
FROM ubuntu:22.04

# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    x11-apps \
    dbus-x11 \
    libegl1 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libfreetype6 \
    libxi6 \
    libgl1 \
    libgomp1 \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Install Open3D from the PyPI repositories
RUN python3 -m pip install --no-cache-dir --upgrade pip numpy matplotlib open3d


RUN mkdir -p /app

WORKDIR /app
