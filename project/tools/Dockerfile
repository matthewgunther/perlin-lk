# Use Ubuntu 24.04 base image
FROM ubuntu:24.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install minimal dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libatlas-base-dev \
    python3-dev \
    python3-numpy \
    libtbb-dev \
    libdc1394-dev \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Download and install OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.8.0.zip \
    && unzip opencv.zip \
    && mkdir -p build && cd build \
    && cmake ../opencv-4.8.0 \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_GENERATE_PKGCONFIG=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) \
    && make install \
    && cd / \
    && rm -rf opencv.zip opencv-4.8.0 build

# Set up environment variables
ENV OpenCV_DIR=/usr/local/lib/cmake/opencv4

# Set the working directory
WORKDIR /app

# Command to run when starting the container
CMD ["/bin/bash"]