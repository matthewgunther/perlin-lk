FROM ubuntu:24.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    build-essential \
    wget \
    g++ \
    make \
    unzip \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libopencv-dev \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libv4l-dev \
    pkg-config \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Set Boost version
ENV BOOST_VERSION=1.81.0

# Download and install Boost
RUN BOOST_VERSION_UNDERSCORE=$(echo ${BOOST_VERSION} | sed 's/\./_/g') && \
    wget https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_UNDERSCORE}.tar.bz2 && \
    tar --bzip2 -xf boost_${BOOST_VERSION_UNDERSCORE}.tar.bz2 && \
    cd boost_${BOOST_VERSION_UNDERSCORE} && \
    ./bootstrap.sh && \
    ./b2 install && \
    cd .. && \
    rm -rf boost_${BOOST_VERSION_UNDERSCORE} boost_${BOOST_VERSION_UNDERSCORE}.tar.bz2

# Set environment variables for Boost
ENV BOOST_ROOT=/usr/local
ENV PATH=$BOOST_ROOT/bin:$PATH

# Install PerlinNoise header file
RUN git clone https://github.com/Reputeless/PerlinNoise.git && \
    mv ./PerlinNoise/PerlinNoise.hpp /usr/include/PerlinNoise.hpp

# Install VS Code
RUN wget "https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64" --output-document=vscode.deb \
    && apt-get update && apt-get install -y ./vscode.deb \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /home/ubuntu
