FROM nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04

# Set the working directory
WORKDIR /home

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/conda/bin:$PATH"

# Preconfigure tzdata
RUN echo "Etc/UTC" > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -y tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install git
RUN apt-get update && \
    apt-get install -y git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    build-essential \
    curl \
    wget \
    python3 \
    python3-pip \
    bzip2 && \
    rm -rf /var/lib/apt/lists/*

# Mambaforge (Conda) Installation
ARG MAMBAFORGE_VERSION=23.3.1-0
ARG MAMBAFORGE_ARCH=aarch64
RUN wget -q "https://github.com/conda-forge/miniforge/releases/download/${MAMBAFORGE_VERSION}/Mambaforge-${MAMBAFORGE_VERSION}-Linux-${MAMBAFORGE_ARCH}.sh" -O /tmp/mamba.sh && \
    /bin/bash /tmp/mamba.sh -b -p /opt/conda && \
    rm /tmp/mamba.sh
    
# Add conda/mamba to the PATH
ENV PATH="/opt/conda/bin:$PATH"
RUN mamba init bash && \
    mamba create -n elevationmap-env python=3.8 -y

# Switch to using Conda env for subsequent RUNs
SHELL ["conda", "run", "-n", "elevationmap-env", "/bin/bash", "-c"]

# Install other Python packages
RUN pip install cupy-cuda11x scikit-learn

# Install ROS Noetic
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Install ROS packages
RUN apt install -y ros-noetic-pybind11-catkin \
    ros-noetic-grid-map-core ros-noetic-grid-map-msgs ros-noetic-grid-map-ros ros-noetic-grid-map-rviz-plugin \
    libopencv-dev \
    libeigen3-dev \
    libgmp-dev \
    libmpfr-dev \
    libboost-all-dev \
    ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-teleop \
    ros-noetic-ros-numpy

# Install python packages
RUN pip install --upgrade pip \
    empy==3.3.4 \
    catkin_pkg \
    setuptools==64.0 \
    wheel \
    ninja \
    cmake \
    build \
    numpy \
    pyyaml \
    rospkg \
    onnxruntime \
    ruamel.yaml \
    simple-parsing \
    matplotlib \
    catkin-tools

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Inject conda activation into .bashrc
RUN echo 'source /opt/conda/etc/profile.d/conda.sh && conda activate elevationmap-env' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]