FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

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

# Install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p /opt/conda && \
    rm miniconda.sh && \
    conda init && \
    conda create -n elevationmap-env python=3.8 -y

# Switch to using Conda env for subsequent RUNs
SHELL ["conda", "run", "-n", "elevationmap-env", "/bin/bash", "-c"]

# Install PyTorch
RUN pip install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html


# Install other Python packages
RUN pip install cupy-cuda11x scikit-learn
RUN pip install 'git+https://github.com/facebookresearch/detectron2.git'

# Install ROS
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
# Install dependencies and ROS Noetic

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
    pyquaternion \
    scipy==1.7 \
    ruamel.yaml \
    opencv-python \
    simple-parsing \
    scikit-image==0.19 \
    matplotlib \
    catkin-tools \
    networkx==3.0 \
    shapely==1.7.1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Inject conda activation into .bashrc
RUN echo 'source /opt/conda/etc/profile.d/conda.sh && conda activate elevationmap-env' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]