FROM dustynv/ros:humble-desktop-l4t-r36.2.0

# Set the working directory
WORKDIR /home

# Build arguments
ARG ROS_DISTRO=humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG RMW_NAME=zenoh
ARG INSTALL_EMCUPY_ROSDEPS=true

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive \
    PATH="/opt/conda/bin:$PATH"

# GPG Key
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros2.list

# Clean up and prepare ROS environment
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get purge -y '*opencv*'

## Install system dependencies and colcon tools
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    build-essential \
    curl \
    wget \
    bzip2

RUN apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install curl -y

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade -y && \
    apt install -y ros-humble-desktop &&\
    rm -rf /var/lib/apt/lists/* && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# Install development tools and dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-wheel \
    python3-rosdep \
    python3-vcstool \
    python3-sphinx \
    python3-breathe \
    ros-dev-tools \
    ros-humble-ament-* \
    python3-rosdep \
    libxine2-dev \
    libtiff5-dev \
    libpostproc-dev \
    libopencv-dev \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Install Miniforge3 (includes mamba and pre-configures conda-forge)
ARG MINIFORGE_VERSION="24.3.0-0"
RUN curl -LO "https://github.com/conda-forge/miniforge/releases/download/${MINIFORGE_VERSION}/Miniforge3-${MINIFORGE_VERSION}-Linux-aarch64.sh" && \
    bash Miniforge3-${MINIFORGE_VERSION}-Linux-aarch64.sh -b -p /opt/conda && \
    rm Miniforge3-${MINIFORGE_VERSION}-Linux-aarch64.sh

# Add conda/mamba to the PATH
ENV PATH="/opt/conda/bin:$PATH"
RUN mamba init bash && \
    mamba create -n kraftonros-env python=3.10 -y

# Switch to using Conda env for subsequent RUNs
SHELL ["conda", "run", "-n", "kraftonros-env", "/bin/bash", "-c"]

# Install uv for faster package management
RUN mamba install -c conda-forge uv

# # Install PyTorch
RUN apt-get update && apt-get install -y libopenblas-base
RUN mkdir -p /home/thirdparty && \
    pip install --no-cache "https://developer.download.nvidia.cn/compute/redist/jp/v60dp/pytorch/torch-2.2.0a0+6a974be.nv23.11-cp310-cp310-linux_aarch64.whl"

# Install git
RUN apt-get update && \
    apt-get install -y git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install cyclone DDS
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp

# Install python packages
RUN uv pip install --upgrade pip \
    empy==3.3.4 \
    catkin_pkg \
    setuptools==64.0 \
    wheel \
    ninja \
    cmake \
    build \
    pyyaml \
    rospkg \
    onnxruntime \
    pyquaternion \
    #### For elevation_mapping_cupy ####
    numpy==1.26.4 \
    scikit-learn \
    opencv-python \
    cupy-cuda12x \
    simple-parsing \
    transforms3d \
    scipy

# Install ROS2 packages
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-wheel \
    python3-rosdep \
    python3-vcstool \
    ### For elevation_mapping_cupy ####
    python3-shapely \
    python3-scipy \
    python3-ruamel.yaml \
    # rosdep grid_map_msgs
    ros-humble-grid-map-msgs \
    # rosdep grid_map_ros
    ros-humble-grid-map-ros \
    # rosdep image_transport
    ros-humble-image-transport \
    # rosdep pcl_ros
    ros-humble-pcl-ros \
    # rosdep cv_bridge
    ros-humble-cv-bridge \
    # rosdep tf-transformations
    ros-humble-tf-transformations \
    # rosdep rviz2
    ros-humble-rviz2 \
    # replacing with rosdep gazebo_ros_pkgs 
    ros-humble-gazebo-ros \
    # rosdep grid_map_cv
    ros-humble-grid-map-cv \
    # rosdep grid_map_core
    ros-humble-grid-map-core \
    # rosdep grid_map_demos
    ros-humble-grid-map-demos \
    ros-humble-point-cloud-transport \
    #################################
    ### For debugging elevation_mapping_cupy by setting up turtlebot3_simulations####
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-gazebo-plugins \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-teleop \
    # replacing with rosdep gazebo_ros_pkgs
    ros-humble-gazebo-ros-pkgs \
    #############################################
    && apt clean && \
    rm -rf /var/lib/apt/lists/* ;\

# Setup rosdep
RUN rosdep init
# COPY docker/extra_rosdeps.yaml /etc/ros/rosdep/sources.list.d/emcupy-rosdeps-fixed.yaml
# Setup rosdep with extra rosdeps and install them for elevation_mapping_cupy
# RUN --mount=type=cache,target=/home/${USERNAME}/.ros/rosdep/sources.cache \
# RUN if [ "$INSTALL_EMCUPY_ROSDEPS" = true ]; then \
#         echo "yaml file:///etc/ros/rosdep/sources.list.d/emcupy-rosdeps-fixed.yaml" | tee /etc/ros/rosdep/sources.list.d/01-emcupy-rosdeps.list && \
#         rosdep update && \
#         # This install doesn't seem to actually install anything, so removing
#         apt update && \
#         rosdep install --from-paths . --ignore-src -y -r; \
#     fi
# Now add the same file again, but this one is symlinked so will change if you change the file in your workspace
RUN ln -s ${WORKSPACE}/src/elevation_mapping_cupy/docker/extra_rosdeps.yaml /etc/ros/rosdep/sources.list.d/emcupy-rosdeps.yaml && \
        echo "yaml file:///etc/ros/rosdep/sources.list.d/emcupy-rosdeps.yaml" | tee /etc/ros/rosdep/sources.list.d/00-emcupy-rosdeps.list;


# Additional python packages for ROS2
RUN uv pip install \
    lark

# For elevation_mapping_cupy
ENV TURTLEBOT3_MODEL=waffle_realsense_depth

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Inject conda activation into .bashrc
RUN echo 'source /opt/conda/etc/profile.d/conda.sh && conda activate kraftonros-env' >> /root/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]