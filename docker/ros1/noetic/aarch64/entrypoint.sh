#!/bin/bash
# Entrypoint to the elevation mapping container

set -e

# Initialize Conda for this shell session
source /opt/conda/etc/profile.d/conda.sh
conda activate elevationmap-env
source /opt/ros/noetic/setup.bash

# Trust the mounted repo path to avoid Git "dubious ownership" error
git config --global --add safe.directory /home/ws/src/elevation_mapping


# Append Git prompt and PS1 config to .bashrc if not already present
BASHRC="/root/.bashrc"  # Change to /home/youruser/.bashrc if needed

if ! grep -q "git-prompt.sh" "$BASHRC"; then
cat << 'EOF' >> "$BASHRC"

# Custom Git-aware prompt
if [ -f "/usr/share/git/git-prompt.sh" ]; then
    source "/usr/share/git/git-prompt.sh"
elif [ -f "/etc/bash_completion.d/git-prompt" ]; then
    source "/etc/bash_completion.d/git-prompt"
fi

# ANSI colors
red="\[\033[0;31m\]"
green="\[\033[0;32m\]"
yellow="\[\033[0;33m\]"
blue="\[\033[0;34m\]"
reset="\[\033[0m\]"
bright_black="\[\033[1;30m\]"
bright_red="\[\033[1;31m\]"
bright_green="\[\033[1;32m\]"
bright_yellow="\[\033[1;33m\]"
bright_blue="\[\033[1;34m\]"
bright_magenta="\[\033[1;35m\]"
bright_cyan="\[\033[1;36m\]"
bright_white="\[\033[1;37m\]"

# Prompt format with conda and git branch
PS1="🐳🗺️ (${bright_magenta}$PROJECT_NAME-$ROS_DISTRO)\n"
PS1+="${bright_green}\u${reset}"                   # user
PS1+=":${bright_blue}\w${reset}"                   # path
PS1+='$( __git_ps1 " ['"${bright_cyan}"'%s'"${reset}"']" )'  # branch
PS1+=" \$ "

export PS1
EOF
fi

export BUILD_DIR=/home/ws/builds/$ROS_DISTRO
echo -e "Building for ${ROS_DISTRO}"

catkin config \
  --workspace /home/ws \
  --build-space $BUILD_DIR/build \
  --devel-space $BUILD_DIR/devel \
  --log-space $BUILD_DIR/log \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5

cd /home/ws/

echo "--- [Step 1/3] Building livox_ros_driver2 package first ---"
cd /home/ws/src/elevation_mapping_g1/livox_ros_driver2/Livox-SDK2/
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && make -j
sudo make install
cd /home/ws/src/elevation_mapping_g1/livox_ros_driver2/
cp -f package_ROS1.xml package.xml
cd /home/ws/
# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf build/
rm -rf devel/
rm -rf install/
# clear src/CMakeLists.txt if it exists.
if [ -f src/CMakeLists.txt ]; then
    rm -f src/CMakeLists.txt
fi
catkin build livox_ros_driver2 -DROS_EDITION="ROS1" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

echo "--- [Step 2/3] Sourcing workspace after livox build ---"
source $BUILD_DIR/devel/setup.bash

echo "--- [Step 3/3] Building all remaining packages ---"
cd /home/ws/src/elevation_mapping_g1/orocos_kinematics_dynamics
git submodule update --init # initialize PyBind11
cd /home/ws
catkin build

echo "source $BUILD_DIR/devel/setup.bash" >> /root/.bashrc

echo -e "\n\n\nElevation mapping cupy container is ready! 🐳🗺️"

# Start an interactive shell inside the env
exec bash