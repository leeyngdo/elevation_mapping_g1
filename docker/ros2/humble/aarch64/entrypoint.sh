#!/bin/bash

set -e

# Initialize Conda for this shell session
source /opt/conda/etc/profile.d/conda.sh
conda activate elevationmap-env
source /opt/ros/humble/setup.bash

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

# Setup colcon config
mkdir -p /home/ws/.colcon
cat <<EOF > /home/ws/.colcon/defaults.yaml
build:
  build-base: $BUILD_DIR/build
  install-base: $BUILD_DIR/install
  cmake-args:
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=On
    - -DBUILD_TESTING=OFF
    - -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"
EOF
export COLCON_DEFAULTS_FILE=/home/ws/.colcon/defaults.yaml

cd /home/ws && colcon build \
        --continue-on-error \
        --parallel-workers $(nproc) \
        --merge-install \
        --symlink-install \
        --event-handlers console_cohesion+ \
        --base-paths src \
        -Wall -Wextra -Wpedantic -Wshadow \
        --packages-skip \
                convex_plane_decomposition \
                convex_plane_decomposition_ros

# Source and exports for ROS2 stuffs
BASHRC="/root/.bashrc"
MARKER="# After-build ROS2 setup block"

if ! grep -Fxq "$MARKER" "$BASHRC"; then
cat <<EOF >> "$BASHRC"

$MARKER
source $BUILD_DIR/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF
fi

echo -e "\n\n\nElevation mapping cupy container is ready! 🐳🗺️"

# Start an interactive shell inside the env
exec bash