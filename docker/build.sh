#!/bin/bash
echo "Building the elevation_mapping docker image..."
# get directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# receive argument --ros (noetic or foxy or humble)
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --ros)
      ROS_DISTRO="$2"
      shift 2
      ;;
    *)
      echo "Unknown parameter: $1"
      exit 1
      ;;
  esac
done

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# Choose build directory based on ROS version and architecture
if [ "$ROS_DISTRO" == "noetic" ]; then
    cd $DIR/ros1/$ROS_DISTRO/$ARCH
else
    echo "Unknown ROS version: $ROS_DISTRO"
    exit 1
fi

# Build docker image
docker build -t elevationmap:$ROS_DISTRO .