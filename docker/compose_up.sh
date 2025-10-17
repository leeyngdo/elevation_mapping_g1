#!/bin/bash

# ===== Default values =====
DISPLAY_ID=":0"
PROJECT_NAME="base"
RUNTIME="runc" # runc or nvidia

# ===== Parse arguments =====
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --display)
      DISPLAY_ID=":$2"
      shift 2
      ;;
    --project)
      PROJECT_NAME="$2"
      shift 2
      ;;
    --ros)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --runtime)
      RUNTIME="$2"
      shift 2
      ;;
    *)
      echo "Unknown parameter: $1"
      exit 1
      ;;
  esac
done

# ===== Setup env vars =====
export DISPLAY=$DISPLAY_ID
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
export PROJECT_NAME=$PROJECT_NAME
export CONTAINER_RUNTIME=$RUNTIME

echo "[*] Using DISPLAY=$DISPLAY"
echo "[*] XAUTHORITY=$XAUTHORITY"

# ===== Allow safe local X11 access only if xhost is available =====
if command -v xhost >/dev/null 2>&1; then
  if xhost +local:docker >/dev/null 2>&1; then
    echo "[*] Enabled local X11 access for Docker"
    XHOST_ENABLED=true
  else
    echo "[!] xhost command failed — skipping X11 access grant"
    XHOST_ENABLED=false
  fi
else
  echo "[!] xhost not found — skipping X11 access grant"
  XHOST_ENABLED=false
fi

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# ===== Run docker-compose =====
if [ "$ROS_DISTRO" == "noetic" ]; then
    docker compose -f docker/ros1/$ROS_DISTRO/$ARCH/docker-compose.yml -p "$PROJECT_NAME" up --remove-orphans
elif [ "$ROS_DISTRO" == "foxy" ]; then
    docker compose -f docker/ros2/$ROS_DISTRO/$ARCH/docker-compose.yml -p "$PROJECT_NAME" up --remove-orphans
elif [ "$ROS_DISTRO" == "humble" ]; then
    docker compose -f docker/ros2/$ROS_DISTRO/$ARCH/docker-compose.yml -p "$PROJECT_NAME" up --remove-orphans
else
    echo "Unknown ROS version: $ROS_DISTRO"
    exit 1
fi

# ===== Cleanup =====
if [ "$XHOST_ENABLED" = true ]; then
  xhost -local:docker
fi