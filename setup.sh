#!/bin/bash

# Function to print error message and exit
error_exit()
{
  echo -e "\e[31m[ERROR]\e[0m $1"
  exit 1
}

# Check Ubuntu version
. /etc/os-release
if [[ "$VERSION_ID" != "22.04" ]]; then
  error_exit "Ubuntu 22.04 is required. Detected: $VERSION_ID ($PRETTY_NAME)"
fi

# Check ROS2 version
ROS_DISTRO_EXPECTED="humble"

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
   # Try sourcing from default location
  if [ -f "/opt/ros/$ROS_DISTRO_EXPECTED/setup.bash" ]; then
     source /opt/ros/$ROS_DISTRO_EXPECTED/setup.bash
  fi
fi

if [ "$ROS_DISTRO" != "$ROS_DISTRO_EXPECTED" ]; then
  error_exit "ROS 2 Humble is required. Detected: ${ROS_DISTRO:-none}"
fi

echo -e "\e[32m[SUCCESS]\e[0m Ubuntu 22.04 and ROS2 Humble are correctly installed."

# Build Repo
colcon build

# Source repo
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SETUP_FILE="$SCRIPT_DIR/install/setup.sh"

if [ -f "$SETUP_FILE" ]; then
  source "$SETUP_FILE"
  echo -e "\e[32m[SUCCESS]\e[0m Successfully sourced $SCRIPT_DIR/install/setup.sh. Ready to run ROS2 packages."
else
  echo -e "\e[31m[ERROR]\e[0m Could not find install/setup.sh at: $SETUP_FILE"
  exit 1
fi
