#!/usr/bin/env bash
# Run explicitly on an Ubuntu 20.04 / ROS Noetic machine, or during Docker build.
set -euo pipefail
APT=(apt-get)
if [[ "$EUID" -ne 0 ]]; then APT=(sudo apt-get); fi
"${APT[@]}" update
"${APT[@]}" install -y --no-install-recommends \
  build-essential cmake git python3-catkin-tools python3-vcstool python3-rosdep \
  python3-numpy python3-yaml python3-shapely python3-rospkg python3-opencv \
  libeigen3-dev libpcl-dev libarmadillo-dev libgazebo11-dev \
  ros-noetic-roscpp ros-noetic-rospy ros-noetic-message-generation \
  ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-cv-bridge \
  ros-noetic-gazebo-ros-pkgs ros-noetic-pcl-ros ros-noetic-roslint \
  ros-noetic-tf ros-noetic-tf2-ros
