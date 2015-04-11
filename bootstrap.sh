#!/usr/bin/env bash
# This file will be run after each update,
# so a step must not fail if performed twice!

# TODO: graphical environment, auto-login
#       prompt to set Git user/email

set -e

# Keep system up to date, install prerequisites.
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y wget git expect

# Install ROS.
if [[ -z "${ROS_DISTRO}" ]]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install -y ros-indigo-desktop-full

  sudo rosdep init
  rosdep update
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc
fi

# Set up Gito repo and ROS workspace.
if [[ "${ROS_PACKAGE_PATH}" != "${HOME}/catkin_ws"* ]]; then
  eval "$(ssh-agent -s)"
  ./ssh/add_key.sh
  git clone ssh://git@github.com/fselius/qtcopter ~/catkin_ws
  cd ~/catkin_ws
  catkin_make
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
fi
