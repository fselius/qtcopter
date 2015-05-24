#!/bin/bash -e

# This will run when the Vagrant VM is initially provisioned (i.e. set up).
# Can also be used to set up a non-VM Ubuntu Trusty for ROS.

cd "${HOME}/catkin_ws"

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
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bash_profile
  source ~/.bash_profile
else
  # List additionally required ROS packages
  sudo apt-get install -y ros-indigo-desktop-full ros-indigo-hector-quadrotor-description ros-indigo-usb-cam
fi

# Set up ROS workspace.
if [[ "${ROS_PACKAGE_PATH}" != "${HOME}/catkin_ws/src:"* ]]; then
  rm -rf build devel
  catkin_make
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bash_profile
  source ~/.bash_profile
fi

if ! grep -q "~/.bash_profile" ~/.bashrc; then
  cat >> ~/.bashrc <<EOS
if [ -f ~/.bash_profile ]; then
    . ~/.bash_profile
fi
EOS
fi

# If not executed from GUI, install one.
# Make sure to not install anything on a non-Vagrant Ubuntu.
if [[ -z "${DISPLAY}" && ! -d /etc/lightdm && "$(whoami)" = vagrant ]]; then
  sudo apt-get install -y xfce4 virtualbox-guest-x11 lightdm xubuntu-icon-theme xubuntu-default-settings
  sudo mkdir /etc/lightdm/lightdm.conf.d
  cat > /tmp/50-qtcopter.conf <<EOS
[SeatDefaults]
user-session=xfce
autologin-user=vagrant
autologin-user-timeout=delay
EOS
  sudo mv /tmp/50-qtcopter.conf /etc/lightdm/lightdm.conf.d/50-qtcopter.conf
  echo "Restart the VM to get a GUI."
fi

# Clean package cache.
sudo apt-get clean
