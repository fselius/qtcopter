# qcopter

    src                 ROS sources

## Quickstart

 1. [Install ROS][install-ros].
 2. Set up this repository as ROS workspace, for example:

```
$ git clone --recursive https://github.com/fselius/qcopter.git ~/catkin_ws
$ cd ~/catkin_ws
$ catkin_make
```

 3. Set up your environment for this new workspace, for example:

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

 4. Check that the environment is set up correctly by trying to run an executable:

```
$ rosrun qcopter calibrate_camera.py
```

[install-ros]: http://wiki.ros.org/indigo/Installation/Ubuntu
