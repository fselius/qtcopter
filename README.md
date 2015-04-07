# qcopter

## Directory Structure

    build/           ROS build directory (not committed to Git repository)
    devel/           ROS devel directory (not committed to Git repository)
    src/             ROS sources
        qcopter      Mission nodes & launchfiles for real missions; implementation
        qcopter_sim  Mission nodes & launchfiles for simulation

## Developer Quickstart

 1. [Install ROS][install-ros].
 2. Set up this repository as ROS workspace:

```
$ git clone --recursive https://github.com/fselius/qcopter.git ~/catkin_ws
$ cd ~/catkin_ws
$ catkin_make
```

 3. Set up your environment for this new workspace (Bash example):

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

 4. Check that the environment is set up correctly by trying to run an executable:

```
$ rosrun qcopter calibrate_camera.py
```

 5. To run the tests in `src/qcopter/test`:

```
$ catkin_make run_tests
```


[install-ros]: http://wiki.ros.org/indigo/Installation/Ubuntu
