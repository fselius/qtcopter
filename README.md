# qtcopter

## Files

    build/              ROS build directory (not committed to Git repository)
    devel/              ROS devel directory (not committed to Git repository)
    src/                ROS sources
        qtcopter        Mission nodes & launchfiles for real missions; implementation
        qtcopter_sim    Mission nodes & launchfiles for simulation
    ssh/                SSH keys to access the private Github repo
    bootstrap.sh        Install/update required software on an Ubuntu 14.04 system
    update_qtcopter.sh  Update script for non-Vagrant VMs - do not execute!
    export_vm.sh        Export a VM for non-developers
    Vagrantfile         Configuration for Vagrant

## For Developers

If you are already running Ubuntu 14.04 (Trusty), you can execute `bootstrap.sh` to set everything up for you.

To use an Ubuntu on a virtual machine, install [Vagrant][vagrant] and run `vagrant up` in this directory. The first time, this will install and set up ROS. To stop the machine, run `vagrant halt`.

To run the tests in `src/qtcopter/test` (within the VM):

```
$ cd ~/catkin_ws
$ catkin_make run_tests
```

To export the VM for non-developers:

```
$ ./export_vm.sh
```

To run tests with a static image (in separate terminals):

```
$ roscore
$ roslaunch qtcopter_sim sim.launch camera:=false image_path:=path/to/image
$ roslaunch qtcopter <mission>.launch
```

To run tests with a connected camera (in separate terminals):

```
$ roscore
$ roslaunch qtcopter_sim sim.launch
$ roslaunch qtcopter <mission>.launch
```

Or use `roslaunch qtcopter_sim <mission>.launch`.

### State Machine

![State machine for the balldrop mission](images/balldrop_statemachine.png)

Write the state machine to a Graphviz file:

```python
with open('sm.dot', 'w') as file:
    sm.write_dot(file)
```

Convert it to an image:

```
$ dot -Tpng sm.dot -o sm.png
```

## For Deploying

Required ROS packages:

    ros-indigo-camera-info-manager-py
    ros-indigo-pointgrey-camera-driver

The last package has to be built from source:

    $ echo "deb http://packages.ros.org/ros/ubuntu trusty main" >> /etc/apt/sources.list.d/ros-latest.list
    $ sudo apt-get install apt-build
    $ sudo apt-build install ros-indigo-pointgrey-camera-driver

## For Users

Obtain a VM image from the developers and start it in [VirtualBox][virtualbox]. Username and password are both `vagrant`. Each time you want to update the Qtcopter code, run `update_qtcopter.sh` from the Desktop.

[vagrant]: https://www.vagrantup.com/
[virtualbox]: https://www.virtualbox.org/
