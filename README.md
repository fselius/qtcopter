# qtcopter

## Files

    build/            ROS build directory (not committed to Git repository)
    devel/            ROS devel directory (not committed to Git repository)
    src/              ROS sources
        qtcopter      Mission nodes & launchfiles for real missions; implementation
        qtcopter_sim  Mission nodes & launchfiles for simulation
    ssh/              SSH keys to access the private Github repo
    bootstrap.sh      Install/update required software on an Ubuntu 14.04 system
    update.sh         Update the repository and required software
    Vagrantfile       Initial creation of the Virtualbox image

## Developer Quickstart

If you are already running Ubuntu 14.04 (Trusty), you can execute `bootstrap.sh` to set everything up for you.

To get a virtual machine with ROS etc, install [Vagrant][vagrant] and run `vagrant up` in this directory.

To run the tests in `src/qtcopter/test`:

```
$ catkin_make run_tests
```

[vagrant]: https://www.vagrantup.com/
