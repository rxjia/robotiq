# Robotiq

## Control robotiq_2f_gripper by serial port (robotiq->USB->PC or robotiq->URe serials tool communication)

### Init (one-time setup)

``` bash
# first setup
sudo usermod -a -G dialout $USER
newgrp dialout  # or reboot
sudo udevadm control --reload-rules && sudo udevadm trigger

# check serial port ---------------------
ls /dev|grep ttyUSB
# or check serial port forwarded from UR e-series
ls /tmp|grep tty

# install pymodbus (the version is important) ---------------------
# install pymodbus 2.1.0 via apt
sudo apt install python3-pymodbus
# or install pymodbus 3.6.9 via pip
pip install pymodbus==3.6.9
```

### (Option 1) server listen to topic '/gripper/set_gripper_open', another node publish it

``` bash

# launch action server, action client
roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_as_client.launch port:=/dev/ttyUSB0

# close gripper
rostopic pub /gripper/set_gripper_open std_msgs/Bool "data: false" -1

# open gripper
rostopic pub /gripper/set_gripper_open std_msgs/Bool "data: true" -1
```

### (Option 2) use client code in Python

```bash
# 1. launch action server
roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server robotiq_2f_gripper_as.launch port:=/dev/ttyUSB0
# 2. use the code in robotiq_2f_gripper_action_server/scripts/robotiq_2f_client_node.py
```

## Control robotiq_2f_gripper by socket (robotiq->USB->UR Control Box)

``` bash
# use the class Robotiq2FGripperURCapBridge in:
robotiq_2f_gripper_control/src/robotiq_2f_gripper_control/robotiq_2f_gripper_urcap_bridge.py
```

## Status

As of 2021-05-28, it would appear this repository is ***unmaintained***.

Robotiq is not maintaining the packages in this repository and the last active maintainer ([jproberge](https://github.com/jproberge)) does not appear to be active any more.

The ROS-Industrial consortia are not involved: for historical reasons, the `robotiq` repository is hosted on the `ros-industrial` Github organisation, but there is no direct link with any of the other repositories there.

Please direct support requests to [dof.robotiq.com](https://dof.robotiq.com/). The tracker here is not monitored by Robotiq employees.


## ROS Distro Support

|         | Indigo | Jade | Kinetic | Melodic |
|:-------:|:------:|:----:|:-------:|:-------:|
| Branch  | [`indigo-devel`](https://github.com/ros-industrial/robotiq/tree/indigo-devel) | [`jade-devel`](https://github.com/ros-industrial/robotiq/tree/jade-devel) | [`kinetic-devel`](https://github.com/ros-industrial/robotiq/tree/kinetic-devel) | [`kinetic-devel`](https://github.com/ros-industrial/robotiq/tree/kinetic-devel) |)
| Status  |  supported | not supported |  supported |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=robotiq) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=robotiq) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=robotiq) | [version](http://repositories.ros.org/status_page/ros_melodic_default.html?q=robotiq) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.com/ros-industrial/robotiq.svg?branch=kinetic-devel)](https://travis-ci.com/ros-industrial/robotiq)

## ROS Buildfarm

There are no up-to-date releases of these packages available from the ROS buildfarm.

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Robotiq meta-package.  See the [ROS wiki][] page for more information. 

## License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Contents

This repo holds source code for all versions > groovy. For those versions <= groovy see: [SVN repo][]

[ROS wiki]: http://ros.org/wiki/robotiq
[SVN repo]: https://code.google.com/p/swri-ros-pkg/source/browse

