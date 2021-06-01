# cares_lib_ros
This package contains general utility functions, classes, and other general purpose methods for projects throughout CARES. 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
1) ROS Noetic

2) ROS Numpy
	a) sudo apt-get install ros-noetic-ros-numpy

3) Open3d
	a) sudo pip3 install open3d

4) cares_msgs
	a) https://github.com/UoA-CARES/cares_msgs
```

NOTE: This package should not have any other major external dependencies added, this package is a generic library useful for all CARES packages and should not add dependency creep! If a function/class/feature you want to add requires a non-typical third party library then it is likely it should be developed as seperate wrapper package for that library instead.

### Installing

A step by step series of examples that tell you how to get a development env running

Clone the package into the catkin directory you are using, presumned here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/UoA-CARES/cares_lib_ros.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

## Running the tests

Tests to be added