# inertialsense_imx5
ROS2 Driver for Inertial Sense IMX5 IMUs. Functionality for GPS has not been added yet. Tested on RUG3-IMX5 IMU with ROS2 Humble. Code is heavily based on [Inertial Sense ROS1 Driver](https://github.com/inertialsense/inertial-sense-ros) with a little help from [MagiicLab ROS2 Driver](https://gitlab.magiccvs.byu.edu/boatlanding/ros2-sensor-drivers/inertialsense_ros2/-/tree/ros2)

# Inertial Sense ROS

A ROS wrapper for the Inertial Sense IMX product line.

## Installation

``` bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git https://github.com/uah-crablab/inertialsense_imx5.git
cd ~/ros_ws/src/inertialsense_imx5/inertialsense_imx5/lib/inertial-sense-sdk/src/libusb && ./autogen.sh
cd ~/ros_ws && colcon build
```