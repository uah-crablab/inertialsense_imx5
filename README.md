# inertialsense_imx5
ROS2 Driver for Inertial Sense IMX5 IMUs. Functionality for GPS has not been added yet. Tested on RUG3-IMX5 IMU with ROS2 Humble. Code is heavily based on [Inertial Sense ROS1 Driver](https://github.com/inertialsense/inertial-sense-ros) with a help from [MagiicLab ROS2 Driver](https://gitlab.magiccvs.byu.edu/boatlanding/ros2-sensor-drivers/inertialsense_ros2/-/tree/ros2). Every effort is made to follow the parameter and topic names of the 'Inertial Sense ROS1 Driver. See [DID Descriptions](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/) for more information on published information. 

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

## Running the Node

```bash
ros2 run inertialsense_imx5 inertialsense_imx5_node 
```

## Topics

Topics are enabled and disabled using parameters.  By default, only the `ins` topic is published to save processor time in serializing unecessary messages.
- `imu` (sensor_msgs/Imu)
    - Imu measurements (accel and gyro) from DID_IMU or DID_IMU_MAG
- `imu_raw` (sensor_msgs/Imu)
    - Imu measurements (accel and gyro) from DID_IMU_RAW
- `imu1_raw` (sensor_msgs/Imu)
    - Imu1 measurements (accel and gyro) from DID_IMU3_RAW
- `imu2_raw` (sensor_msgs/Imu)
    - Imu2 measurements (accel and gyro) from DID_IMU3_RAW
- `imu3_raw` (sensor_msgs/Imu)
    - Imu3 measurements (accel and gyro) from DID_IMU3_RAW
- `mag` (sensor_msgs/MagneticField) 
    - magnetic field measurement from DID_MAG, DID_IMU_MAG, or DID_PIMU_MAG
- `baro` (sensor_msgs/FluidPressure)
    - barometer measurements in kPa from DID_BAROMETER
- `preint_imu` (inertial_sense_ros/DThetaVel)
    - preintegrated coning and sculling integrals of IMU measurements from DID_PIMU or DID_PIMU_MAG

## Parameters

* `port` (string, default: "/dev/ttyACM0")
  - Serial port to connect to
* `baudrate` (int, default: 921600)
  - baudrate of serial communication
* `frame_id` (string, default "body")
  - frame id of all measurements
* `enable_log` (bool, default: false)
  - enable Inertial Sense Logger - logs PPD log in .dat format
* `navigation_dt_ms` (int, default: 10)
   - milliseconds between internal navigation filter updates.  This is also determines the rate at which the topics are published.
* `imu_dt_ms` (int, default: 1)
   - IMU sample (system input data) period in milliseconds set on startup. See `startupImuDtMs` within [DID Descriptions](https://docs.inertialsense.com/user-manual/com-protocol/DID-descriptions/)
* `declination` (float, default: 0.0)
  - declination angle of local magnetic field


**Topic Configuration**

* `stream_preint_IMU_mag_` (bool, default: false)
   - Flag to stream preintegrated IMU with magnetometometer (DID_PIMU_MAG)
* `stream_IMU_mag` (bool, default: true)
   - Flag to stream both IMU and magnetometer measurements (DID_IMU_MAG)
* `stream_IMU` (bool, default: false)
   - Flag to stream IMU measurements or not (DID_IMU)
* `stream_preint_IMU` (bool, default: false)
   - Flag to stream preintegrated IMU or not (DID_PIMU)
* `stream_IMU_raw` (bool, default: false)
   - Flag to stream DID_IMU_RAW message
* `stream_IMU3_raw` (bool, default: false)
   - Flag to stream DID_IMU3_RAW message
* `stream_baro` (bool, default: false)
   - Flag to stream baro or not (DID_BAROMETER)
* `stream_mag` (bool, default: false)
   - Flag to stream magnetometer or not (DID_MAGNETOMETER)