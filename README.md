<<<<<<< HEAD
#  Visual-wheel odometry and LiDAR SLAM
This work aims to imporve the odometry accuracy by fusing wheel odometry, visual odometry and imu as optional on the EKF framework.  Using cartographer with the fused odometry to build a good gridmap for localization.   [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3),  [cartographer](https://github.com/cartographer-project/cartographer) and [robot_pose_ekf](https://github.com/ros-planning/robot_pose_ekf) are used for visual odometry, LiDAR SLAM and sensor fusion. [Kalibr](https://github.com/ethz-asl/kalibr) is used for camera calibration.
The test environment is Ubuntu 18.04, ROS melodic.

## Prerequisites
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) : tested with Pangolin v0.6
- [OpenCV](https://opencv.org/) : tested with OpenCV 3.4.2.
- [Eigen3](http://eigen.tuxfamily.org.) : tested with Eigen 3.3.9.
- [Python](https://www.python.org/) :  tested with Python 2.7.17 and Python 3.6.9.
- [D435i SDK](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) : tested with Intel® RealSense™ SDK 2.0.
- [ROS Wrapper for Intel® RealSense™ Devices](https://github.com/IntelRealSense/realsense-ros) : tested with Eigen 3.3.9.

#Installation and Building
Download and install instructions can be found at  [cartographer](https://github.com/cartographer-project/cartographer), [cartographer-ros](https://github.com/cartographer-project/cartographer_ros, ), [robot_pose_ekf](https://github.com/ros-planning/robot_pose_ekf), [Kalibr](https://github.com/ethz-asl/kalibr).

Build ORB_SLAM3:
```sh
$ cd ~/ORB_SLAM3
$ ./build.sh
$ ./build_ros.sh
```


Build robot_pose_ekf , pub_odom and cartographer-ros in a catkin workspace by catkin build:
```sh
$ cd catkin_ws/
$ catkin build
```
Build Kalibr in another workspace kalibr_ws:
```sh
$ cd kalibr_ws/
$ catkin build 
```


## Calibration
Calibration targets can refer to [Aprilgrid](https://github.com/ethz-asl/kalibr/wiki/calibration-targets). Either use RGB camera in Realsense D435i to calibrate the rolling shutter camera, or RGB aligned with Depth camera in D435i.
The calibration can be run using:
RGB:
```sh
$ rosrun kalibr kalibr_calibrate_rs_cameras --bag MYROSBAG.bag --model pinhole-radtan-rs --target aprilgrid.yaml --topic /rgb/image_raw --feature-variance 1 --frame-rate 30
```


or RGB-D:
```sh
$ kalibr_calibrate_cameras --bag MYROSBAG.bag --topics /aligned-rgb/image_raw  --models pinhole-radtan --target  aprilgrid.yaml 
```
Use the calibration validator to validate the reprojection error:
```sh
$ kalibr_camera_validator --cam camchain.yaml --target target.yaml
```
Calibration for Camera-LiDAR system can refer to [Autoware](https://autoware.readthedocs.io/en/feature-documentation_rtd/DevelopersGuide/PackagesAPI/sensing/autoware_camera_lidar_calibrator.html)  or [CamLaserCalibraTool](https://github.com/MegviiRobot/CamLaserCalibraTool) to get the extrinsics of two devices.

IMU calibration needs to performed through the official SDK first, the instructions are given in [rs-imu-calibration Tool](https://github.com/IntelRealSense/librealsense/tree/development/tools/rs-imu-calibration#rs-imu-calibration-tool).

Time synchronization between MiR and external pc can be done by [master_time_sync](https://github.com/matchRos/Match_Mobile_Robotics/tree/main/general_hardware_helper/master_time_sync). If the time difference is still to large,  use [NTP Pool](https://www.pool.ntp.org/zone/de) or set a local ntp server and synchronized by command ```ntpdate``` in Linux. 

## Running with rosbag
Run the visual odometry and fusion are combined in a launch file:

```sh
$ roslaunch robot_pose_ekf robot_pose_ekf.launch
```
Run cartographer:
```sh
$ roslaunch cartographer_ros mir_100_laser2.launch

```
Play rosbag:
```sh
$ rosbag play ROSBAG.bag
```
After mapping is finished, save the map by:
```sh
$ rosservice call /finish_trajectory 0
```
Generate a pbstream file:
```sh
$ rosservice call /write_state /PATH/map.pbstream
```
Convert to a pgm file:
```sh
$ rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/PATH/pgmmap -pbstream_filename=/PATH/map.pbstream -resolution=0.05
```





=======
# masterarbeit
>>>>>>> 0e85885d989d28d44a6b1f2ba9daadb794250a96
