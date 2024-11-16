# Elevation Mapping Humanoid

## Overview

![](./figs/demo.gif)

This repository provides an implementation of elevation mapping for multiple humanoid robots using a single MID-360 LiDAR. It is primarily built upon [Robot-Centric Elevation Mapping](https://github.com/ANYbotics/elevation_mapping) and [Fast Lio Mid360](https://github.com/SylarAnh/fast_lio_mid360). The package generates steady, complete, and smooth elevation maps in the odometry frame, which can be further transformed to the `torso_link` frame using data from the LiDAR, IMU, and robot pose.


**Note**: The contribution of this repository is mainly on engineering and most of the credits should be attributed to the original research paper listed below. If you find this repository useful, please consider citing them:**

[1] Fankhauser P, Bloesch M, Hutter M. Probabilistic terrain mapping for mobile robots with uncertain localization[J]. IEEE Robotics and Automation Letters, 2018, 3(4): 3019-3026.

[2] Xu W, Zhang F. Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter[J]. IEEE Robotics and Automation Letters, 2021, 6(2): 3317-3324.


## Installation

### Dependencies

This package is built on the Robotic Operating System ([ROS](http://www.ros.org)), which needs to be [installed](http://wiki.ros.org) first.

Additionally, it depends on the following packages:

- [Grid Map](https://github.com/anybotics/grid_map) (grid map library for mobile robots)
- [kindr](http://github.com/anybotics/kindr) (kinematics and dynamics library for robotics)
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing)
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library)
- [Livox Ros Driver](https://github.com/Livox-SDK/livox_ros_driver) or [Livox Ros Driver2](https://github.com/Livox-SDK/livox_ros_driver2) (driver packages for connecting Livox LiDARs)

### Building

1. **Build the Livox ROS Driver:**
   
   Follow the instructions in the [Livox ROS Driver documentation](https://github.com/Livox-SDK/livox_ros_driver?tab=readme-ov-file#livox-ros-driver%E8%A7%88%E6%B2%83ros%E9%A9%B1%E5%8A%A8%E7%A8%8B%E5%BA%8F%E4%B8%AD%E6%96%87%E8%AF%B4%E6%98%8E) or [Livox ROS Driver2 documentation](https://github.com/Livox-SDK/livox_ros_driver2?tab=readme-ov-file#livox-ros-driver-2). After building, source the Livox ROS driver:

    ```
        source $Livox_ros_driver_dir$/devel/setup.bash
    ```


2. **Clone and build this repository**:
   
   Clone this repository into your catkin workspace and compile it:

    ```
        cd catkin_workspace/src
        git clone https://github.com/smoggy-P/elevation_mapping_humanoid.git
        cd ../
        catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build
        source devel/setup.bash
    ```

    **Note**: If you use Livox ROS Driver2, please replace all occurrences of `livox_ros_driver` with `livox_ros_driver2` in the following files before building:

   - `fast_lio_mid360/src/preprocess.h`
   - `fast_lio_mid360/src/preprocess.cpp`
   - `fast_lio_mid360/src/laserMapping.cpp`

### Running

Launch the elevation mapping:

```
    roslaunch elevation_mapping_demos realsense_demo.launch
```

Once the process starts successfully, you can visualize the elevation map in `rviz`. Additionally, ensure the elevation map data is being published correctly by checking the `/elevation_mapping/elevation_map` topic. The topic should contain valid data (i.e., not all `NaN` values):

```
    rostopic echo /elevation_mapping/elevation_map
```

### Adapting to Your Robot

To adapt this package to your robot, modify the static transformations in the [publish_tf.py](./fast_lio_mid360/script/publish_tf.py) script to match your robot's configuration.
