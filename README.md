# Elevation Mapping Humanoid

## Overview

![](./figs/demo.gif)

This repository provides an implementation of elevation mapping for multiple humanoid robots using a single MID-360 LiDAR. It is primarily built upon [Robot-Centric Elevation Mapping](https://github.com/ANYbotics/elevation_mapping) and [Fast Lio Mid360](https://github.com/SylarAnh/fast_lio_mid360). The package generates steady, complete, and smooth elevation maps in the odometry frame, which can be further transformed to the `torso_link` frame using data from the LiDAR, IMU, and robot pose.

**Note**: The contribution of this repository is mainly on engineering and most of the credits should be attributed to the original research paper listed below. If you find this repository useful, please consider citing them:

```bibtex
@article{fankhauser_probabilistic_2018,
	title = {Probabilistic Terrain Mapping for Mobile Robots With Uncertain Localization},
	volume = {3},
	url = {https://ieeexplore.ieee.org/document/8392399/},
	doi = {10.1109/LRA.2018.2849506},
	pages = {3019--3026},
	journaltitle = {{IEEE} Robotics and Automation Letters},
	author = {Fankhauser, Peter and Bloesch, Michael and Hutter, Marco},
	date = {2018-10}
}

@misc{xu_fast-lio_2021,
	title = {{FAST}-{LIO}: A Fast, Robust {LiDAR}-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter},
	url = {http://arxiv.org/abs/2010.08196},
	doi = {10.48550/arXiv.2010.08196},
	publisher = {{arXiv}},
	author = {Xu, Wei and Zhang, Fu},
	date = {2021-04-14}
}
```

The repo has been applied in the following papers:
```bibtex
@misc{long_learning_2024,
	title = {Learning Humanoid Locomotion with Perceptive Internal Model},
	url = {http://arxiv.org/abs/2411.14386},
	doi = {10.48550/arXiv.2411.14386},
	publisher = {{arXiv}},
	author = {Long, Junfeng and Ren, Junli and Shi, Moji and Wang, Zirui and Huang, Tao and Luo, Ping and Pang, Jiangmiao},
	date = {2024-11-21},
}

@misc{wang_beamdojo_2025,
	title = {{BeamDojo}: Learning Agile Humanoid Locomotion on Sparse Footholds},
	url = {http://arxiv.org/abs/2502.10363},
	doi = {10.48550/arXiv.2502.10363},
	publisher = {{arXiv}},
	author = {Wang, Huayi and Wang, Zirui and Ren, Junli and Ben, Qingwei and Huang, Tao and Zhang, Weinan and Pang, Jiangmiao},
	date = {2025-02-14},
}

@misc{ren_vb-com_2025,
	title = {{VB}-Com: Learning Vision-Blind Composite Humanoid Locomotion Against Deficient Perception},
	url = {http://arxiv.org/abs/2502.14814},
	doi = {10.48550/arXiv.2502.14814},
	publisher = {{arXiv}},
	author = {Ren, Junli and Huang, Tao and Wang, Huayi and Wang, Zirui and Ben, Qingwei and Pang, Jiangmiao and Luo, Ping},
	date = {2025-02-20},
}
```
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
