# neonavigation meta-package

![Build Status](https://github.com/at-wat/neonavigation/workflows/build/badge.svg)
[![Codecov](https://codecov.io/gh/at-wat/neonavigation/branch/master/graph/badge.svg)](https://codecov.io/gh/at-wat/neonavigation)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


ROS meta-package for autonomous vehicle navigation.

## Important notices

- Topic/service namespace model is migrated to ROS recommended style.
  See each package's README and runtime migration messages from the nodes.

## Install

- **Note 1: neonavigation_msgs meta-package is required to build neonavigation meta-package.**
- **Note 2: neonavigation_rviz_plugins meta-package is required to visualize PathWithVelocity message used between planner_3d and trajectory_tracker.**

```shell
# clone
cd /path/to/your/catkin_ws/src
git clone https://github.com/at-wat/neonavigation.git
git clone https://github.com/at-wat/neonavigation_msgs.git
git clone https://github.com/at-wat/neonavigation_rviz_plugins.git

# build
cd /path/to/your/catkin_ws
rosdep install --from-paths src --ignore-src -y  # Install dependencies
catkin_make -DCMAKE_BUILD_TYPE=Release  # Release build is recommended
```

## Demo

A quick demonstration with a simple simulated robot is available.

```
roslaunch neonavigation_launch demo.launch
```

![Rviz image of the demo](https://github.com/at-wat/neonavigation/blob/master/neonavigation_launch/doc/images/demo.png?raw=true)

## Packages

### [costmap_cspace](costmap_cspace/README.md)

3-DOF configuration space costmap handler.

### [planner_cspace](planner_cspace/README.md)

2-D/3-DOF seamless global-local path and motion planner and serial joint collision avoidance.

### [safety_limiter](safety_limiter/README.md)

Collision prevention control.

### [trajectory_tracker](trajectory_tracker/README.md)

Path following control and path handling. 

### [map_organizer](map_organizer/README.md)

Layered map handler.

### [track_odometry](track_odometry/README.md)

Slip compensation for vehicle odometry.

### [obj_to_pointcloud](obj_to_pointcloud/README.md)

Obj surface data to pointcloud converter.

### [neonavigation_launch](neonavigation_launch/README.md)

Sample launch files.

## References

A. Watanabe, D. Endo, G. Yamauchi and K. Nagatani, "*Neonavigation meta-package: 2-D/3-DOF seamless global-local planner for ROS — Development and field test on the representative offshore oil plant,*" 2016 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR), Lausanne, Switzerland, 2016, pp. 86-91.
(doi: 10.1109/SSRR.2016.7784282)

## Contributing

*neonavigation meta-package* is developed under [GitHub flow](https://guides.github.com/introduction/flow/).
Feel free to open new Issue and/or Pull Request.

The code in this repository is following [ROS C++ Style Guide](https://wiki.ros.org/CppStyleGuide).
A configuration file for clang-format is available at https://github.com/seqsense/ros_style/.

## License

*neonavigation meta-package* is available under BSD license.
