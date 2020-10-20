# trajectory_tracker package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## trajectory_tracker

trajectory_tracker node controls vehicle velocity to follow given path.

### Subscribed topics

* ~/path (new: path) [nav_msgs::Path]
* ~/speed (new: speed) [std_msgs::Float32]
* /tf
* /odom [nav_msgs::Odometry] (Optional: this topic is subscribed only when "use_odom" option is true)


### Published topics

* ~/cmd_vel (new: cmd_vel) [geometry_msgs::Twist]
* ~/status [trajectory_tracker_msgs::TrajectoryTrackerStatus]
* ~/tracking [geometry_msgs::PoseStamped]

### Services


### Called services


### Parameters

* "frame_robot" (string, default: std::string("base_link"))
* "path" **deprecated** (string, default: std::string("path"))
* "cmd_vel" **deprecated** (string, default: std::string("cmd_vel"))
* "hz" (double, default: 50.0)
* "look_forward" (double, default: 0.5)
* "curv_forward" (double, default: 0.5)
* "k_dist" (double, default: 1.0)
* "k_ang" (double, default: 1.0)
* "k_avel" (double, default: 1.0)
* "gain_at_vel" (double, default: 0.0)
  > compensate k_ang according to the current linear velocity to keep convergence characteristic at the linear velocity, specified by this parameter, if `gain_at_vel != 0`
* "dist_lim" (double, default: 0.5)
* "dist_stop" (double, default: 2.0)
* "rotate_ang" (?, default: M_PI / 4)
* "max_vel" (double, default: 0.5)
* "max_angvel" (double, default: 1.0)
* "max_acc" (double, default: 1.0)
* "max_angacc" (double, default: 2.0)
* "acc_toc_factor" (double, default: 0.9)
  > decrease max_acc by this factor in time optimal control to reduce vibration due to control delay.
* "angacc_toc_factor" (double, default: 0.9)
  > decrease max_angacc by this factor in time optimal control to reduce vibration due to control delay. This parameter is valid when "use_time_optimal_control" is true.
* "path_step" (int, default: 1)
* "goal_tolerance_dist" (double, default: 0.2)
* "goal_tolerance_ang" (double, default: 0.1)
* "stop_tolerance_dist" (double, default: 0.1)
* "stop_tolerance_ang" (double, default: 0.05)
* "no_position_control_dist" (double, default: 0.0)
* "min_tracking_path" (?, default: noPosCntlDist)
* "allow_backward" (bool, default: true)
* "limit_vel_by_avel" (bool, default: false)
* "check_old_path" (bool, default: false)
* "use_odom" (bool, default: false)
  > When `use_odom` is false, trajectory_tracker publishes command velocities at a constant rate specified in "hz" option. When `use_odom` is true, it publishes command velocities just after odometry is updated. "hz" option is ignored in this mode.
* "predict_odom" (bool, default: true)
  > If true, predicted coordinates of the robot at the present timestamp are used. This parameter is valid when "use_odom" is true.
* "odom_timeout_sec" (double, default: 0.1)
  > Robot will be stopped after the duration specified in this parameter has passed since the last odometry was received. This parameter is valid when "use_odom" is true.
* "use_time_optimal_control" (bool, default: True)
  > If true, time optimal control mode is used during turning in place. Otherwise, the same algorithm used for path tracking is used.
* "time_optimal_control_future_gain" (double, default: 1.5)
  > A gain to look ahead to robot's angle used in time optimal control. This parameter is valid when "use_time_optimal_control" is true. 
* "k_ang_rotation" (double, default: 1.0)
  > "k_ang" value used during turning in place. This parameter is valid when "use_time_optimal_control" is false.
* "k_avel_rotation" (double, default: 1.0)
  > "k_avvel" value used during turning in place. This parameter is valid when "use_time_optimal_control" is false.

----

## trajectory_recorder

trajectory_recorder node generates Path message from TF.

### Subscribed topics

* /tf

### Published topics

* ~/recpath (new: path) [nav_msgs::Path]

### Services


### Called services


### Parameters

* "frame_robot" (string, default: std::string("base_link"))
* "frame_global" (string, default: std::string("map"))
* "path" (string, default: std::string("recpath"))
* "dist_interval" (double, default: 0.3)
* "ang_interval" (double, default: 1.0)

----

## trajectory_saver

trajectory_saver node saves Path message to file.

### Subscribed topics

* ~/recpath (new: path) [nav_msgs::Path]
* /tf

### Published topics


### Services


### Called services


### Parameters

* "path" **deprecated** (string, default: std::string("recpath"))
* "file" (string, default: std::string("a.path"))

----

## trajectory_server

trajectory_server node loads Path from file and publishes it.

### Subscribed topics

* /tf

### Published topics

* ~/path (new: path) [nav_msgs::Path]
* ~/status [trajectory_tracker_msgs::TrajectoryServerStatus]

### Services

* ~/ChangePath (new: change_path) [trajectory_tracker_msgs::ChangePath]

### Called services


### Parameters

* "path" **deprecated** (string, default: std::string("path"))
* "file" (string, default: std::string("a.path"))
* "hz" (?, default: double(5))
* "filter_step" (double, default: 0.0)

----


# Acknowledgement

This research was supported by a contract with the Ministry of Internal Affairs and Communications entitled, 'Novel and innovative R&D making use of brain structures'


This software was implemented to accomplish the above research.
Original idea of the implemented control scheme was published on:  
S. Iida, S. Yuta, "Vehicle command system and trajectory control for autonomous mobile robots," in *Proceedings of the 1991 IEEE/RSJ International Workshop on Intelligent Robots and Systems (IROS)*, 1991, pp. 212-217.
