# planner_cspace package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## planner_3d

planner_3d node provides 2-D/3-DOF seamless global-local path and motion planner.

### Subscribed topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/goal (new: move_base_simple/goal) [geometry_msgs::PoseStamped]
* /tf

### Published topics

* ~/path (new: path) [nav_msgs::Path]
* ~/debug [sensor_msgs::PointCloud]
    > debug output of planner internal costmap
* ~/remembered [sensor_msgs::PointCloud]
    > debug output of obstacles probability estimated by BBF
* ~/path_start [geometry_msgs::PoseStamped]
* ~/path_end [geometry_msgs::PoseStamped]
* ~/status [planner_cspace_msgs::PlannerStatus]

### Services

* ~/forget (new: forget_planning_cost) [std_srvs::Empty]

### Called services


### Parameters

* "goal_tolerance_lin" (double, default: 0.05)
* "goal_tolerance_ang" (double, default: 0.1)
* "goal_tolerance_ang_finish" (double, default: 0.05)
* "unknown_cost" (int, default: 100)
* "hist_cnt_max" (int, default: 20)
* "hist_cnt_thres" (int, default: 19)
* "hist_cost" (int, default: 90)
* "hist_ignore_range" (double, default: 1.0)
* "remember_updates" (bool, default: false)
* "local_range" (double, default: 2.5)
* "longcut_range" (double, default: 0.0)
* "esc_range" (double, default: 0.25)
* "find_best" (bool, default: true)
* "pos_jump" (double, default: 1.0)
* "yaw_jump" (double, default: 1.5)
* "jump_detect_frame" (string, default: base_link)
* "force_goal_orientation" (bool, default: true)
* "temporary_escape" (bool, default: true)
* "fast_map_update" (bool, default: false)
* "debug_mode" (string, default: std::string("cost_estim"))
    > debug output data type
    > - "hyst": path hysteresis cost
    > - "cost_estim": estimated cost to the goal used as A\* heuristic function
* "queue_size_limit" (int, default: 0)
* "antialias_start" (bool, default: false)
    > If enabled, the planner searches path from multiple surrounding grids within the grid size to reduce path chattering.

----

## planner_2dof_serial_joints

planner_2dof_serial_joints provides collision avoidance for 2-DOF serial joint (e.g. controlling interfering pairs of sub-tracks for tracked vehicle.)

### Subscribed topics

* ~/trajectory_in (new: trajectory_in) [trajectory_msgs::JointTrajectory]
* ~/joint (new: joint_states) [sensor_msgs::JointState]
* /tf

### Published topics

* ~/trajectory_out (new: joint_trajectory) [trajectory_msgs::JointTrajectory]
* ~/status [planner_cspace_msgs::PlannerStatus]

### Services


### Called services


### Parameters

* "resolution" (int, default: 128)
* "debug_aa" (bool, default: false)
* "replan_interval" (double, default: 0.2)
* "queue_size_limit" (int, default: 0)
* "link0_name" (string, default: std::string("link0"))
* "link1_name" (string, default: std::string("link1"))
* "point_vel_mode" (string, default: std::string("prev"))
* "range" (int, default: 8)
* "num_groups" (int, default: 1)

----

## dummy_robot

stub

----

## patrol

stub
