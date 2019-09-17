# safety_limiter package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## safety_limiter

safety_limiter node limits vehicle velocity to avoid collisions based on the linear prediction of the motion.

### Subscribed topics

* ~/cmd_vel_in (new: cmd_vel_in) [geometry_msgs::Twist]
* ~/cloud (new: cloud) [sensor_msgs::PointCloud2]
* ~/disable (new: disable_safety) [std_msgs::Bool]
* ~/watchdog_reset (new: watchdog_reset) [std_msgs::Empty]
* /tf

### Published topics

* ~/cmd_vel_out (new: cmd_vel) [geometry_msgs::Twist]
* ~/collision (new: collision) [sensor_msgs::PointCloud]

### Services


### Called services


### Parameters

* "freq" (double, default: 6.0)
* "cloud_timeout" (double, default: 0.8)
* "disable_timeout" (double, default: 0.1)
* "lin_vel" (double, default: 0.5)
* "lin_acc" (double, default: 1.0)
* "ang_vel" (double, default: 0.8)
* "ang_acc" (double, default: 1.6)
* "z_range_min" (double, default: 0.0)
* "z_range_max" (double, default: 0.5)
* "dt" (double, default: 0.1)
* "d_margin" (double, default: 0.2)
* "yaw_margin" (double, default: 0.2)
* "downsample_grid" (double, default: 0.05)
* "frame_id" (string, default: std::string("base_link"))
* "footprint" (?, default: footprint_xml)
