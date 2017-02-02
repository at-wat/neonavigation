----
# safety_limiter package

## safety_limiter

safety_limiter node limits vehicle velocity to avoid collisions based on the linear prediction of the motion.

### Sources

* src/safety_limiter.cpp

### Subscribed topics

* ~/cmd_vel_in (geometry_msgs::Twist)
* ~/cloud (sensor_msgs::PointCloud2)
* ~/disable (std_msgs::Bool)
* /tf

### Published topics

* ~/cmd_vel_out (geometry_msgs::Twist)
* ~/collision (sensor_msgs::PointCloud)
* ~/debug (sensor_msgs::PointCloud)

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
* "t_margin" (double, default: 0.2)
* "downsample_grid" (double, default: 0.05)
* "frame_id" (string, default: std::string("base_link"))
* "footprint" (?, default: footprint_xml)

----

