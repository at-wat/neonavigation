# track_odometry package

## track_odometry

track_odometry node compensates translational slip based on angular slip estimation based on IMU data.

### Subscribed topics

* /odom_raw (new: odom_raw) [nav_msgs::Odometry]
* /imu (new: imu/data) [sensor_msgs::Imu]
* ~/reset_z (new: reset_odometry_z) [std_msgs::Float32]
* /tf

### Published topics

* /odom (new: odom) [nav_msgs::Odometry]
* /tf

### Services


### Called services


### Parameters

* "base_link_id" (string, default: std::string(""))
* "base_link_projected_id" (string, default: std::string("base_link_projected"))
* "z_filter" (double, default: 0.99)
* "tf_tolerance" (double, default: 0.01)
* "use_kf" (bool, default: true)
* "enable_negative_slip" (bool, default: false)
* "debug" (bool, default: false)
* "sigma_odom" (double, default: 0.005)
* "sigma_predict" (double, default: 0.5)
* "predict_filter_tc" (double, default: 1.0)

----
## tf_projection

stub
