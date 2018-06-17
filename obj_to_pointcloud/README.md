# obj_to_pointcloud package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## obj_to_pointcloud

obj_to_pointcloud node converts obj file (surface data) to pointcloud.
This node can be used to load CAD based map for 3-D localization.

### Subscribed topics


### Published topics

* ~/cloud (new: cloud) [sensor_msgs::PointCloud2]

### Services


### Called services


### Parameters

* "frame_id" (string, default: std::string("map"))
* "objs" (string, default: std::string(""))
* "points_per_meter_sq" (double, default: 600.0)
* "downsample_grid" (double, default: 0.05)
* "offset_x" (double, default: 0.0)
* "offset_y" (double, default: 0.0)
* "offset_z" (double, default: 0.0)
* "scale" (double, default: 1.0)
