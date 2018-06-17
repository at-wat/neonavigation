----
# costmap_cspace package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## costmap_3d

costmap_3d node converts 2-D (x, y) OccupancyGrid to 2-D/3-DOF (x, y, yaw) configuration space based on given footprint.

### Subscribed topics

* /map [nav_msgs::OccupancyGrid]
* /map_overlay [nav_msgs::OccupancyGrid]

### Published topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/footprint [geometry_msgs::PolygonStamped]
* ~/debug [sensor_msgs::PointCloud]

### Services


### Called services


### Parameters

* "ang_resolution" (int, default: 16)
* "linear_expand" (double, default: 0.2f)
* "linear_spread" (double, default: 0.5f)
* "unknown_cost" (int, default: 0)
* "overlay_mode" (string, default: std::string(""))
* "footprint" (?, default: footprint_xml)

----
## laserscan_to_map

stub

----
## pointcloud2_to_map

stub

----
## largemap_to_map

stub
