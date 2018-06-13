----
# costmap_cspace package

## costmap_3d

costmap_3d node converts 2-D (x, y) OccupancyGrid to 2-D/3-DOF (x, y, yaw) configuration space based on given footprint.

### Sources

* src/costmap_3d.cpp

### Subscribed topics

* /map (nav_msgs::OccupancyGrid)
* /map_overlay (nav_msgs::OccupancyGrid)

### Published topics

* ~/costmap (costmap_cspace_msgs::CSpace3D)
* ~/costmap_update (costmap_cspace_msgs::CSpace3DUpdate)
* ~/footprint (geometry_msgs::PolygonStamped)
* ~/debug (sensor_msgs::PointCloud)

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

