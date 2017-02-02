----
# map_organizer package

## pointcloud_to_maps

pointcloud_to_maps node detects floors from given pointcloud and publishes layered OccupancyGrid.

### Sources

* src/pointcloud_to_maps.cpp

### Subscribed topics

* ~/map_cloud (sensor_msgs::PointCloud2)

### Published topics

* /maps (map_organizer::OccupancyGridArray)
* /map? (nav_msgs::OccupancyGrid)

### Services


### Called services


### Parameters


----

## tie_maps

tie_maps node loads maps from files and ties into layered OccupancyGrid.

### Sources

* src/tie_maps.cpp

### Subscribed topics


### Published topics

* /maps (map_organizer::OccupancyGridArray)
* /map? (nav_msgs::OccupancyGrid)

### Services


### Called services


### Parameters

* "map_files" (string, default: std::string(""))
* "frame_id" (string, default: std::string("map"))

----

## save_maps

save_maps saves layered OccupancyGrid to map files.

### Sources

* src/save_maps.cpp

### Subscribed topics

* ~/maps (map_organizer::OccupancyGridArray)

### Published topics


### Services


### Called services


### Parameters


----

## select_map

select_map node publishes the desired layer from layered OccupancyGrid.

### Sources

* src/select_map.cpp

### Subscribed topics

* /maps (map_organizer::OccupancyGridArray)
* ~/floor (std_msgs::Int32)

### Published topics

* /map (nav_msgs::OccupancyGrid)
* /tf

### Services


### Called services


### Parameters


----

## pose_transform

pose_transform transforms given pose into the desired tf-frame.
This node is useful to convert rviz initialpose output to desired map frame.

### Sources

* src/pose_transform.cpp

### Subscribed topics

* ~/pose_in (geometry_msgs::PoseWithCovarianceStamped)
* /tf

### Published topics

* ~/pose_out (geometry_msgs::PoseWithCovarianceStamped)

### Services


### Called services


### Parameters

* "to_frame" (string, default: std::string("map"))

----

