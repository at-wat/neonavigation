# map_organizer package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## pointcloud_to_maps

pointcloud_to_maps node detects floors from given pointcloud and publishes layered OccupancyGrid.

### Subscribed topics

* ~/map_cloud (new: mapcloud) [sensor_msgs::PointCloud2]

### Published topics

* maps [map_organizer_msgs::OccupancyGridArray]
* map? [nav_msgs::OccupancyGrid]

### Services


### Called services


### Parameters

* points_thresh_rate (double, default 0.5)
  > Layers with larger numbers of points than `(max * rate)` are extracted as floors.
* floor_area_thresh_rate (double, default 0.8)
  > Layers with `(max * rate)` are extracted as floors.
* robot_height (double, default 1.0)
  > Points with `(floor_height + floor_tolerance)` to `floor_height` of height are assumed as walls.
* floor_height (double, default 0.1)
  > Points with `+-floor_height` of height are assumed as floors.
* floor_tolerance (double, default 0.2)
  > Points with `floor_height` to `(floor_height + floor_tolerance)` of height are ignored.
* min_floor_area (double, default 100.0)
  > Minimum floor area (m^2).


----

## tie_maps

tie_maps node loads maps from files and ties into layered OccupancyGrid.

### Subscribed topics


### Published topics

* maps [map_organizer_msgs::OccupancyGridArray]
* map? [nav_msgs::OccupancyGrid]

### Services


### Called services


### Parameters

* "map_files" (string, default: std::string(""))
* "frame_id" (string, default: std::string("map"))

----

## save_maps

save_maps saves layered OccupancyGrid to map files.

### Subscribed topics

* ~/maps (new: maps) [map_organizer_msgs::OccupancyGridArray]

### Published topics


### Services


### Called services


### Parameters


----

## select_map

select_map node publishes the desired layer from layered OccupancyGrid.

### Subscribed topics

* /maps (new: maps) [map_organizer_msgs::OccupancyGridArray]
* ~/floor (new: floor) [std_msgs::Int32]

### Published topics

* /map (new: map) [nav_msgs::OccupancyGrid]
* /tf

### Services


### Called services


### Parameters


----

## pose_transform

pose_transform transforms given pose into the desired tf-frame.
This node is useful to convert rviz initialpose output to desired map frame.

### Subscribed topics

* ~/pose_in (new: pose_in) [geometry_msgs::PoseWithCovarianceStamped]
* /tf

### Published topics

* ~/pose_out (new: pose_out) [geometry_msgs::PoseWithCovarianceStamped]

### Services


### Called services


### Parameters

* "to_frame" (string, default: std::string("map"))
