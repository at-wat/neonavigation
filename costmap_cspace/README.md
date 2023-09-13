# costmap_cspace package

The topic names will be migrated to ROS recommended namespace model.
Set `/neonavigation_compatible` parameter to `1` to use new topic names.

## costmap_3d

costmap_3d node converts 2-D (x, y) OccupancyGrid to 2-D/3-DOF (x, y, yaw) configuration space based on given footprint.

### single layer mode (simple version)

#### Subscribed topics

* map [nav_msgs::OccupancyGrid]
* map_overlay [nav_msgs::OccupancyGrid]

#### Published topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/footprint [geometry_msgs::PolygonStamped]
* ~/debug [sensor_msgs::PointCloud]

#### Parameters

* "ang_resolution" (int, default: 16)
* "linear_expand" (double, default: 0.2f)
* "linear_spread" (double, default: 0.5f)
* "linear_spread_min_cost" (int, default: 0)
* "unknown_cost" (int, default: 0)
* "overlay_mode" (string, default: std::string(""))
* "footprint" (?, default: footprint_xml)

### multiple layer mode

#### Subscribed topics

* map [nav_msgs::OccupancyGrid]
* **layer name** [nav_msgs::OccupancyGrid]: Subscribed topics are named according to the layers parameters

#### Published topics

* ~/costmap (new: costmap) [costmap_cspace_msgs::CSpace3D]
* ~/costmap_update (new: costmap_update) [costmap_cspace_msgs::CSpace3DUpdate]
* ~/footprint [geometry_msgs::PolygonStamped]
* ~/debug [sensor_msgs::PointCloud]

#### Parameters

* "ang_resolution" (int, default: 16): for root layer
* "linear_expand" (double, default: 0.2f): for root layer
* "linear_spread" (double, default: 0.5f): for root layer
* "linear_spread_min_cost" (int, default: 0)
* "footprint" (?, default: footprint_xml): for root layer
* "static_layers": array of layer configurations
* "layers": array of layer configurations

Each layer configuration contains:
* "name" (string) layer name
* "type" (string) layer type name
* "parameters" layer specific parameters

Available layer types and parameters are:
- **Costmap3dLayerFootprint**: Configuration space costmap layer according to the given footprint.
  - "linear_expand" (double)
  - "linear_spread" (double)
  - "linear_spread_min_cost" (int, default: 0)
  - "footprint" (?, default: root layer's footprint)
- **Costmap3dLayerPlain**: Costmap layer without considering footpring.
  - "linear_expand" (double)
  - "linear_spread" (double)
  - "linear_spread_min_cost" (int, default: 0)
- **Costmap3dLayerOutput**: Output generated costmap at this point. In most case, this is placed at the last layer.
- **Costmap3dLayerStopPropagation**: Stop propagating parent layer's cost to the child. This can be used at the beginning of layer to ignore changes in static layers.
- **Costmap3dLayerUnknownHandle**: Set unknown cell's cost.
  - "unknown_cost" (int)

See [example parameters](https://github.com/at-wat/neonavigation/blob/master/neonavigation_launch/config/navigate.yaml).

The diagram below shows how "linear_expand", "linear_spread" and "linear_spread_min_cost" work.
"linear_spread_min_cost" can make the costs of grids near obstacles higher to avoid other costs such as preferences overriding the costs of these grids.

![Screenshot from 2023-09-12 17-19-52](https://github.com/at-wat/neonavigation/assets/8833040/58001b49-b603-47e9-92e7-0517f0aaf8ba)

----
## laserscan_to_map

stub

----
## pointcloud2_to_map

stub

----
## largemap_to_map

stub
