/*
 * Copyright (c) 2014-2017, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <laser_geometry/laser_geometry.h>

#include <string>

#include <costmap_cspace/pointcloud_accumulator/accumulator.h>

class laserscan_to_map
{
private:
  ros::NodeHandle n;
  ros::Publisher pub_map;
  ros::Subscriber sub_scan;

  nav_msgs::OccupancyGrid map;
  tf::TransformListener tfl;
  laser_geometry::LaserProjection projector;
  ros::Time published;
  ros::Duration publish_interval;

  double z_min, z_max;
  std::string global_frame;
  std::string robot_frame;

  unsigned int width;
  unsigned int height;
  float origin_x;
  float origin_y;

  PointcloudAccumurator<sensor_msgs::PointCloud> accum;

public:
  laserscan_to_map()
    : n("~")
  {
    n.param("z_min", z_min, 0.1);
    n.param("z_max", z_max, 1.0);
    n.param("global_frame", global_frame, std::string("map"));
    n.param("robot_frame", robot_frame, std::string("base_link"));

    double accum_duration;
    n.param("accum_duration", accum_duration, 1.0);
    accum.reset(ros::Duration(accum_duration));

    pub_map = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sub_scan = n.subscribe("/scan", 2, &laserscan_to_map::cb_scan, this);

    int width_param;
    n.param("width", width_param, 30);
    height = width = width_param;
    map.header.frame_id = global_frame;

    double resolution;
    n.param("resolution", resolution, 0.1);
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.data.resize(map.info.width * map.info.height);

    double hz;
    n.param("hz", hz, 1.0);
    publish_interval = ros::Duration(1.0 / hz);
  }

private:
  void cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan)
  {
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud cloud_global;
    projector.projectLaser(*scan, cloud);
    try
    {
      tfl.waitForTransform(global_frame, cloud.header.frame_id,
                           cloud.header.stamp, ros::Duration(0.5));
      tfl.transformPointCloud(global_frame, cloud, cloud_global);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
    }
    accum.push(PointcloudAccumurator<sensor_msgs::PointCloud>::Points(
        cloud_global, cloud_global.header.stamp));

    ros::Time now = scan->header.stamp;
    if (published + publish_interval > now)
      return;
    published = now;

    try
    {
      tf::StampedTransform trans;
      tfl.lookupTransform(global_frame, robot_frame, ros::Time(0), trans);

      auto pos = trans.getOrigin();
      float x = static_cast<int>(pos.x() / map.info.resolution) * map.info.resolution;
      float y = static_cast<int>(pos.y() / map.info.resolution) * map.info.resolution;
      map.info.origin.position.x = x - map.info.width * map.info.resolution * 0.5;
      map.info.origin.position.y = y - map.info.height * map.info.resolution * 0.5;
      map.info.origin.position.z = 0.0;
      map.info.origin.orientation.w = 1.0;
      origin_x = x - width * map.info.resolution * 0.5;
      origin_y = y - height * map.info.resolution * 0.5;
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    for (auto &cell : map.data)
      cell = 0;

    for (auto &pc : accum)
    {
      for (auto &p : pc.points)
      {
        unsigned int x = int(
            (p.x - map.info.origin.position.x) / map.info.resolution);
        unsigned int y = int(
            (p.y - map.info.origin.position.y) / map.info.resolution);
        if (x >= map.info.width || y >= map.info.height)
          continue;
        map.data[x + y * map.info.width] = 100;
      }
    }

    pub_map.publish(map);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscan_to_map");

  laserscan_to_map conv;
  ros::spin();

  return 0;
}
