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

class LaserscanToMapNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_map_;
  ros::Subscriber sub_scan_;

  nav_msgs::OccupancyGrid map;
  tf::TransformListener tfl_;
  laser_geometry::LaserProjection projector_;
  ros::Time published_;
  ros::Duration publish_interval_;

  double z_min_, z_max_;
  std::string global_frame_;
  std::string robot_frame_;

  unsigned int width_;
  unsigned int height_;
  float origin_x_;
  float origin_y_;

  PointcloudAccumurator<sensor_msgs::PointCloud> accum_;

public:
  LaserscanToMapNode()
    : nh_()
    , pnh_("~")
  {
    pnh_.param("z_min", z_min_, 0.1);
    pnh_.param("z_max", z_max_, 1.0);
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_frame", robot_frame_, std::string("base_link"));

    double accum_duration;
    pnh_.param("accum_duration", accum_duration, 1.0);
    accum_.reset(ros::Duration(accum_duration));

    pub_map_ = pnh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sub_scan_ = nh_.subscribe("scan", 2, &LaserscanToMapNode::cbScan, this);

    int width_param;
    pnh_.param("width", width_param, 30);
    height_ = width_ = width_param;
    map.header.frame_id = global_frame_;

    double resolution;
    pnh_.param("resolution", resolution, 0.1);
    map.info.resolution = resolution;
    map.info.width = width_;
    map.info.height = height_;
    map.data.resize(map.info.width * map.info.height);

    double hz;
    pnh_.param("hz", hz, 1.0);
    publish_interval_ = ros::Duration(1.0 / hz);
  }

private:
  void cbScan(const sensor_msgs::LaserScan::ConstPtr &scan)
  {
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud cloud_global;
    projector_.projectLaser(*scan, cloud);
    try
    {
      tfl_.waitForTransform(global_frame_, cloud.header.frame_id,
                            cloud.header.stamp, ros::Duration(0.5));
      tfl_.transformPointCloud(global_frame_, cloud, cloud_global);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
    }
    accum_.push(PointcloudAccumurator<sensor_msgs::PointCloud>::Points(
        cloud_global, cloud_global.header.stamp));

    ros::Time now = scan->header.stamp;
    if (published_ + publish_interval_ > now)
      return;
    published_ = now;

    try
    {
      tf::StampedTransform trans;
      tfl_.lookupTransform(global_frame_, robot_frame_, ros::Time(0), trans);

      auto pos = trans.getOrigin();
      float x = static_cast<int>(pos.x() / map.info.resolution) * map.info.resolution;
      float y = static_cast<int>(pos.y() / map.info.resolution) * map.info.resolution;
      map.info.origin.position.x = x - map.info.width * map.info.resolution * 0.5;
      map.info.origin.position.y = y - map.info.height * map.info.resolution * 0.5;
      map.info.origin.position.z = 0.0;
      map.info.origin.orientation.w = 1.0;
      origin_x_ = x - width_ * map.info.resolution * 0.5;
      origin_y_ = y - height_ * map.info.resolution * 0.5;
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    for (auto &cell : map.data)
      cell = 0;

    for (auto &pc : accum_)
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

    pub_map_.publish(map);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscan_to_map");

  LaserscanToMapNode conv;
  ros::spin();

  return 0;
}
