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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

#include <costmap_cspace/pointcloud_accumulator.h>
#include <neonavigation_common/compatibility.h>

class Pointcloud2ToMapNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_map_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_cloud_single_;

  nav_msgs::OccupancyGrid map_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  ros::Time published_;
  ros::Duration publish_interval_;

  double z_min_, z_max_;
  std::string global_frame_;
  std::string robot_frame_;

  unsigned int width_;
  unsigned int height_;
  float origin_x_;
  float origin_y_;

  std::vector<costmap_cspace::PointcloudAccumurator<sensor_msgs::PointCloud2>> accums_;

public:
  Pointcloud2ToMapNode()
    : nh_()
    , pnh_("~")
    , tfl_(tfbuf_)
    , accums_(2)
  {
    neonavigation_common::compat::checkCompatMode();
    pnh_.param("z_min", z_min_, 0.1);
    pnh_.param("z_max", z_max_, 1.0);
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_frame", robot_frame_, std::string("base_link"));

    double accum_duration;
    pnh_.param("accum_duration", accum_duration, 1.0);
    accums_[0].reset(ros::Duration(accum_duration));
    accums_[1].reset(ros::Duration(0.0));

    pub_map_ = neonavigation_common::compat::advertise<nav_msgs::OccupancyGrid>(
        nh_, "map_local",
        pnh_, "map", 1, true);
    sub_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "cloud", 100,
        boost::bind(&Pointcloud2ToMapNode::cbCloud, this, _1, false));
    sub_cloud_single_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "cloud_singleshot", 100,
        boost::bind(&Pointcloud2ToMapNode::cbCloud, this, _1, true));

    int width_param;
    pnh_.param("width", width_param, 30);
    height_ = width_ = width_param;
    map_.header.frame_id = global_frame_;

    double resolution;
    pnh_.param("resolution", resolution, 0.1);
    map_.info.resolution = resolution;
    map_.info.width = width_;
    map_.info.height = height_;
    map_.data.resize(map_.info.width * map_.info.height);

    double hz;
    pnh_.param("hz", hz, 1.0);
    publish_interval_ = ros::Duration(1.0 / hz);
  }

private:
  void cbCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud, const bool singleshot)
  {
    sensor_msgs::PointCloud2 cloud_global;
    geometry_msgs::TransformStamped trans;
    try
    {
      trans = tfbuf_.lookupTransform(global_frame_, cloud->header.frame_id,
                                     cloud->header.stamp, ros::Duration(0.5));
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    tf2::doTransform(*cloud, cloud_global, trans);

    const int buffer = singleshot ? 1 : 0;
    accums_[buffer].push(costmap_cspace::PointcloudAccumurator<sensor_msgs::PointCloud2>::Points(
        cloud_global, cloud_global.header.stamp));

    ros::Time now = cloud->header.stamp;
    if (published_ + publish_interval_ > now)
      return;
    published_ = now;

    float robot_z;
    try
    {
      tf2::Stamped<tf2::Transform> trans;
      tf2::fromMsg(tfbuf_.lookupTransform(global_frame_, robot_frame_, ros::Time(0)), trans);

      auto pos = trans.getOrigin();
      float x = static_cast<int>(pos.x() / map_.info.resolution) * map_.info.resolution;
      float y = static_cast<int>(pos.y() / map_.info.resolution) * map_.info.resolution;
      map_.info.origin.position.x = x - map_.info.width * map_.info.resolution * 0.5;
      map_.info.origin.position.y = y - map_.info.height * map_.info.resolution * 0.5;
      map_.info.origin.position.z = 0.0;
      map_.info.origin.orientation.w = 1.0;
      origin_x_ = x - width_ * map_.info.resolution * 0.5;
      origin_y_ = y - height_ * map_.info.resolution * 0.5;
      robot_z = pos.z();
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    for (auto& cell : map_.data)
      cell = 0;

    for (auto& accum : accums_)
    {
      for (auto& pc : accum)
      {
        sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
          if (*iter_z - robot_z < z_min_ || z_max_ < *iter_z - robot_z)
            continue;
          unsigned int x = int(
              (*iter_x - map_.info.origin.position.x) / map_.info.resolution);
          unsigned int y = int(
              (*iter_y - map_.info.origin.position.y) / map_.info.resolution);
          if (x >= map_.info.width || y >= map_.info.height)
            continue;
          map_.data[x + y * map_.info.width] = 100;
        }
      }
    }

    pub_map_.publish(map_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2_to_map");

  Pointcloud2ToMapNode conv;
  ros::spin();

  return 0;
}
