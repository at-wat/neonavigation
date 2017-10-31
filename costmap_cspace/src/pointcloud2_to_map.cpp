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

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

#include <pointcloud_accumulator/accumulator.h>

class pointcloud2_to_map
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Publisher pub_map;
  ros::Subscriber sub_cloud;
  ros::Subscriber sub_cloud_single;

  nav_msgs::OccupancyGrid map;
  tf::TransformListener tfl;
  ros::Time published;
  ros::Duration publish_interval;

  double z_min, z_max;
  std::string global_frame;
  std::string robot_frame;

  unsigned int width;
  unsigned int height;
  float origin_x;
  float origin_y;

  std::vector<PointcloudAccumurator<sensor_msgs::PointCloud2>> accums;

public:
  pointcloud2_to_map()
    : nh()
    , pnh("~")
    , accums(2)
  {
    pnh.param("z_min", z_min, 0.1);
    pnh.param("z_max", z_max, 1.0);
    pnh.param("global_frame", global_frame, std::string("map"));
    pnh.param("robot_frame", robot_frame, std::string("base_link"));

    double accum_duration;
    pnh.param("accum_duration", accum_duration, 1.0);
    accums[0].reset(ros::Duration(accum_duration));
    accums[1].reset(ros::Duration(0.0));

    pub_map = pnh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "cloud", 100,
        boost::bind(&pointcloud2_to_map::cb_cloud, this, _1, false));
    sub_cloud_single = nh.subscribe<sensor_msgs::PointCloud2>(
        "cloud_singleshot", 100,
        boost::bind(&pointcloud2_to_map::cb_cloud, this, _1, true));

    int width_param;
    pnh.param("width", width_param, 30);
    height = width = width_param;
    map.header.frame_id = global_frame;

    double resolution;
    pnh.param("resolution", resolution, 0.1);
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.data.resize(map.info.width * map.info.height);

    double hz;
    pnh.param("hz", hz, 1.0);
    publish_interval = ros::Duration(1.0 / hz);
  }

private:
  void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr &cloud, const bool singleshot)
  {
    sensor_msgs::PointCloud2 cloud_global;
    geometry_msgs::TransformStamped trans;
    try
    {
      tf::StampedTransform trans_tf;
      tfl.waitForTransform(global_frame, cloud->header.frame_id,
                           cloud->header.stamp, ros::Duration(0.5));
      tfl.lookupTransform(global_frame, cloud->header.frame_id,
                          cloud->header.stamp, trans_tf);
      tf::transformStampedTFToMsg(trans_tf, trans);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    tf2::doTransform(*cloud, cloud_global, trans);

    const int buffer = singleshot ? 1 : 0;
    accums[buffer].push(PointcloudAccumurator<sensor_msgs::PointCloud2>::Points(
        cloud_global, cloud_global.header.stamp));

    ros::Time now = cloud->header.stamp;
    if (published + publish_interval > now)
      return;
    published = now;

    float robot_z;
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
      robot_z = pos.z();
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    for (auto &cell : map.data)
      cell = 0;

    for (auto &accum : accums)
    {
      for (auto &pc : accum)
      {
        sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
          if (*iter_z - robot_z < z_min || z_max < *iter_z - robot_z)
            continue;
          unsigned int x = int(
              (*iter_x - map.info.origin.position.x) / map.info.resolution);
          unsigned int y = int(
              (*iter_y - map.info.origin.position.y) / map.info.resolution);
          if (x >= map.info.width || y >= map.info.height)
            continue;
          map.data[x + y * map.info.width] = 100;
        }
      }
    }

    pub_map.publish(map);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud2_to_map");

  pointcloud2_to_map conv;
  ros::spin();

  return 0;
}
