/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <string>
#include <vector>

#include <ros/ros.h>

#include <map_organizer_msgs/OccupancyGridArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtest/gtest.h>

sensor_msgs::PointCloud2 generateMapCloud()
{
  struct Point
  {
    float x, y, z;
    Point(const float ax, const float ay, const float az)
      : x(ax)
      , y(ay)
      , z(az)
    {
    }
  };
  std::vector<Point> points;
  for (float x = 0.025; x < 1.0; x += 0.05)
  {
    for (float y = 0.025; y < 1.0; y += 0.05)
    {
      points.push_back(Point(x, y, 0.0));
    }
  }
  for (float z = 0.05; z < 0.5; z += 0.05)
  {
    points.push_back(Point(0.425, 0.425, z));
    points.push_back(Point(0.575, 0.425, z));
    points.push_back(Point(0.425, 0.575, z));
    points.push_back(Point(0.575, 0.575, z));
  }
  for (float x = 0.425; x < 0.6; x += 0.05)
  {
    for (float y = 0.425; y < 0.6; y += 0.05)
    {
      points.push_back(Point(x, y, 0.5));
    }
  }
  for (float x = 0.225; x < 0.8; x += 0.05)
  {
    for (float y = 0.225; y < 0.8; y += 0.05)
    {
      points.push_back(Point(x, y, 2.0));
    }
  }
  for (float x = 0.225; x < 0.8; x += 0.05)
  {
    points.push_back(Point(x, 0.225, 2.5));
    points.push_back(Point(x, 0.775, 2.5));
    points.push_back(Point(0.225, x, 2.5));
    points.push_back(Point(0.775, x, 2.5));
  }

  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(points.size());
  cloud.height = 1;
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const Point& p : points)
  {
    *iter_x = p.x;
    *iter_y = p.y;
    *iter_z = p.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

TEST(PointcloudToMaps, Convert)
{
  ros::NodeHandle nh;

  map_organizer_msgs::OccupancyGridArray::ConstPtr maps;
  const boost::function<void(const map_organizer_msgs::OccupancyGridArray::ConstPtr&)>
      cb = [&maps](const map_organizer_msgs::OccupancyGridArray::ConstPtr& msg) -> void
  {
    maps = msg;
  };
  ros::Subscriber sub = nh.subscribe("maps", 1, cb);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("mapcloud", 1, true);

  pub.publish(generateMapCloud());
  ros::Rate rate(10.0);
  for (int i = 0; i < 50 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (maps)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_EQ("map", maps->maps[i].header.frame_id);
    ASSERT_EQ(20u, maps->maps[i].info.width);
    ASSERT_EQ(20u, maps->maps[i].info.height);
  }
  ASSERT_NEAR(0.0, maps->maps[0].info.origin.position.z, 0.05);
  for (int u = 0; u < 20; ++u)
  {
    for (int v = 0; v < 20; ++v)
    {
      if (8 <= u && u < 12 && 8 <= v && v < 12)
      {
        ASSERT_EQ(100, maps->maps[0].data[v * 20 + u])
            << u << ", " << v;
      }
      else
      {
        ASSERT_EQ(0, maps->maps[0].data[v * 20 + u])
            << u << ", " << v;
      }
    }
  }
  ASSERT_NEAR(2.0, maps->maps[1].info.origin.position.z, 0.05);
  for (int u = 0; u < 20; ++u)
  {
    for (int v = 0; v < 20; ++v)
    {
      if (4 <= u && u < 16 && 4 <= v && v < 16)
      {
        if (4 == u || u == 15 || 4 == v || v == 15)
        {
          ASSERT_EQ(100, maps->maps[1].data[v * 20 + u])
              << u << ", " << v;
        }
        else
        {
          ASSERT_EQ(0, maps->maps[1].data[v * 20 + u])
              << u << ", " << v;
        }
      }
      else
      {
        ASSERT_EQ(-1, maps->maps[1].data[v * 20 + u])
            << u << ", " << v;
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_pointcloud_to_maps");

  return RUN_ALL_TESTS();
}
