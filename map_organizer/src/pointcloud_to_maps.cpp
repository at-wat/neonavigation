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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_organizer/OccupancyGridArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <utility>
#include <vector>

pcl::PointXYZ operator-(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
  auto c = a;
  c.x -= b.x;
  c.y -= b.y;
  c.z -= b.z;
  return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
  auto c = a;
  c.x += b.x;
  c.y += b.y;
  c.z += b.z;
  return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ &a, const float &b)
{
  auto c = a;
  c.x *= b;
  c.y *= b;
  c.z *= b;
  return c;
}

class pointcloud_to_maps
{
public:
  pointcloud_to_maps()
    : n("~")
  {
    subPoints = n.subscribe("map_cloud", 1, &pointcloud_to_maps::cbPoints, this);
    pubMapArray = n.advertise<map_organizer::OccupancyGridArray>("/maps", 1, true);
  }
  void cbPoints(const sensor_msgs::PointCloud2::Ptr &msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pc);

    double grid;
    int min_points;
    int robot_height;
    double min_floor_area;
    double floor_area_filter;

    n.param("grid", grid, 0.05);
    n.param("min_points", min_points, 5000);
    n.param("robot_height", robot_height, 20);
    n.param("min_floor_area", min_floor_area, 100.0);
    n.param("floor_area_filter", floor_area_filter, 300.0);

    std::map<int, int> hist;
    int x_min = INT_MAX, x_max = 0;
    int y_min = INT_MAX, y_max = 0;
    int h_min = INT_MAX, h_max = 0;

    for (auto &p : pc->points)
    {
      int h = (p.z / grid);
      int x = (p.x / grid);
      int y = (p.y / grid);
      if (x_min > x)
        x_min = x;
      if (y_min > y)
        y_min = y;
      if (h_min > h)
        h_min = h;
      if (x_max < x)
        x_max = x;
      if (y_max < y)
        y_max = y;
      if (h_max < h)
        h_max = h;
      if (hist.find(h) == hist.end())
        hist[h] = 0;
      hist[h]++;
    }
    int max_height = h_max;
    int min_height = h_min;
    std::map<int, float> floor_area;
    for (int i = min_height; i < max_height; i++)
      floor_area[i] = 0;

    nav_msgs::MapMetaData mmd;
    mmd.resolution = grid;
    mmd.origin.position.x = x_min * grid;
    mmd.origin.position.y = y_min * grid;
    mmd.origin.orientation.w = 1.0;
    mmd.width = x_max - x_min;
    mmd.height = y_max - y_min;
    ROS_INFO("width %d, height %d", mmd.width, mmd.height);
    std::vector<nav_msgs::OccupancyGrid> maps;

    int hist_max = INT_MIN;
    for (auto h : hist)
      if (h.second > hist_max)
        hist_max = h.second;

    int map_num = 0;
    for (int i = min_height; i < max_height; i++)
    {
      if (hist[i] > min_points)
      {
        std::map<std::pair<int, int>, char> floor;
        for (auto &p : pc->points)
        {
          int x = (p.x / grid);
          int y = (p.y / grid);
          int z = (p.z / grid);
          auto v = std::pair<int, int>(x, y);

          if (abs(i - z) <= 1)
          {
            if (floor.find(v) == floor.end())
            {
              floor[v] = 0;
            }
          }
          else if (i + 3 < z && z <= i + robot_height)
          {
            floor[v] = 1;
          }
        }
        int cnt = 0;
        for (auto &m : floor)
        {
          if (m.second == 0)
            cnt++;
        }
        floor_area[i] = cnt * (grid * grid);
        if (floor_area[i] < floor_area_filter)
        {
          floor_area[i] = 0;
        }
        else
        {
          nav_msgs::OccupancyGrid map;
          map.info = mmd;
          map.info.origin.position.z = i * grid;
          map.header = msg->header;
          map.data.resize(mmd.width * mmd.height);
          for (auto &c : map.data)
            c = -1;
          for (auto &m : floor)
          {
            int addr = (m.first.first - x_min) + (m.first.second - y_min) * mmd.width;
            if (m.second == 0)
            {
              map.data[addr] = 0;
            }
            else if (m.second == 1)
              map.data[addr] = 100;
          }

          maps.push_back(map);
          map_num++;
        }
      }
      else
      {
        floor_area[i] = 0;
      }
    }
    ROS_INFO("Floor candidates: %d", map_num);
    auto it_prev = maps.rbegin();
    for (auto it = maps.rbegin() + 1; it != maps.rend(); it++)
    {
      if (fabs(it_prev->info.origin.position.z - it->info.origin.position.z) < grid * 1.5)
      {
        // merge slopes
        for (unsigned int i = 0; i < it->data.size(); i++)
        {
          if (it->data[i] == 100 && it_prev->data[i] == 0)
          {
            it->data[i] = 0;
            it_prev->data[i] = -1;
          }
        }
      }
      it_prev = it;
    }
    std::vector<float> floor_area_ext(map_num);
    int num = 0;
    for (auto &map : maps)
    {
      auto map_exp = map.data;
      for (unsigned int i = 0; i < map_exp.size(); i++)
      {
        if (map.data[i] != 0)
        {
          unsigned int x = i % mmd.width;
          unsigned int y = i / mmd.width;
          if (x >= mmd.width)
            continue;
          if (y >= mmd.height)
            continue;
          int addr = x + y * mmd.width;
          map_exp[addr] = map.data[i];
        }
      }
      int cnt = 0;
      for (auto &c : map_exp)
        if (c == 0)
          cnt++;
      floor_area_ext[num] = cnt * grid * grid;
      num++;
    }
    for (int i = max_height - 1; i >= min_height; i--)
    {
      printf(" %6.2f ", i * grid);
      for (int j = 0; j < 16; j++)
      {
        if (j < hist[i] * 16 / hist_max)
          printf("#");
        else
          printf(" ");
      }
      printf("  (%5d points, %5.0f m^2 occupied)\n", hist[i], floor_area[i]);
    }
    num = 0;
    int floor_num = 0;
    map_organizer::OccupancyGridArray map_array;
    for (auto &map : maps)
    {
      num++;
      if (floor_area_ext[num] < min_floor_area)
        continue;
      std::string name = "map" + std::to_string(floor_num);
      pubMaps[name] = n.advertise<nav_msgs::OccupancyGrid>(name, 1, true);
      pubMaps[name].publish(map);
      map_array.maps.push_back(map);
      ROS_ERROR("floor %d (%5.2fm^2), h = %0.2fm",
                floor_num, floor_area_ext[num], map.info.origin.position.z);
      floor_num++;
    }
    pubMapArray.publish(map_array);
  }

private:
  ros::NodeHandle n;
  std::map<std::string, ros::Publisher> pubMaps;
  ros::Publisher pubMapArray;
  ros::Subscriber subPoints;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_to_maps");

  pointcloud_to_maps p2m;
  ros::spin();

  return 0;
}
