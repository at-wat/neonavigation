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

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_organizer_msgs/OccupancyGridArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <neonavigation_common/compatibility.h>

class PointcloudToMapsNode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  std::map<std::string, ros::Publisher> pub_maps_;
  ros::Publisher pub_map_array_;
  ros::Subscriber sub_points_;

public:
  PointcloudToMapsNode()
    : pnh_("~")
    , nh_()
  {
    neonavigation_common::compat::checkCompatMode();
    sub_points_ = neonavigation_common::compat::subscribe(
        nh_, "mapcloud",
        pnh_, "map_cloud", 1, &PointcloudToMapsNode::cbPoints, this);
    pub_map_array_ = nh_.advertise<map_organizer_msgs::OccupancyGridArray>("maps", 1, true);
  }
  void cbPoints(const sensor_msgs::PointCloud2::Ptr& msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pc);

    double grid;
    int min_points;
    double robot_height_f;
    int robot_height;
    double floor_height_f;
    int floor_height;
    double min_floor_area;
    double floor_area_thresh_rate;
    double floor_tolerance_f;
    int floor_tolerance;
    double points_thresh_rate;

    pnh_.param("grid", grid, 0.05);
    pnh_.param("points_thresh_rate", points_thresh_rate, 0.5);
    pnh_.param("robot_height", robot_height_f, 1.0);
    pnh_.param("floor_height", floor_height_f, 0.1);
    pnh_.param("floor_tolerance", floor_tolerance_f, 0.2);
    pnh_.param("min_floor_area", min_floor_area, 100.0);
    pnh_.param("floor_area_thresh_rate", floor_area_thresh_rate, 0.8);
    robot_height = std::lround(robot_height_f / grid);
    floor_height = std::lround(floor_height_f / grid);
    floor_tolerance = std::lround(floor_tolerance_f / grid);

    std::map<int, int> hist;
    int x_min = std::numeric_limits<int>::max(), x_max = 0;
    int y_min = std::numeric_limits<int>::max(), y_max = 0;
    int h_min = std::numeric_limits<int>::max(), h_max = 0;

    for (const auto& p : pc->points)
    {
      const int h = (p.z / grid);
      const int x = (p.x / grid);
      const int y = (p.y / grid);
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
    const int max_height = h_max;
    const int min_height = h_min;
    std::map<int, float> floor_area;
    std::map<int, float> floor_runnable_area;
    for (int i = min_height; i <= max_height; i++)
      floor_area[i] = 0;

    nav_msgs::MapMetaData mmd;
    mmd.resolution = grid;
    mmd.origin.position.x = x_min * grid;
    mmd.origin.position.y = y_min * grid;
    mmd.origin.orientation.w = 1.0;
    mmd.width = x_max - x_min + 1;
    mmd.height = y_max - y_min + 1;
    ROS_INFO("width %d, height %d", mmd.width, mmd.height);
    std::vector<nav_msgs::OccupancyGrid> maps;

    int hist_max = std::numeric_limits<int>::lowest();
    for (const auto& h : hist)
      if (h.second > hist_max)
        hist_max = h.second;

    min_points = hist_max * points_thresh_rate;

    double floor_area_max = 0;
    double floor_runnable_area_max = 0;
    std::map<int, std::map<std::pair<int, int>, char>> floor;
    for (int i = min_height; i <= max_height; i++)
    {
      if (hist[i] > min_points)
      {
        for (auto& p : pc->points)
        {
          const int x = (p.x / grid);
          const int y = (p.y / grid);
          const int z = (p.z / grid);
          const auto v = std::pair<int, int>(x, y);

          if (std::abs(i - z) <= floor_height)
          {
            if (floor[i].find(v) == floor[i].end())
            {
              floor[i][v] = 0;
            }
          }
          else if (i + floor_height + floor_tolerance < z && z <= i + robot_height)
          {
            floor[i][v] = 1;
          }
        }

        int cnt = 0;
        for (const auto& m : floor[i])
        {
          if (m.second == 0)
            cnt++;
        }
        floor_runnable_area[i] = cnt * (grid * grid);
        floor_area[i] = floor[i].size() * (grid * grid);
        if (floor_area_max < floor_area[i])
          floor_area_max = floor_area[i];
        if (floor_runnable_area_max < floor_runnable_area[i])
          floor_runnable_area_max = floor_runnable_area[i];
      }
      else
      {
        floor_area[i] = 0;
      }
    }
    const double floor_area_filter = floor_runnable_area_max * floor_area_thresh_rate;
    int map_num = 0;
    for (int i = min_height; i <= max_height; i++)
    {
      if (hist[i] > min_points &&
          (i == min_height || floor_runnable_area[i - 1] <= floor_runnable_area[i]) &&
          (i == max_height || floor_runnable_area[i + 1] <= floor_runnable_area[i]))
      {
        if (floor_runnable_area[i] > floor_area_filter)
        {
          nav_msgs::OccupancyGrid map;
          map.info = mmd;
          map.info.origin.position.z = i * grid;
          map.header = msg->header;
          map.data.resize(mmd.width * mmd.height);
          for (auto& c : map.data)
            c = -1;
          for (const auto& m : floor[i])
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
    }
    ROS_INFO("Floor candidates: %d", map_num);
    auto it_prev = maps.rbegin();
    for (auto it = maps.rbegin() + 1; it != maps.rend() && it_prev != maps.rend(); it++)
    {
      const int h = it->info.origin.position.z / grid;
      const int h_prev = it_prev->info.origin.position.z / grid;
      if (std::abs(it_prev->info.origin.position.z - it->info.origin.position.z) < grid * 1.5)
      {
        // merge slopes
        for (size_t i = 0; i < it->data.size(); i++)
        {
          if (it->data[i] != 0 && it_prev->data[i] == 0)
          {
            it->data[i] = 0;
            it_prev->data[i] = -1;
          }
          else if (it->data[i] == 0 && it_prev->data[i] == 0)
          {
            it_prev->data[i] = -1;
          }
        }
        int cnt = 0;
        for (const auto c : it->data)
          if (c == 0)
            ++cnt;
        floor_runnable_area[h] = cnt * grid * grid;
        int cnt_prev = 0;
        for (const auto c : it_prev->data)
          if (c == 0)
            ++cnt_prev;
        floor_runnable_area[h_prev] = cnt_prev * grid * grid;
      }
      it_prev = it;
    }
    for (int i = max_height; i >= min_height; i--)
    {
      printf(" %6.2f ", i * grid);
      for (int j = 0; j <= 16; j++)
      {
        if (j <= hist[i] * 16 / hist_max)
          printf("#");
        else
          printf(" ");
      }
      if (floor_runnable_area[i] == 0.0)
        printf("  (%7d points)\n", hist[i]);
      else
        printf("  (%7d points, %5.2f m^2 of floor)\n", hist[i], floor_runnable_area[i]);
    }
    int num = -1;
    int floor_num = 0;
    map_organizer_msgs::OccupancyGridArray map_array;
    for (auto& map : maps)
    {
      num++;
      int h = map.info.origin.position.z / grid;
      if (floor_runnable_area[h] < min_floor_area)
      {
        ROS_WARN("floor %d (%5.2fm^2), h = %0.2fm skipped",
                 floor_num, floor_runnable_area[num], map.info.origin.position.z);
        continue;
      }

      {
        const auto map_cp = map.data;
        for (unsigned int i = 0; i < map_cp.size(); i++)
        {
          if (map_cp[i] == 0)
          {
            const int width = 6;
            int floor_width = width;
            for (int xp = -width; xp <= width; xp++)
            {
              for (int yp = -width; yp <= width; yp++)
              {
                int width_sq = xp * xp + yp * yp;
                if (width_sq > width * width)
                  continue;
                const unsigned int x = i % mmd.width + xp;
                const unsigned int y = i / mmd.width + yp;
                if (x >= mmd.width || y >= mmd.height)
                  continue;
                const int addr = x + y * mmd.width;
                if (map_cp[addr] == 100)
                {
                  if (width_sq < floor_width * floor_width)
                    floor_width = std::sqrt(width_sq);
                }
              }
            }
            floor_width--;
            for (int xp = -floor_width; xp <= floor_width; xp++)
            {
              for (int yp = -floor_width; yp <= floor_width; yp++)
              {
                if (xp * xp + yp * yp > floor_width * floor_width)
                  continue;
                const unsigned int x = i % mmd.width + xp;
                const unsigned int y = i / mmd.width + yp;
                if (x >= mmd.width || y >= mmd.height)
                  continue;
                const int addr = x + y * mmd.width;
                if (map_cp[addr] != 100)
                {
                  map.data[addr] = 0;
                }
              }
            }
          }
        }
      }

      std::string name = "map" + std::to_string(floor_num);
      pub_maps_[name] = pnh_.advertise<nav_msgs::OccupancyGrid>(name, 1, true);
      pub_maps_[name].publish(map);
      map_array.maps.push_back(map);
      ROS_WARN("floor %d (%5.2fm^2), h = %0.2fm",
               floor_num, floor_runnable_area[h], map.info.origin.position.z);
      floor_num++;
    }
    pub_map_array_.publish(map_array);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_to_maps");

  PointcloudToMapsNode p2m;
  ros::spin();

  return 0;
}
