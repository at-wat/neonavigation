/*
 * Copyright (c) 2018, the neonavigation authors
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

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <neonavigation_common/compatibility.h>

class LargeMapToMapNode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Publisher pub_map_;
  ros::Subscriber sub_largemap_;
  ros::Timer timer_;

  nav_msgs::OccupancyGrid::ConstPtr large_map_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  std::string robot_frame_;

  int width_;
  bool round_local_map_;
  bool simulate_occlusion_;
  std::map<size_t, std::vector<size_t>> occlusion_table_;

public:
  LargeMapToMapNode()
    : pnh_("~")
    , nh_()
    , tfl_(tfbuf_)
  {
    neonavigation_common::compat::checkCompatMode();
    pnh_.param("robot_frame", robot_frame_, std::string("base_link"));

    pub_map_ = neonavigation_common::compat::advertise<nav_msgs::OccupancyGrid>(
        nh_, "map_local",
        pnh_, "map", 1, true);
    sub_largemap_ = nh_.subscribe("map", 2, &LargeMapToMapNode::cbLargeMap, this);

    pnh_.param("width", width_, 30);
    pnh_.param("round_local_map", round_local_map_, false);
    pnh_.param("simulate_occlusion", simulate_occlusion_, false);

    for (size_t addr = 0; addr < static_cast<size_t>(width_ * width_); ++addr)
    {
      const int ux = addr % width_;
      const int uy = addr / width_;
      const float x = ux - width_ / 2.0;
      const float y = uy - width_ / 2.0;
      const float l = std::sqrt(x * x + y * y);
      for (float r = 1.0; r < l - 1.0; r += 0.5)
      {
        const float x2 = x * r / l;
        const float y2 = y * r / l;
        const int ux2 = x2 + width_ / 2.0;
        const int uy2 = y2 + width_ / 2.0;
        const int addr2 = uy2 * width_ + ux2;
        occlusion_table_[addr2].push_back(addr);
      }
    }
    for (auto& cell : occlusion_table_)
    {
      std::sort(cell.second.begin(), cell.second.end());
      cell.second.erase(std::unique(cell.second.begin(), cell.second.end()), cell.second.end());
      const auto self_it = std::find(cell.second.begin(), cell.second.end(), cell.first);
      if (self_it != cell.second.end())
        cell.second.erase(self_it);
    }

    double hz;
    pnh_.param("hz", hz, 1.0);
    timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &LargeMapToMapNode::cbTimer, this);
  }

private:
  void cbTimer(const ros::TimerEvent& event)
  {
    publishMap();
  }
  void cbLargeMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    large_map_ = msg;
  }
  void publishMap()
  {
    if (!large_map_)
      return;
    tf2::Stamped<tf2::Transform> trans;
    try
    {
      tf2::fromMsg(tfbuf_.lookupTransform(large_map_->header.frame_id, robot_frame_, ros::Time(0)), trans);
    }
    catch (tf2::TransformException& e)
    {
      return;
    }

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = large_map_->header.frame_id;
    map.header.stamp = ros::Time::now();
    map.info = large_map_->info;
    map.info.width = width_;
    map.info.height = width_;

    const auto pos = trans.getOrigin();
    const float x = static_cast<int>(pos.x() / map.info.resolution) * map.info.resolution;
    const float y = static_cast<int>(pos.y() / map.info.resolution) * map.info.resolution;
    map.info.origin.position.x = x - map.info.width * map.info.resolution * 0.5;
    map.info.origin.position.y = y - map.info.height * map.info.resolution * 0.5;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(width_ * width_);

    const int gx =
        std::lround((map.info.origin.position.x - large_map_->info.origin.position.x) / map.info.resolution);
    const int gy =
        std::lround((map.info.origin.position.y - large_map_->info.origin.position.y) / map.info.resolution);
    const float half_width = width_ / 2.0;

    for (int y = gy; y < gy + width_; ++y)
    {
      for (int x = gx; x < gx + width_; ++x)
      {
        const int lx = x - gx;
        const int ly = y - gy;
        const size_t addr = ly * width_ + lx;
        const size_t addr_large = y * large_map_->info.width + x;
        if (round_local_map_ &&
            std::pow(lx - half_width, 2) + std::pow(ly - half_width, 2) > std::pow(half_width, 2))
        {
          map.data[addr] = -1;
        }
        else if (x < 0 || y < 0 ||
                 x >= static_cast<int>(large_map_->info.width) ||
                 y >= static_cast<int>(large_map_->info.height))
        {
          map.data[addr] = -1;
        }
        else
        {
          map.data[addr] = large_map_->data[addr_large];
        }
      }
    }
    if (simulate_occlusion_)
    {
      for (size_t addr = 0; addr < static_cast<size_t>(width_ * width_); ++addr)
      {
        if (map.data[addr] == 100)
        {
          for (auto a : occlusion_table_[addr])
          {
            map.data[a] = -1;
          }
        }
      }
    }

    pub_map_.publish(map);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "largemap_to_map");

  LargeMapToMapNode conv;
  ros::spin();

  return 0;
}
