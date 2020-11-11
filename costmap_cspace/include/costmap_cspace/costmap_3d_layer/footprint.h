/*
 * Copyright (c) 2014-2019, the neonavigation authors
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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_FOOTPRINT_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_FOOTPRINT_H

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <xmlrpcpp/XmlRpcValue.h>

#include <costmap_cspace/costmap_3d_layer/base.h>
#include <costmap_cspace/cspace3_cache.h>
#include <costmap_cspace/polygon.h>

namespace costmap_cspace
{
class Costmap3dLayerFootprint : public Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerFootprint>;

protected:
  float footprint_radius_;
  geometry_msgs::PolygonStamped footprint_;
  float linear_expand_;
  float linear_spread_;
  Polygon footprint_p_;
  bool keep_unknown_;

  CSpace3Cache cs_template_;
  int range_max_;
  std::vector<bool> unknown_buf_;

public:
  Costmap3dLayerFootprint()
    : linear_expand_(0.0)
    , linear_spread_(0.0)
    , keep_unknown_(false)
    , range_max_(0)
  {
  }
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
    setExpansion(
        static_cast<double>(config["linear_expand"]),
        static_cast<double>(config["linear_spread"]));
    setFootprint(costmap_cspace::Polygon(config["footprint"]));
    if (config.hasMember("keep_unknown"))
      setKeepUnknown(config["keep_unknown"]);
  }
  void setKeepUnknown(const bool keep_unknown)
  {
    keep_unknown_ = keep_unknown;
  }
  void setExpansion(
      const float linear_expand,
      const float linear_spread)
  {
    linear_expand_ = linear_expand;
    linear_spread_ = linear_spread;

    ROS_ASSERT(linear_expand >= 0.0);
    ROS_ASSERT(std::isfinite(linear_expand));
    ROS_ASSERT(linear_spread >= 0.0);
    ROS_ASSERT(std::isfinite(linear_spread));
  }
  void setFootprint(const Polygon footprint)
  {
    footprint_p_ = footprint;
    footprint_radius_ = footprint.radius();
    footprint_ = footprint.toMsg();
  }
  Polygon& getFootprint()
  {
    return footprint_p_;
  }
  const geometry_msgs::PolygonStamped& getFootprintMsg() const
  {
    return footprint_;
  }
  float getFootprintRadius() const
  {
    return footprint_radius_;
  }
  int getRangeMax() const
  {
    return range_max_;
  }
  const CSpace3Cache& getTemplate() const
  {
    return cs_template_;
  }
  void setMapMetaData(const costmap_cspace_msgs::MapMetaData3D& info)
  {
    ROS_ASSERT(footprint_p_.v.size() > 2);

    range_max_ =
        std::ceil((footprint_radius_ + linear_expand_ + linear_spread_) / info.linear_resolution);
    cs_template_.reset(range_max_, range_max_, info.angle);

    // C-Space template
    for (size_t yaw = 0; yaw < info.angle; yaw++)
    {
      for (int y = -range_max_; y <= range_max_; y++)
      {
        for (int x = -range_max_; x <= range_max_; x++)
        {
          auto f = footprint_p_;
          f.move(x * info.linear_resolution,
                 y * info.linear_resolution,
                 yaw * info.angular_resolution);
          Vec p;
          p[0] = 0;
          p[1] = 0;
          if (f.inside(p))
          {
            cs_template_.e(x, y, yaw) = 100;
          }
          else
          {
            const float d = f.dist(p);
            if (d < linear_expand_)
            {
              cs_template_.e(x, y, yaw) = 100;
            }
            else if (d < linear_expand_ + linear_spread_)
            {
              cs_template_.e(x, y, yaw) = 100 - (d - linear_expand_) * 100 / linear_spread_;
            }
            else
            {
              cs_template_.e(x, y, yaw) = 0;
            }
          }
        }
      }
      if (footprint_radius_ == 0)
        cs_template_.e(0, 0, yaw) = 100;
    }
  }

protected:
  bool updateChain(const bool output)
  {
    return false;
  }
  void updateCSpace(
      const nav_msgs::OccupancyGrid::ConstPtr& map,
      const UpdatedRegion& region)
  {
    if (root_)
      generateCSpace(map_, map, region);
    else
      generateCSpace(map_overlay_, map, region);
  }
  virtual void generateCSpace(
      CSpace3DMsg::Ptr map,
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const UpdatedRegion& region)
  {
    ROS_ASSERT(ang_grid_ > 0);
    clearTravelableArea(map, msg);
    for (size_t yaw = 0; yaw < map->info.angle; yaw++)
    {
      generateSpecifiedCSpace(map, msg, yaw);
    }
  }

  // Clear travelable area in OVERWRITE mode
  void clearTravelableArea(
      CSpace3DMsg::Ptr map,
      const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    if (overlay_mode_ != OVERWRITE || root_)
    {
      return;
    }
    const int ox =
        std::lround((msg->info.origin.position.x - map->info.origin.position.x) /
                    map->info.linear_resolution);
    const int oy =
        std::lround((msg->info.origin.position.y - map->info.origin.position.y) /
                    map->info.linear_resolution);
    const double resolution_scale = msg->info.resolution / map->info.linear_resolution;

    for (size_t yaw = 0; yaw < map->info.angle; yaw++)
    {
      for (size_t i = 0; i < msg->data.size(); i++)
      {
        const auto& val = msg->data[i];
        if (val < 0)
          continue;

        const int x = std::lround((i % msg->info.width) * resolution_scale);
        if (x < range_max_ || static_cast<int>(msg->info.width) - range_max_ <= x)
          continue;
        const int y = std::lround((i / msg->info.width) * resolution_scale);
        if (y < range_max_ || static_cast<int>(msg->info.height) - range_max_ <= y)
          continue;

        const int res_up = std::ceil(resolution_scale);
        for (int yp = 0; yp < res_up; yp++)
        {
          const int y2 = y + oy + yp;
          if (static_cast<size_t>(y2) >= map->info.height)
            continue;
          for (int xp = 0; xp < res_up; xp++)
          {
            const int x2 = x + ox + xp;
            if (static_cast<size_t>(x2) >= map->info.width)
              continue;

            map->getCost(x2, y2, yaw) = -1;
          }
        }
      }
    }
  }
  void generateSpecifiedCSpace(
      CSpace3DMsg::Ptr map,
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const size_t yaw)
  {
    const int ox =
        std::lround((msg->info.origin.position.x - map->info.origin.position.x) /
                    map->info.linear_resolution);
    const int oy =
        std::lround((msg->info.origin.position.y - map->info.origin.position.y) /
                    map->info.linear_resolution);
    const double resolution_scale = msg->info.resolution / map->info.linear_resolution;
    if (keep_unknown_)
    {
      unknown_buf_.resize(msg->data.size());
      for (size_t i = 0; i < msg->data.size(); i++)
      {
        const int gx = std::lround((i % msg->info.width) * resolution_scale) + ox;
        const int gy = std::lround((i / msg->info.width) * resolution_scale) + oy;
        if (static_cast<size_t>(gx) >= map->info.width ||
            static_cast<size_t>(gy) >= map->info.height)
          continue;
        // If the cell is unknown in parent map and also updated map,
        // the cell is never measured.
        // Never measured cells should be marked unknown
        // to be correctly processed in the planner.
        unknown_buf_[i] = msg->data[i] < 0 && map->getCost(gx, gy, yaw) < 0;
      }
    }
    for (size_t i = 0; i < msg->data.size(); i++)
    {
      const int8_t val = msg->data[i];
      if (val < 0)
      {
        continue;
      }
      const int gx = std::lround((i % msg->info.width) * resolution_scale) + ox;
      const int gy = std::lround((i / msg->info.width) * resolution_scale) + oy;
      if (static_cast<size_t>(gx) >= map->info.width ||
          static_cast<size_t>(gy) >= map->info.height)
        continue;
      if (val == 0)
      {
        int8_t& m = map->getCost(gx, gy, yaw);
        if (m < 0)
          m = 0;
        continue;
      }

      const int map_x_min = std::max(gx - range_max_, 0);
      const int map_x_max = std::min(gx + range_max_, static_cast<int>(map->info.width) - 1);
      const int map_y_min = std::max(gy - range_max_, 0);
      const int map_y_max = std::min(gy + range_max_, static_cast<int>(map->info.height) - 1);
      for (int map_y = map_y_min; map_y <= map_y_max; ++map_y)
      {
        // Use raw pointers for faster iteration
        int8_t* cost_addr = &(map->getCost(map_x_min, map_y, yaw));
        const char* cs_addr = &(cs_template_.e(map_x_min - gx, map_y - gy, yaw));
        for (int n = 0; n <= map_x_max - map_x_min; ++n, ++cost_addr, ++cs_addr)
        {
          const int8_t c = *cs_addr * val / 100;
          if (c > 0 && *cost_addr < c)
            *cost_addr = c;
        }
      }
    }
    if (keep_unknown_)
    {
      for (size_t i = 0; i < unknown_buf_.size(); i++)
      {
        if (!unknown_buf_[i])
          continue;
        const int gx = std::lround((i % msg->info.width) * resolution_scale) + ox;
        const int gy = std::lround((i / msg->info.width) * resolution_scale) + oy;
        if (static_cast<size_t>(gx) >= map->info.width ||
            static_cast<size_t>(gy) >= map->info.height)
          continue;
        map->getCost(gx, gy, yaw) = -1;
      }
    }
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_FOOTPRINT_H
