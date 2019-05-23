/*
 * Copyright (c) 2014-2018, the neonavigation authors
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

#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

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

  CSpace3Cache cs_template_;
  int range_max_;

public:
  Costmap3dLayerFootprint()
    : linear_expand_(0.0)
    , linear_spread_(0.0)
    , range_max_(0)
  {
  }
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
    setExpansion(
        static_cast<double>(config["linear_expand"]),
        static_cast<double>(config["linear_spread"]));
    setFootprint(costmap_cspace::Polygon(config["footprint"]));
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
        ceilf((footprint_radius_ + linear_expand_ + linear_spread_) / info.linear_resolution);
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
      gemerateCSpace(map_, map, region);
    else
      gemerateCSpace(map_overlay_, map, region);
  }
  void gemerateCSpace(
      CSpace3DMsg::Ptr map,
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const UpdatedRegion& region)
  {
    ROS_ASSERT(ang_grid_ > 0);
    const int ox =
        lroundf((msg->info.origin.position.x - map->info.origin.position.x) /
                map->info.linear_resolution);
    const int oy =
        lroundf((msg->info.origin.position.y - map->info.origin.position.y) /
                map->info.linear_resolution);
    // Clear travelable area in OVERWRITE mode
    if (overlay_mode_ == OVERWRITE && !root_)
    {
      for (size_t yaw = 0; yaw < map->info.angle; yaw++)
      {
        for (unsigned int i = 0; i < msg->data.size(); i++)
        {
          const auto& val = msg->data[i];
          if (val < 0)
            continue;

          const int x =
              lroundf((i % msg->info.width) * msg->info.resolution / map->info.linear_resolution);
          if (x < range_max_ || static_cast<int>(msg->info.width) - range_max_ <= x)
            continue;
          const int y =
              lroundf((i / msg->info.width) * msg->info.resolution / map->info.linear_resolution);
          if (y < range_max_ || static_cast<int>(msg->info.height) - range_max_ <= y)
            continue;

          const int res_up = ceilf(msg->info.resolution / map->info.linear_resolution);
          for (int yp = 0; yp < res_up; yp++)
          {
            for (int xp = 0; xp < res_up; xp++)
            {
              const int x2 = x + ox + xp;
              const int y2 = y + oy + yp;
              if (static_cast<unsigned int>(x2) >= map->info.width ||
                  static_cast<unsigned int>(y2) >= map->info.height)
                continue;

              auto& m = map->getCost(x2, y2, yaw);
              m = 0;
            }
          }
        }
      }
    }
    // Get max
    for (size_t yaw = 0; yaw < map->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < msg->data.size(); i++)
      {
        const int gx = lroundf((i % msg->info.width) * msg->info.resolution / map->info.linear_resolution) + ox;
        const int gy = lroundf((i / msg->info.width) * msg->info.resolution / map->info.linear_resolution) + oy;
        if ((unsigned int)gx >= map->info.width ||
            (unsigned int)gy >= map->info.height)
          continue;

        auto val = msg->data[i];
        if (val <= 0)
          continue;

        for (int y = -range_max_; y <= range_max_; y++)
        {
          for (int x = -range_max_; x <= range_max_; x++)
          {
            const int x2 = gx + x;
            const int y2 = gy + y;
            if (static_cast<unsigned int>(x2) >= map->info.width ||
                static_cast<unsigned int>(y2) >= map->info.height)
              continue;

            auto& m = map->getCost(x2, y2, yaw);
            const auto c = cs_template_.e(x, y, yaw) * val / 100;
            if (m < c)
              m = c;
          }
        }
      }
    }
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_FOOTPRINT_H
