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

#ifndef COSTMAP_3D_LAYER_BASE_H
#define COSTMAP_3D_LAYER_BASE_H

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace/CSpace3D.h>
#include <costmap_cspace/CSpace3DUpdate.h>

#include <cspace3_cache.h>

namespace costmap_cspace
{
class Costmap3dLayerBase
{
public:
  enum map_overlay_mode
  {
    OVERWRITE,
    MAX
  };

  using Ptr = std::shared_ptr<Costmap3dLayerBase>;

protected:
  int ang_grid_;
  float linear_expand_;
  float linear_spread_;
  map_overlay_mode overlay_mode_;

  costmap_cspace::CSpace3Cache cs_;
  int range_max_;

  costmap_cspace::CSpace3D::Ptr map_;
  costmap_cspace::CSpace3D::Ptr map_overlay_;
  nav_msgs::OccupancyGrid map_base_;

  Costmap3dLayerBase::Ptr child_;

public:
  Costmap3dLayerBase()
    : ang_grid_(-1)
    , linear_expand_(0.0)
    , linear_spread_(0.0)
    , overlay_mode_(map_overlay_mode::MAX)
    , map_(new costmap_cspace::CSpace3D)
    , map_overlay_(new costmap_cspace::CSpace3D)
  {
  }
  void setCSpaceConfig(
      const int ang_resolution,
      const float linear_expand,
      const float linear_spread,
      const map_overlay_mode overlay_mode)
  {
    ang_grid_ = ang_resolution;
    linear_expand_ = linear_expand;
    linear_spread_ = linear_spread;
    overlay_mode_ = overlay_mode;
  }
  virtual void generateCSpaceTemplate(const costmap_cspace::MapMetaData3D &info) = 0;
  void setBaseMap(const nav_msgs::OccupancyGrid &base_map)
  {
    assert(base_map.data.size() >= base_map.info.width * base_map.info.height);
    map_base_ = base_map;

    map_->header = map_base_.header;
    map_->info.width = map_base_.info.width;
    map_->info.height = map_base_.info.height;
    map_->info.angle = ang_grid_;
    map_->info.linear_resolution = map_base_.info.resolution;
    map_->info.angular_resolution = 2.0 * M_PI / ang_grid_;
    map_->info.origin = map_base_.info.origin;
    map_->data.resize(map_base_.data.size() * map_->info.angle);

    generateCSpaceTemplate(map_->info);

    for (size_t yaw = 0; yaw < map_->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map_base_.data.size(); i++)
      {
        map_->data[i + yaw * map_base_.data.size()] = 0;
      }
    }
    gemerateCSpace(map_, map_base_);
    for (size_t yaw = 0; yaw < map_->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map_base_.data.size(); i++)
      {
        if (map_base_.data[i] < 0)
        {
          map_->data[i + yaw * map_base_.data.size()] = -1;
        }
      }
    }
    *map_overlay_ = *map_;
    if (child_)
      child_->setBaseMapChain();
  }
  void registerChild(Costmap3dLayerBase::Ptr child)
  {
    child_ = child;
    child_->setMap(getMapOverlay());
  }
  costmap_cspace::CSpace3DUpdate processMapOverlay(const nav_msgs::OccupancyGrid &msg)
  {
    const int ox = lroundf((msg.info.origin.position.x - map_->info.origin.position.x) / map_->info.linear_resolution);
    const int oy = lroundf((msg.info.origin.position.y - map_->info.origin.position.y) / map_->info.linear_resolution);
    *map_overlay_ = *map_;
    gemerateCSpace(map_overlay_, msg, true);

    const int w = lroundf(msg.info.width * msg.info.resolution / map_->info.linear_resolution);
    const int h = lroundf(msg.info.height * msg.info.resolution / map_->info.linear_resolution);
    return generateUpdateMsg(map_overlay_, ox, oy, 0, w, h, map_->info.angle, msg.header.stamp);
  }
  costmap_cspace::CSpace3D::Ptr getMap()
  {
    return map_;
  }
  void setMap(costmap_cspace::CSpace3D::Ptr map)
  {
    map_ = map;
  }
  costmap_cspace::CSpace3D::Ptr getMapOverlay()
  {
    return map_overlay_;
  }
  CSpace3Cache &getTemplate()
  {
    return cs_;
  }
  int getRangeMax() const
  {
    return range_max_;
  }
  int getAngularGrid() const
  {
    return ang_grid_;
  }
  int8_t getCost(const int &x, const int &y, const int &yaw) const
  {
    return getCost(x, y, yaw, map_overlay_);
  }
  int8_t getCost(const int &x, const int &y, const int &yaw, costmap_cspace::CSpace3D::ConstPtr map) const
  {
    assert(static_cast<size_t>(yaw) < map->info.angle);
    assert(static_cast<size_t>(x) < map->info.width);
    assert(static_cast<size_t>(y) < map->info.height);

    const size_t addr = (yaw * map->info.height + y) * map->info.width + x;
    assert(addr < map->data.size());

    return map->data[addr];
  }
  int8_t &getCostRef(const int &x, const int &y, const int &yaw)
  {
    return getCostRef(x, y, yaw, map_overlay_);
  }
  int8_t &getCostRef(const int &x, const int &y, const int &yaw, costmap_cspace::CSpace3D::Ptr map)
  {
    assert(static_cast<size_t>(yaw) < map->info.angle);
    assert(static_cast<size_t>(x) < map->info.width);
    assert(static_cast<size_t>(y) < map->info.height);

    const size_t addr = (yaw * map->info.height + y) * map->info.width + x;
    assert(addr < map->data.size());

    return map->data[addr];
  }

protected:
  void setBaseMapChain()
  {
    generateCSpaceTemplate(map_->info);
    *map_overlay_ = *map_;
    if (child_)
      child_->setBaseMapChain();
  }
  void gemerateCSpace(costmap_cspace::CSpace3D::Ptr map, const nav_msgs::OccupancyGrid &msg, bool overlay = false)
  {
    const int ox = lroundf((msg.info.origin.position.x - map->info.origin.position.x) / map->info.linear_resolution);
    const int oy = lroundf((msg.info.origin.position.y - map->info.origin.position.y) / map->info.linear_resolution);
    // Clear travelable area in OVERWRITE mode
    if (overlay_mode_ == OVERWRITE && overlay)
    {
      for (size_t yaw = 0; yaw < map->info.angle; yaw++)
      {
        for (unsigned int i = 0; i < msg.data.size(); i++)
        {
          const auto &val = msg.data[i];
          if (val < 0)
            continue;

          const int x = lroundf((i % msg.info.width) * msg.info.resolution / map->info.linear_resolution);
          if (x < range_max_ || static_cast<int>(msg.info.width) - range_max_ <= x)
            continue;
          const int y = lroundf((i / msg.info.width) * msg.info.resolution / map->info.linear_resolution);
          if (y < range_max_ || static_cast<int>(msg.info.height) - range_max_ <= y)
            continue;

          const int res_up = ceilf(msg.info.resolution / map->info.linear_resolution);
          for (int yp = 0; yp < res_up; yp++)
          {
            for (int xp = 0; xp < res_up; xp++)
            {
              const int x2 = x + ox + xp;
              const int y2 = y + oy + yp;
              if (static_cast<unsigned int>(x2) >= map->info.width ||
                  static_cast<unsigned int>(y2) >= map->info.height)
                continue;

              auto &m = getCostRef(x2, y2, yaw, map);
              m = 0;
            }
          }
        }
      }
    }
    // Get max
    for (size_t yaw = 0; yaw < map->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < msg.data.size(); i++)
      {
        const int gx = lroundf((i % msg.info.width) * msg.info.resolution / map->info.linear_resolution) + ox;
        const int gy = lroundf((i / msg.info.width) * msg.info.resolution / map->info.linear_resolution) + oy;
        if ((unsigned int)gx >= map->info.width ||
            (unsigned int)gy >= map->info.height)
          continue;

        auto val = msg.data[i];
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

            auto &m = getCostRef(x2, y2, yaw, map);
            const auto c = cs_.e(x, y, yaw) * val / 100;
            if (m < c)
              m = c;
          }
        }
      }
    }
  }
  costmap_cspace::CSpace3DUpdate generateUpdateMsg(
      costmap_cspace::CSpace3D::Ptr map, const int &x, const int &y, const int &yaw,
      const int &width, const int &height, const int &angle,
      const ros::Time &stamp)
  {
    costmap_cspace::CSpace3DUpdate update;
    update.header = map->header;
    map->header.stamp = stamp;
    int update_x = x - range_max_;
    int update_y = y - range_max_;
    int update_width = width + range_max_ * 2;
    int update_height = height + range_max_ * 2;
    if (update_x < 0)
    {
      update_width += update_x;
      update_x = 0;
    }
    if (update_y < 0)
    {
      update_height += update_y;
      update_y = 0;
    }
    if (update_x + update_width > static_cast<int>(map->info.width))
    {
      update_width = map->info.width - update_x;
    }
    if (update_y + update_height > static_cast<int>(map->info.height))
    {
      update_height = map->info.height - update_y;
    }
    update.x = update_x;
    update.y = update_y;
    update.width = update_width;
    update.height = update_height;
    update.yaw = yaw;
    update.angle = angle;
    update.data.resize(update.width * update.height * update.angle);

    for (int i = 0; i < static_cast<int>(update.width); i++)
    {
      for (int j = 0; j < static_cast<int>(update.height); j++)
      {
        for (int k = 0; k < static_cast<int>(update.angle); k++)
        {
          const int x2 = update.x + i;
          const int y2 = update.y + j;
          const int yaw2 = yaw + k;

          const auto &m = getCostRef(x2, y2, yaw2, map);
          const size_t addr = (k * update.height + j) * update.width + i;
          assert(addr < update.data.size());
          auto &up = update.data[addr];
          up = m;
        }
      }
    }
    return update;
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_3D_LAYER_BASE_H
