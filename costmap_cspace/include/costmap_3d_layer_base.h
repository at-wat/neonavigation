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

#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace/CSpace3D.h>
#include <costmap_cspace/CSpace3DUpdate.h>

#include <cspace3_cache.h>
#include <polygon.h>

namespace costmap_cspace
{
class CSpace3DMsg : public CSpace3D
{
public:
  using Ptr = std::shared_ptr<CSpace3DMsg>;
  const int8_t &getCost(const int &x, const int &y, const int &yaw) const
  {
    ROS_ASSERT(static_cast<size_t>(yaw) < info.angle);
    ROS_ASSERT(static_cast<size_t>(x) < info.width);
    ROS_ASSERT(static_cast<size_t>(y) < info.height);

    const size_t addr = (yaw * info.height + y) * info.width + x;
    ROS_ASSERT(addr < data.size());

    return data[addr];
  }
  int8_t &getCost(const int &x, const int &y, const int &yaw)
  {
    ROS_ASSERT(static_cast<size_t>(yaw) < info.angle);
    ROS_ASSERT(static_cast<size_t>(x) < info.width);
    ROS_ASSERT(static_cast<size_t>(y) < info.height);

    const size_t addr = (yaw * info.height + y) * info.width + x;
    ROS_ASSERT(addr < data.size());

    return data[addr];
  }
};
class Costmap3dLayerBase
{
public:
  enum map_overlay_mode
  {
    OVERWRITE,
    MAX
  };
  class UpdatedRegion
  {
  public:
    int x_, y_, yaw_;
    int width_, height_, angle_;
    ros::Time stamp_;

    UpdatedRegion()
      : x_(0)
      , y_(0)
      , yaw_(0)
      , width_(0)
      , height_(0)
      , angle_(0)
      , stamp_(0)
    {
    }
    UpdatedRegion(
        const int &x, const int &y, const int &yaw,
        const int &width, const int &height, const int &angle,
        const ros::Time &stamp)
      : x_(x)
      , y_(y)
      , yaw_(yaw)
      , width_(width)
      , height_(height)
      , angle_(angle)
      , stamp_(stamp)
    {
    }
    void merge(const UpdatedRegion &region)
    {
      if (region.width_ == 0 || region.height_ == 0 || region.angle_ == 0)
        return;
      if (region.stamp_ > stamp_)
        stamp_ = region.stamp_;

      if (width_ == 0 || height_ == 0 || angle_ == 0)
      {
        *this = region;
        return;
      }
      int x2 = x_ + width_;
      int y2 = y_ + height_;
      int yaw2 = yaw_ + angle_;

      const int mx2 = region.x_ + region.width_;
      const int my2 = region.y_ + region.height_;
      const int myaw2 = region.yaw_ + region.angle_;

      if (region.x_ < x_)
        x_ = region.x_;
      if (region.y_ < y_)
        y_ = region.y_;
      if (region.yaw_ < yaw_)
        yaw_ = region.yaw_;

      if (x2 < mx2)
        x2 = mx2;
      if (y2 < my2)
        y2 = my2;
      if (yaw2 < myaw2)
        yaw2 = myaw2;

      width_ = x2 - x_;
      height_ = y2 - y_;
      angle_ = yaw2 - yaw_;
    }
  };

  using Ptr = std::shared_ptr<Costmap3dLayerBase>;

protected:
  int ang_grid_;
  float linear_expand_;
  float linear_spread_;
  map_overlay_mode overlay_mode_;
  bool root_;

  CSpace3Cache cs_template_;
  int range_max_;

  CSpace3DMsg::Ptr map_;
  CSpace3DMsg::Ptr map_overlay_;

  Costmap3dLayerBase::Ptr child_;
  UpdatedRegion region_;
  nav_msgs::OccupancyGrid map_updated_;

public:
  Costmap3dLayerBase()
    : ang_grid_(-1)
    , linear_expand_(0.0)
    , linear_spread_(0.0)
    , overlay_mode_(map_overlay_mode::MAX)
    , root_(true)
    , range_max_(0)
    , map_(new CSpace3DMsg)
    , map_overlay_(new CSpace3DMsg)
  {
  }
  void setCSpaceConfig(
      const int ang_resolution,
      const float linear_expand,
      const float linear_spread)
  {
    ang_grid_ = ang_resolution;
    linear_expand_ = linear_expand;
    linear_spread_ = linear_spread;

    ROS_ASSERT(linear_expand >= 0.0);
    ROS_ASSERT(std::isfinite(linear_expand));
    ROS_ASSERT(linear_spread >= 0.0);
    ROS_ASSERT(std::isfinite(linear_spread));
  }
  void setOverlayMode(
      const map_overlay_mode overlay_mode)
  {
    overlay_mode_ = overlay_mode;
  }
  void setFootprint(const Polygon footprint)
  {
  }
  void setChild(Costmap3dLayerBase::Ptr child)
  {
    child_ = child;
    child_->setMap(getMapOverlay());
    child_->root_ = false;
  }
  virtual void generateCSpaceTemplate(const MapMetaData3D &info) = 0;
  void setBaseMap(const nav_msgs::OccupancyGrid &base_map)
  {
    ROS_ASSERT(root_);
    ROS_ASSERT(ang_grid_ > 0);
    ROS_ASSERT(base_map.data.size() >= base_map.info.width * base_map.info.height);

    const size_t xy_size = base_map.info.width * base_map.info.height;
    map_->header = base_map.header;
    map_->info.width = base_map.info.width;
    map_->info.height = base_map.info.height;
    map_->info.angle = ang_grid_;
    map_->info.linear_resolution = base_map.info.resolution;
    map_->info.angular_resolution = 2.0 * M_PI / ang_grid_;
    map_->info.origin = base_map.info.origin;
    map_->data.resize(xy_size * map_->info.angle);

    generateCSpaceTemplate(map_->info);

    for (size_t yaw = 0; yaw < map_->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < xy_size; i++)
      {
        map_->data[i + yaw * xy_size] = 0;
      }
    }
    gemerateCSpace(map_, base_map);
    for (size_t yaw = 0; yaw < map_->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < xy_size; i++)
      {
        if (base_map.data[i] < 0)
        {
          map_->data[i + yaw * xy_size] = -1;
        }
      }
    }
    *map_overlay_ = *map_;
    if (child_)
      child_->setBaseMapChain();
  }
  void processMapOverlay(const nav_msgs::OccupancyGrid &msg)
  {
    ROS_ASSERT(!root_);
    ROS_ASSERT(ang_grid_ > 0);
    const int ox = lroundf((msg.info.origin.position.x - map_->info.origin.position.x) / map_->info.linear_resolution);
    const int oy = lroundf((msg.info.origin.position.y - map_->info.origin.position.y) / map_->info.linear_resolution);

    const int w = lroundf(msg.info.width * msg.info.resolution / map_->info.linear_resolution);
    const int h = lroundf(msg.info.height * msg.info.resolution / map_->info.linear_resolution);

    map_updated_ = msg;

    updateChainEntry(UpdatedRegion(ox, oy, 0, w, h, map_->info.angle, msg.header.stamp));
  }
  CSpace3DMsg::Ptr getMap()
  {
    return map_;
  }
  void setMap(CSpace3DMsg::Ptr map)
  {
    map_ = map;
  }
  CSpace3DMsg::Ptr getMapOverlay()
  {
    return map_overlay_;
  }
  CSpace3Cache &getTemplate()
  {
    return cs_template_;
  }
  int getRangeMax() const
  {
    return range_max_;
  }
  int getAngularGrid() const
  {
    return ang_grid_;
  }

protected:
  virtual bool updateChain() = 0;
  bool updateChainEntry(const UpdatedRegion &region)
  {
    region_.merge(region);

    *map_overlay_ = *map_;
    if (map_updated_.info.width > 0 && map_updated_.info.height > 0)
      gemerateCSpace(map_overlay_, map_updated_, true);

    if (updateChain())
      return true;

    if (child_)
    {
      return child_->updateChainEntry(region_);
    }
    return false;
  }
  void setBaseMapChain()
  {
    generateCSpaceTemplate(map_->info);
    *map_overlay_ = *map_;
    if (child_)
      child_->setBaseMapChain();
  }
  void gemerateCSpace(CSpace3DMsg::Ptr map, const nav_msgs::OccupancyGrid &msg, bool overlay = false)
  {
    ROS_ASSERT(ang_grid_ > 0);
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

              auto &m = map->getCost(x2, y2, yaw);
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

            auto &m = map->getCost(x2, y2, yaw);
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

#endif  // COSTMAP_3D_LAYER_BASE_H
