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

#ifndef COSTMAP_3D_H
#define COSTMAP_3D_H

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace/CSpace3D.h>
#include <costmap_cspace/CSpace3DUpdate.h>

#include <XmlRpcValue.h>

#include <cspace3_cache.h>
#include <polygon.h>

namespace XmlRpc
{
bool isNumber(const XmlRpc::XmlRpcValue &value)
{
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
         value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
}
}  // namespace XmlRpc

namespace costmap_cspace
{
class Costmap3DOF
{
public:
  enum map_overlay_mode
  {
    OVERWRITE,
    MAX
  };

protected:
  int ang_resolution_;
  float linear_expand_;
  float linear_spread_;
  int unknown_cost_;
  map_overlay_mode overlay_mode_;

  float footprint_radius_;

  costmap_cspace::CSpace3Cache cs_;
  geometry_msgs::PolygonStamped footprint_;
  costmap_cspace::Polygon footprint_p_;
  int range_max_;

  costmap_cspace::CSpace3D map_;
  costmap_cspace::CSpace3D map_overlay_;
  nav_msgs::OccupancyGrid map_base_;

public:
  Costmap3DOF()
    : ang_resolution_(-1)
    , linear_expand_(0.0)
    , linear_spread_(0.0)
    , unknown_cost_(0.0)
    , overlay_mode_(map_overlay_mode::MAX)
  {
  }
  void setCSpaceConfig(
      const int ang_resolution,
      const float linear_expand,
      const float linear_spread,
      const int unknown_cost,
      const map_overlay_mode overlay_mode)
  {
    ang_resolution_ = ang_resolution;
    linear_expand_ = linear_expand;
    linear_spread_ = linear_spread;
    overlay_mode_ = overlay_mode;
  }
  bool setFootprint(const XmlRpc::XmlRpcValue footprint_xml_const)
  {
    XmlRpc::XmlRpcValue footprint_xml = footprint_xml_const;
    if (footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
    {
      return false;
    }

    footprint_.polygon.points.clear();
    footprint_.header.frame_id = "base_link";
    footprint_radius_ = 0;
    for (int i = 0; i < footprint_xml.size(); i++)
    {
      if (!XmlRpc::isNumber(footprint_xml[i][0]) ||
          !XmlRpc::isNumber(footprint_xml[i][1]))
      {
        return false;
      }

      geometry_msgs::Point32 point;
      costmap_cspace::Vec v;
      v[0] = point.x = static_cast<double>(footprint_xml[i][0]);
      v[1] = point.y = static_cast<double>(footprint_xml[i][1]);
      point.z = 0;
      footprint_.polygon.points.push_back(point);

      footprint_p_.v.push_back(v);

      auto dist = hypotf(point.x, point.y);
      if (dist > footprint_radius_)
        footprint_radius_ = dist;
    }
    footprint_p_.v.push_back(footprint_p_.v.front());
    footprint_.polygon.points.push_back(footprint_.polygon.points[0]);

    return true;
  }
  void generateCSpaceTemplate(const costmap_cspace::MapMetaData3D &info)
  {
    range_max_ =
        ceilf((footprint_radius_ + linear_expand_ + linear_spread_) / info.linear_resolution);
    cs_.reset(range_max_, range_max_, info.angle);

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
          costmap_cspace::Vec p;
          p[0] = 0;
          p[1] = 0;
          if (f.inside(p))
          {
            cs_.e(x, y, yaw) = 100;
          }
          else
          {
            float d = f.dist(p);
            if (d < linear_expand_)
            {
              cs_.e(x, y, yaw) = 100;
            }
            else if (d < linear_expand_ + linear_spread_)
            {
              cs_.e(x, y, yaw) = 100 - (d - linear_expand_) * 100 / linear_spread_;
            }
            else
            {
              cs_.e(x, y, yaw) = 0;
            }
          }
        }
      }
    }
  }
  void setBaseMap(const nav_msgs::OccupancyGrid &map)
  {
    map_base_ = map;
  }
  void processMap()
  {
    map_.header = map_base_.header;
    map_.info.width = map_base_.info.width;
    map_.info.height = map_base_.info.height;
    map_.info.angle = ang_resolution_;
    map_.info.linear_resolution = map_base_.info.resolution;
    map_.info.angular_resolution = 2.0 * M_PI / ang_resolution_;
    map_.info.origin = map_base_.info.origin;
    map_.data.resize(map_base_.data.size() * map_.info.angle);

    generateCSpaceTemplate(map_.info);

    for (size_t yaw = 0; yaw < map_.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map_base_.data.size(); i++)
      {
        map_.data[i + yaw * map_base_.data.size()] = 0;
      }
    }
    mapCopy(map_, map_base_);
    for (size_t yaw = 0; yaw < map_.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map_base_.data.size(); i++)
      {
        if (map_base_.data[i] < 0)
        {
          map_.data[i + yaw * map_base_.data.size()] = -1;
        }
      }
    }
  }
  costmap_cspace::CSpace3DUpdate processMapOverlay(const nav_msgs::OccupancyGrid &msg)
  {
    int ox = lroundf((msg.info.origin.position.x - map_.info.origin.position.x) / map_.info.linear_resolution);
    int oy = lroundf((msg.info.origin.position.y - map_.info.origin.position.y) / map_.info.linear_resolution);
    auto omap = msg;
    if (overlay_mode_ == OVERWRITE)
    {
      for (unsigned int i = 0; i < msg.data.size(); i++)
      {
        if (omap.data[i] < 0)
        {
          int mx = lroundf((i % msg.info.width) * msg.info.resolution / map_.info.linear_resolution);
          if (msg.info.width <= (unsigned int)mx)
            continue;
          int my = lroundf((i / msg.info.width) * msg.info.resolution / map_.info.linear_resolution);
          if (msg.info.height <= (unsigned int)my)
            continue;

          omap.data[i] = map_base_.data[(my + oy) * map_base_.info.width + (mx + ox)];
        }
      }
    }
    map_overlay_ = map_;
    mapCopy(map_overlay_, omap, true);

    int w = lroundf(msg.info.width * msg.info.resolution / map_.info.linear_resolution);
    int h = lroundf(msg.info.height * msg.info.resolution / map_.info.linear_resolution);
    return updateMap(map_overlay_, ox, oy, 0, w, h, map_.info.angle);
  }
  CSpace3Cache &getTemplate()
  {
    return cs_;
  }
  Polygon &getFootprint()
  {
    return footprint_p_;
  }
  int getRangeMax()
  {
    return range_max_;
  }
  float getFootprintRadius()
  {
    return footprint_radius_;
  }

protected:
  void mapCopy(costmap_cspace::CSpace3D &map_, const nav_msgs::OccupancyGrid &msg, bool overlay = false)
  {
    int ox = lroundf((msg.info.origin.position.x - map_.info.origin.position.x) / map_.info.linear_resolution);
    int oy = lroundf((msg.info.origin.position.y - map_.info.origin.position.y) / map_.info.linear_resolution);
    // Clear travelable area in OVERWRITE mode
    if (overlay_mode_ == OVERWRITE && overlay)
    {
      for (size_t yaw = 0; yaw < map_.info.angle; yaw++)
      {
        for (unsigned int i = 0; i < msg.data.size(); i++)
        {
          auto &val = msg.data[i];
          if (val < 0)
            continue;

          int x = lroundf((i % msg.info.width) * msg.info.resolution / map_.info.linear_resolution);
          if (x < range_max_ || static_cast<int>(msg.info.width) - range_max_ <= x)
            continue;
          int y = lroundf((i / msg.info.width) * msg.info.resolution / map_.info.linear_resolution);
          if (y < range_max_ || static_cast<int>(msg.info.height) - range_max_ <= y)
            continue;

          int res_up = ceilf(msg.info.resolution / map_.info.linear_resolution);
          for (int yp = 0; yp < res_up; yp++)
          {
            for (int xp = 0; xp < res_up; xp++)
            {
              auto &m = map_.data[(yaw * map_.info.height + (y + oy + yp)) * map_.info.width + (x + ox + xp)];
              m = 0;
            }
          }
        }
      }
    }
    // Get max
    for (size_t yaw = 0; yaw < map_.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < msg.data.size(); i++)
      {
        int gx = lroundf((i % msg.info.width) * msg.info.resolution / map_.info.linear_resolution) + ox;
        int gy = lroundf((i / msg.info.width) * msg.info.resolution / map_.info.linear_resolution) + oy;
        if ((unsigned int)gx >= map_.info.width ||
            (unsigned int)gy >= map_.info.height)
          continue;

        auto val = msg.data[i];
        if (val < 0)
        {
          if (overlay_mode_ != OVERWRITE && overlay)
            continue;
          bool edge = false;
          int mx = lroundf((i % msg.info.width) * msg.info.resolution / map_.info.linear_resolution);
          if (mx < 1 || static_cast<int>(msg.info.width) - 1 <= mx)
            continue;
          int my = lroundf((i / msg.info.width) * msg.info.resolution / map_.info.linear_resolution);
          if (my < 1 || static_cast<int>(msg.info.height) - 1 <= my)
            continue;

          for (int y = -1; y <= 1; y++)
          {
            for (int x = -1; x <= 1; x++)
            {
              if (x == 0 && y == 0)
                continue;
              if ((unsigned int)(mx + x) >= map_.info.width ||
                  (unsigned int)(my + y) >= map_.info.height)
                continue;
              size_t addr = i + y * map_.info.width + x;
              if (addr >= msg.data.size())
                continue;
              auto v = msg.data[addr];
              if (v >= 0)
              {
                edge = true;
                break;
              }
            }
          }
          if (edge)
          {
            val = unknown_cost_;
          }
        }
        if (val <= 0)
          continue;

        for (int y = -range_max_; y <= range_max_; y++)
        {
          for (int x = -range_max_; x <= range_max_; x++)
          {
            int x2 = gx + x;
            int y2 = gy + y;
            if ((unsigned int)x2 >= map_.info.width ||
                (unsigned int)y2 >= map_.info.height)
              continue;

            auto &m = map_.data[(yaw * map_.info.height + y2) * map_.info.width + x2];
            auto c = cs_.e(x, y, yaw) * val / 100;
            if (m < c)
              m = c;
          }
        }
      }
    }
  }
  costmap_cspace::CSpace3DUpdate updateMap(
      costmap_cspace::CSpace3D &map_, const int &x, const int &y, const int &yaw,
      const int &width, const int &height, const int &angle)
  {
    costmap_cspace::CSpace3DUpdate update;
    update.header = map_.header;
    map_.header.stamp = ros::Time::now();
    update.x = x - range_max_;
    update.y = y - range_max_;
    update.width = width + range_max_ * 2;
    update.height = height + range_max_ * 2;
    if (update.x < 0)
    {
      update.width += update.x;
      update.x = 0;
    }
    if (update.y < 0)
    {
      update.height += update.y;
      update.y = 0;
    }
    if (update.x + update.width > map_.info.width)
    {
      update.width = map_.info.width - update.x;
    }
    if (update.y + update.height > map_.info.height)
    {
      update.height = map_.info.height - update.y;
    }
    update.yaw = yaw;
    update.angle = angle;
    update.data.resize(update.width * update.height * update.angle);

    costmap_cspace::Vec p;
    for (p[0] = 0; p[0] < update.width; p[0]++)
    {
      for (p[1] = 0; p[1] < update.height; p[1]++)
      {
        for (p[2] = 0; p[2] < update.angle; p[2]++)
        {
          int x2 = update.x + p[0];
          int y2 = update.y + p[1];
          int yaw2 = yaw + p[2];

          auto &m = map_.data[(yaw2 * map_.info.height + y2) * map_.info.width + x2];
          auto &up = update.data[(p[2] * update.height + p[1]) * update.width + p[0]];
          up = m;
        }
      }
    }
    return update;
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_3D_H
