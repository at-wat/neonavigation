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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_BASE_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_BASE_H

#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <algorithm>
#include <map>
#include <string>

namespace costmap_cspace
{
class CSpace3DMsg : public costmap_cspace_msgs::CSpace3D
{
public:
  using Ptr = std::shared_ptr<CSpace3DMsg>;
  using ConstPtr = std::shared_ptr<const CSpace3DMsg>;
  size_t address(const int& x, const int& y, const int& yaw) const
  {
    return (yaw * info.height + y) * info.width + x;
  }
  const int8_t& getCost(const int& x, const int& y, const int& yaw) const
  {
    ROS_ASSERT(static_cast<size_t>(yaw) < info.angle);
    ROS_ASSERT(static_cast<size_t>(x) < info.width);
    ROS_ASSERT(static_cast<size_t>(y) < info.height);

    const size_t addr = address(x, y, yaw);
    ROS_ASSERT(addr < data.size());

    return data[addr];
  }
  int8_t& getCost(const int& x, const int& y, const int& yaw)
  {
    ROS_ASSERT(static_cast<size_t>(yaw) < info.angle);
    ROS_ASSERT(static_cast<size_t>(x) < info.width);
    ROS_ASSERT(static_cast<size_t>(y) < info.height);

    const size_t addr = address(x, y, yaw);
    ROS_ASSERT(addr < data.size());

    return data[addr];
  }
};

enum MapOverlayMode
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
      const int& x, const int& y, const int& yaw,
      const int& width, const int& height, const int& angle,
      const ros::Time& stamp = ros::Time())
    : x_(x)
    , y_(y)
    , yaw_(yaw)
    , width_(width)
    , height_(height)
    , angle_(angle)
    , stamp_(stamp)
  {
  }
  void merge(const UpdatedRegion& region)
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
  void expand(const int& ex)
  {
    ROS_ASSERT(ex >= 0);
    x_ -= ex;
    y_ -= ex;
    width_ += 2 * ex;
    height_ += 2 * ex;
  }
  void normalize()
  {
    const int x2 = x_ + width_;
    const int y2 = y_ + height_;
    if (x_ < 0)
      x_ = 0;
    if (y_ < 0)
      y_ = 0;

    width_ = x2 - x_;
    height_ = y2 - y_;
  }
  void bitblt(const CSpace3DMsg::Ptr& dest, const CSpace3DMsg::ConstPtr& src)
  {
    ROS_ASSERT(dest->info.angle == src->info.angle);
    ROS_ASSERT(dest->info.width == src->info.width);
    ROS_ASSERT(dest->info.height == src->info.height);

    normalize();
    const size_t copy_length =
        std::min<size_t>(width_, src->info.width - x_) *
        sizeof(src->data[0]);
    for (
        size_t a = yaw_;
        static_cast<int>(a) < yaw_ + angle_ && a < src->info.angle;
        ++a)
    {
      auto dest_pos = &dest->data[dest->address(x_, y_, a)];
      auto src_pos = &src->data[src->address(x_, y_, a)];
      const auto dest_stride = dest->info.width * sizeof(dest->data[0]);
      const auto src_stride = src->info.width * sizeof(src->data[0]);
      for (
          size_t y = y_;
          static_cast<int>(y) < y_ + height_ && y < src->info.height;
          ++y)
      {
        memcpy(
            dest_pos, src_pos, copy_length);
        src_pos += src_stride;
        dest_pos += dest_stride;
      }
    }
  }
};

class Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerBase>;

protected:
  int ang_grid_;
  MapOverlayMode overlay_mode_;
  bool root_;

  CSpace3DMsg::Ptr map_;
  CSpace3DMsg::Ptr map_overlay_;

  Costmap3dLayerBase::Ptr child_;
  UpdatedRegion region_;
  UpdatedRegion region_prev_;
  nav_msgs::OccupancyGrid::ConstPtr map_updated_;

public:
  Costmap3dLayerBase()
    : ang_grid_(-1)
    , overlay_mode_(MapOverlayMode::MAX)
    , root_(true)
    , map_(new CSpace3DMsg)
    , map_overlay_(new CSpace3DMsg)
  {
  }

  virtual void loadConfig(XmlRpc::XmlRpcValue config) = 0;
  virtual void setMapMetaData(const costmap_cspace_msgs::MapMetaData3D& info) = 0;

  void setAngleResolution(
      const int ang_resolution)
  {
    ang_grid_ = ang_resolution;
  }
  void setOverlayMode(
      const MapOverlayMode overlay_mode)
  {
    overlay_mode_ = overlay_mode;
  }
  void setChild(Costmap3dLayerBase::Ptr child)
  {
    child_ = child;
    child_->setMap(getMapOverlay());
    child_->root_ = false;
  }
  void setBaseMap(const nav_msgs::OccupancyGrid::ConstPtr& base_map)
  {
    ROS_ASSERT(root_);
    ROS_ASSERT(ang_grid_ > 0);
    ROS_ASSERT(base_map->data.size() >= base_map->info.width * base_map->info.height);

    const size_t xy_size = base_map->info.width * base_map->info.height;
    map_->header = base_map->header;
    map_->info.width = base_map->info.width;
    map_->info.height = base_map->info.height;
    map_->info.angle = ang_grid_;
    map_->info.linear_resolution = base_map->info.resolution;
    map_->info.angular_resolution = 2.0 * M_PI / ang_grid_;
    map_->info.origin = base_map->info.origin;
    map_->data.resize(xy_size * map_->info.angle);

    setMapMetaData(map_->info);

    for (size_t yaw = 0; yaw < map_->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < xy_size; i++)
      {
        map_->data[i + yaw * xy_size] = 0;
      }
    }
    updateCSpace(
        base_map,
        UpdatedRegion(
            0, 0, 0,
            map_->info.width, map_->info.height, map_->info.angle,
            map_->header.stamp));
    for (size_t yaw = 0; yaw < map_->info.angle; yaw++)
    {
      for (unsigned int i = 0; i < xy_size; i++)
      {
        if (base_map->data[i] < 0)
        {
          map_->data[i + yaw * xy_size] = -1;
        }
      }
    }
    *map_overlay_ = *map_;
    if (child_)
      child_->setBaseMapChain();

    updateChainEntry(
        UpdatedRegion(
            0, 0, 0, map_->info.width, map_->info.height, map_->info.angle,
            base_map->header.stamp));
  }
  void processMapOverlay(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    ROS_ASSERT(!root_);
    ROS_ASSERT(ang_grid_ > 0);
    const int ox =
        lroundf((msg->info.origin.position.x - map_->info.origin.position.x) /
                map_->info.linear_resolution);
    const int oy =
        lroundf((msg->info.origin.position.y - map_->info.origin.position.y) /
                map_->info.linear_resolution);

    const int w =
        lroundf(msg->info.width * msg->info.resolution / map_->info.linear_resolution);
    const int h =
        lroundf(msg->info.height * msg->info.resolution / map_->info.linear_resolution);

    map_updated_ = msg;

    region_ = UpdatedRegion(ox, oy, 0, w, h, map_->info.angle, msg->header.stamp);
    updateChainEntry(UpdatedRegion(ox, oy, 0, w, h, map_->info.angle, msg->header.stamp));
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
  int getAngularGrid() const
  {
    return ang_grid_;
  }

protected:
  virtual bool updateChain(const bool output) = 0;
  virtual void updateCSpace(
      const nav_msgs::OccupancyGrid::ConstPtr& map,
      const UpdatedRegion& region) = 0;
  virtual int getRangeMax() const = 0;

  bool updateChainEntry(const UpdatedRegion& region, bool output = true)
  {
    auto region_expand = region;
    region_expand.expand(getRangeMax());

    region_.merge(region_expand);

    auto region_prev_now = region_;
    region_prev_now.merge(region_prev_);
    region_prev_ = region_;

    region_prev_now.bitblt(map_overlay_, map_);
    if (map_updated_)
    {
      if (map_->header.frame_id == map_updated_->header.frame_id)
      {
        updateCSpace(map_updated_, region_);
      }
      else
      {
        ROS_ERROR("map and map_overlay must have same frame_id. skipping");
      }
    }

    if (updateChain(output))
      output = false;

    if (child_)
    {
      return child_->updateChainEntry(region_, output);
    }
    return false;
  }
  void setBaseMapChain()
  {
    setMapMetaData(map_->info);
    *map_overlay_ = *map_;
    if (child_)
      child_->setBaseMapChain();
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_BASE_H
