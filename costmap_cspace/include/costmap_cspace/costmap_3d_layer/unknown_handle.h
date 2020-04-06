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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_UNKNOWN_HANDLE_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_UNKNOWN_HANDLE_H

#include <memory>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <costmap_cspace/costmap_3d_layer/base.h>

namespace costmap_cspace
{
class Costmap3dLayerUnknownHandle : public Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerUnknownHandle>;

protected:
  int8_t unknown_cost_;

public:
  Costmap3dLayerUnknownHandle()
    : unknown_cost_(-1)
  {
  }
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
    if (config.hasMember("unknown_cost"))
    {
      unknown_cost_ = static_cast<int>(config["unknown_cost"]);
    }
  }
  void setMapMetaData(const costmap_cspace_msgs::MapMetaData3D& info)
  {
  }

protected:
  int getRangeMax() const
  {
    return 0;
  }
  bool updateChain(const bool output)
  {
    for (
        size_t a = region_.yaw_;
        static_cast<int>(a) < region_.yaw_ + region_.angle_ && a < map_->info.angle;
        ++a)
    {
      for (
          size_t y = region_.y_;
          static_cast<int>(y) < region_.y_ + region_.height_ && y < map_->info.height;
          ++y)
      {
        for (
            size_t x = region_.x_;
            static_cast<int>(x) < region_.x_ + region_.width_ && x < map_->info.width;
            ++x)
        {
          auto& m = map_overlay_->getCost(x, y, a);
          if (m < 0)
            m = unknown_cost_;
        }
      }
    }
    return false;
  }
  void updateCSpace(
      const nav_msgs::OccupancyGrid::ConstPtr& map,
      const UpdatedRegion& region)
  {
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_UNKNOWN_HANDLE_H
