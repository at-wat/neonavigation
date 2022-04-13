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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_OUTPUT_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_OUTPUT_H

#include <memory>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_cspace/costmap_3d_layer/base.h>

namespace costmap_cspace
{
template <class CALLBACK>
class Costmap3dLayerOutput : public Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerOutput>;

protected:
  CALLBACK cb_;
  UpdatedRegion region_prev_;

public:
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
  }
  void setHandler(CALLBACK cb)
  {
    cb_ = cb;
  }
  void setMapMetaData(const costmap_cspace_msgs::MapMetaData3D& info)
  {
  }

protected:
  int getRangeMax() const
  {
    return 0;
  }
  void updateCSpace(
      const nav_msgs::OccupancyGrid::ConstPtr& map,
      const UpdatedRegion& region)
  {
  }
};

class Costmap3dStaticLayerOutput
  : public Costmap3dLayerOutput<boost::function<bool(const typename costmap_cspace::CSpace3DMsg::Ptr&)>>
{
public:
  using Ptr = std::shared_ptr<Costmap3dStaticLayerOutput>;

protected:
  bool updateChain(const bool output)
  {
    if (cb_ && output)
      return cb_(map_);
    return true;
  }
};

class Costmap3dUpdateLayerOutput
  : public Costmap3dLayerOutput<boost::function<bool(const typename costmap_cspace::CSpace3DMsg::Ptr&,
                                                     const typename costmap_cspace_msgs::CSpace3DUpdate::Ptr&)>>
{
public:
  using Ptr = std::shared_ptr<Costmap3dUpdateLayerOutput>;

protected:
  bool updateChain(const bool output)
  {
    auto update_msg = generateUpdateMsg();
    if (cb_ && output)
      return cb_(map_, update_msg);
    return true;
  }

  costmap_cspace_msgs::CSpace3DUpdate::Ptr generateUpdateMsg()
  {
    costmap_cspace_msgs::CSpace3DUpdate::Ptr update_msg(new costmap_cspace_msgs::CSpace3DUpdate);
    update_msg->header = map_->header;
    map_->header.stamp = region_.stamp_;

    UpdatedRegion region = region_;
    region.normalize(map_->info.width, map_->info.height);

    UpdatedRegion region_merged = region;
    region_merged.merge(region_prev_);
    region_prev_ = region;
    region_ = UpdatedRegion();

    if (region.width_ == 0 || region.height_ == 0)
    {
      return nullptr;
    }

    update_msg->x = region_merged.x_;
    update_msg->y = region_merged.y_;
    update_msg->width = region_merged.width_;
    update_msg->height = region_merged.height_;
    update_msg->yaw = region_merged.yaw_;
    update_msg->angle = region_merged.angle_;
    update_msg->data.resize(update_msg->width * update_msg->height * update_msg->angle);
    if ((update_msg->x == 0) && (update_msg->y == 0) && (update_msg->yaw == 0) &&
        (update_msg->width == map_->info.width) && (update_msg->height == map_->info.height) &&
        (update_msg->angle == map_->info.angle))
    {
      CSpace3DMsg::copyCells(*update_msg, 0, 0, 0, *map_, 0, 0, 0, update_msg->data.size());
      return update_msg;
    }
    for (unsigned int k = 0; k < update_msg->angle; ++k)
    {
      for (unsigned int j = 0; j < update_msg->height; ++j)
      {
        CSpace3DMsg::copyCells(*update_msg, 0, j, k,
                               *map_, update_msg->x, update_msg->y + j, update_msg->yaw + k, update_msg->width);
      }
    }
    return update_msg;
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_OUTPUT_H
