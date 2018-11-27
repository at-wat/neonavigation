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

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <costmap_cspace/costmap_3d_layer/base.h>

namespace costmap_cspace
{
class Costmap3dLayerOutput : public Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerOutput>;
  using Callback = boost::function<bool(const CSpace3DMsg::Ptr, const costmap_cspace_msgs::CSpace3DUpdate::Ptr)>;

protected:
  Callback cb_;
  UpdatedRegion region_prev_;

public:
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
  }
  void setHandler(Callback cb)
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
  bool updateChain(const bool output)
  {
    auto update_msg = generateUpdateMsg();
    if (cb_ && output)
      return cb_(map_, update_msg);
    return true;
  }
  void updateCSpace(
      const nav_msgs::OccupancyGrid::ConstPtr& map,
      const UpdatedRegion& region)
  {
  }
  costmap_cspace_msgs::CSpace3DUpdate::Ptr generateUpdateMsg()
  {
    costmap_cspace_msgs::CSpace3DUpdate::Ptr update_msg(new costmap_cspace_msgs::CSpace3DUpdate);
    update_msg->header = map_->header;
    map_->header.stamp = region_.stamp_;
    int update_x = region_.x_;
    int update_y = region_.y_;
    int update_width = region_.width_;
    int update_height = region_.height_;
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
    if (update_x + update_width > static_cast<int>(map_->info.width))
    {
      update_width = map_->info.width - update_x;
    }
    if (update_y + update_height > static_cast<int>(map_->info.height))
    {
      update_height = map_->info.height - update_y;
    }
    UpdatedRegion region(
        update_x,
        update_y,
        region_.yaw_,
        update_width,
        update_height,
        region_.angle_);
    UpdatedRegion region_merged = region;
    region_merged.merge(region_prev_);
    region_prev_ = region;
    region_ = UpdatedRegion();

    update_msg->x = region_merged.x_;
    update_msg->y = region_merged.y_;
    update_msg->width = region_merged.width_;
    update_msg->height = region_merged.height_;
    update_msg->yaw = region_merged.yaw_;
    update_msg->angle = region_merged.angle_;
    update_msg->data.resize(update_msg->width * update_msg->height * update_msg->angle);

    for (int i = 0; i < static_cast<int>(update_msg->width); i++)
    {
      for (int j = 0; j < static_cast<int>(update_msg->height); j++)
      {
        for (int k = 0; k < static_cast<int>(update_msg->angle); k++)
        {
          const int x2 = update_msg->x + i;
          const int y2 = update_msg->y + j;
          const int yaw2 = update_msg->yaw + k;

          const auto& m = map_->getCost(x2, y2, yaw2);
          const size_t addr = (k * update_msg->height + j) * update_msg->width + i;
          ROS_ASSERT(addr < update_msg->data.size());
          auto& up = update_msg->data[addr];
          up = m;
        }
      }
    }
    return update_msg;
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_OUTPUT_H
