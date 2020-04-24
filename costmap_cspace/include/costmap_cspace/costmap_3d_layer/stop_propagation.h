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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_STOP_PROPAGATION_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_STOP_PROPAGATION_H

#include <memory>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <costmap_cspace/costmap_3d_layer/base.h>

namespace costmap_cspace
{
class Costmap3dLayerStopPropagation : public Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerStopPropagation>;

public:
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
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
    region_ = UpdatedRegion(
        0, 0, 0, map_->info.width, map_->info.height, map_->info.angle, ros::Time(0));
    for (auto& c : map_overlay_->data)
      c = -1;
    return false;
  }
  void updateCSpace(
      const nav_msgs::OccupancyGrid::ConstPtr& map,
      const UpdatedRegion& region)
  {
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_STOP_PROPAGATION_H
