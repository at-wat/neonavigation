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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_PLAIN_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_PLAIN_H

#include <memory>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_cspace/costmap_3d_layer/base.h>
#include <costmap_cspace/costmap_3d_layer/footprint.h>

namespace costmap_cspace
{
class Costmap3dLayerPlain : public Costmap3dLayerFootprint
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerPlain>;

  Costmap3dLayerPlain()
  {
    Polygon footprint;
    footprint.v.resize(3);
    for (auto& p : footprint.v)
    {
      p[0] = p[1] = 0.0;
    }
    setFootprint(footprint);
  }
  void loadConfig(XmlRpc::XmlRpcValue config)
  {
    setExpansion(
        static_cast<double>(config["linear_expand"]),
        static_cast<double>(config["linear_spread"]));
  }

protected:
  void generateCSpace(
      CSpace3DMsg::Ptr map,
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const UpdatedRegion& region) final
  {
    ROS_ASSERT(ang_grid_ > 0);
    clearTravelableArea(map, msg);
    generateSpecifiedCSpace(map, msg, 0);
    for (size_t i = 1; i < map->info.angle; ++i)
    {
      CSpace3DMsg::copyCells(*map, 0, 0, i, *map, 0, 0, 0, map->info.width * map->info.height);
    }
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_PLAIN_H
