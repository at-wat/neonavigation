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

#ifndef COSTMAP_3D_LAYER_FOOTPRINT_H
#define COSTMAP_3D_LAYER_FOOTPRINT_H

#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace/CSpace3D.h>
#include <costmap_cspace/CSpace3DUpdate.h>

#include <xmlrpcpp/XmlRpcValue.h>

#include <costmap_3d_layer/base.h>
#include <polygon.h>

namespace costmap_cspace
{
class Costmap3dLayerFootprint : public Costmap3dLayerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerFootprint>;

protected:
  float footprint_radius_;
  geometry_msgs::PolygonStamped footprint_;
  Polygon footprint_p_;

public:
  void setFootprint(const Polygon footprint)
  {
    footprint_p_ = footprint;
    footprint_radius_ = footprint.radius();
    footprint_ = footprint.toMsg();
  }
  void generateCSpaceTemplate(const MapMetaData3D &info)
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
    }
  }
  Polygon &getFootprint()
  {
    return footprint_p_;
  }
  const geometry_msgs::PolygonStamped &getFootprintMsg() const
  {
    return footprint_;
  }
  float getFootprintRadius() const
  {
    return footprint_radius_;
  }

protected:
  bool updateChain()
  {
    return false;
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_3D_LAYER_FOOTPRINT_H
