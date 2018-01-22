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

#ifndef COSTMAP_3D_H
#define COSTMAP_3D_H

#include <costmap_3d_layer_footprint.h>
#include <costmap_3d_layer_plain.h>
#include <costmap_3d_layer_output.h>

#include <vector>

namespace costmap_cspace
{
class Costmap3d
{
protected:
  std::vector<Costmap3dLayerBase::Ptr> costmaps_;
  int ang_resolution_;
  float linear_expand_;
  float linear_spread_;
  Polygon footprint_;

public:
  using Ptr = std::shared_ptr<Costmap3d>;

  Costmap3d(
      const int ang_resolution,
      const float linear_expand,
      const float linear_spread,
      const Polygon footprint = Polygon())
  {
    ang_resolution_ = ang_resolution;
    linear_expand_ = linear_expand;
    ang_resolution_ = ang_resolution;
    footprint_ = footprint;
  }
  template <typename T>
  typename T::Ptr addRootLayer()
  {
    typename T::Ptr
        costmap_base(new T);

    costmap_base->setCSpaceConfig(
        ang_resolution_,
        linear_expand_,
        linear_spread_);

    costmap_base->setOverlayMode(
        Costmap3dLayerBase::map_overlay_mode::MAX);

    costmap_base->setFootprint(footprint_);
    costmaps_.resize(1);
    costmaps_[0] = costmap_base;

    return costmap_base;
  }
  template <typename T>
  typename T::Ptr addLayer(
      const Costmap3dLayerBase::map_overlay_mode overlay_mode =
          Costmap3dLayerBase::map_overlay_mode::MAX)
  {
    typename T::Ptr costmap_overlay(new T);
    costmap_overlay->setCSpaceConfig(
        ang_resolution_,
        linear_expand_,
        linear_spread_);
    costmap_overlay->setOverlayMode(overlay_mode);

    costmap_overlay->setFootprint(footprint_);

    costmaps_.back()->setChild(costmap_overlay);
    costmaps_.push_back(costmap_overlay);

    return costmap_overlay;
  }
  Costmap3dLayerBase::Ptr getRootLayer()
  {
    return costmaps_.front();
  }
};
}  // namespace costmap_cspace

#endif  // COSTMAP_3D_H
