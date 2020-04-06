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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_H
#define COSTMAP_CSPACE_COSTMAP_3D_H

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace/costmap_3d_layer/footprint.h>
#include <costmap_cspace/costmap_3d_layer/plain.h>
#include <costmap_cspace/costmap_3d_layer/output.h>
#include <costmap_cspace/costmap_3d_layer/stop_propagation.h>
#include <costmap_cspace/costmap_3d_layer/unknown_handle.h>

#include <costmap_cspace/costmap_3d_layer/class_loader.h>

namespace costmap_cspace
{
class Costmap3d
{
protected:
  std::vector<Costmap3dLayerBase::Ptr> costmaps_;
  int ang_resolution_;

public:
  using Ptr = std::shared_ptr<Costmap3d>;

  explicit Costmap3d(const int ang_resolution)
  {
    ang_resolution_ = ang_resolution;

    ROS_ASSERT(ang_resolution_ > 0);
  }
  template <typename T>
  typename T::Ptr addRootLayer()
  {
    typename T::Ptr
        costmap_base(new T);

    costmap_base->setAngleResolution(ang_resolution_);

    costmap_base->setOverlayMode(MapOverlayMode::MAX);

    costmaps_.resize(1);
    costmaps_[0] = costmap_base;

    return costmap_base;
  }
  template <typename T>
  typename T::Ptr addLayer(
      const MapOverlayMode overlay_mode = MapOverlayMode::MAX)
  {
    typename T::Ptr costmap_overlay(new T);
    costmap_overlay->setAngleResolution(ang_resolution_);
    costmap_overlay->setOverlayMode(overlay_mode);

    costmaps_.back()->setChild(costmap_overlay);
    costmaps_.push_back(costmap_overlay);

    return costmap_overlay;
  }
  Costmap3dLayerBase::Ptr addLayer(
      Costmap3dLayerBase::Ptr costmap_overlay,
      const MapOverlayMode overlay_mode = MapOverlayMode::MAX)
  {
    costmap_overlay->setAngleResolution(ang_resolution_);
    costmap_overlay->setOverlayMode(overlay_mode);

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

#endif  // COSTMAP_CSPACE_COSTMAP_3D_H
