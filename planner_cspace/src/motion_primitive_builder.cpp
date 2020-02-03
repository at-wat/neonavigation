/*
 * Copyright (c) 2020, the neonavigation authors
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

#include <planner_cspace/planner_3d/motion_primitive_builder.h>

#include <utility>
#include <vector>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>
#include <planner_cspace/planner_3d/rotation_cache.h>

namespace planner_cspace
{
namespace planner_3d
{
std::vector<std::vector<MotionPrimitiveBuilder::Vec>> MotionPrimitiveBuilder::build(
    const costmap_cspace_msgs::MapMetaData3D& map_info, const CostCoeff& cc, const int range)
{
  RotationCache rot_cache;
  rot_cache.reset(map_info.linear_resolution, map_info.angular_resolution, range);
  Vecf resolution_(1.0f / map_info.linear_resolution,
                   1.0f / map_info.linear_resolution,
                   1.0f / map_info.angular_resolution);

  std::vector<std::vector<Vec>> motion_primitives;
  std::vector<Vec> search_list;
  {
    Vec d;
    for (d[0] = -range; d[0] <= range; d[0]++)
    {
      for (d[1] = -range; d[1] <= range; d[1]++)
      {
        if (d.sqlen() > range * range)
          continue;
        for (d[2] = 0; d[2] < static_cast<int>(map_info.angle); d[2]++)
        {
          search_list.push_back(d);
        }
      }
    }
  }

  motion_primitives.resize(map_info.angle);
  for (int i = 0; i < static_cast<int>(motion_primitives.size()); ++i)
  {
    auto& current_primitives = motion_primitives[i];
    for (const auto& prim : search_list)
    {
      if (prim[0] == 0 && prim[1] == 0)
      {
        if (prim[2] == 0)
        {
          continue;
        }
        // Rotation
        current_primitives.push_back(prim);
        continue;
      }
      int next_angle = i + prim[2];
      if (next_angle >= static_cast<int>(map_info.angle))
      {
        next_angle -= map_info.angle;
      }
      const Vec d2(prim[0] + range, prim[1] + range, next_angle);
      const Vecf motion = rot_cache.getMotion(i, d2);
      const Vecf motion_grid = motion * resolution_;

      if (std::lround(motion_grid[0]) == 0 && std::lround(motion_grid[1]) != 0)
      {
        // Not non-holonomic
        continue;
      }

      static constexpr float EPS = 1.0e-6f;
      if (std::abs(motion[2]) >= 2.0 * M_PI / 4.0 - EPS)
      {
        // Over 90 degree turn
        // must be separated into two curves
        continue;
      }

      if (prim[2] == 0)
      {
        // Straight
        if (std::lround(motion_grid[0]) == 0)
        {
          // side slip
          continue;
        }
        const float aspect = motion[0] / motion[1];
        if (std::abs(aspect) < cc.angle_resolution_aspect_)
        {
          // large y offset
          continue;
        }
        current_primitives.push_back(prim);
      }
      else
      {
        // Curve
        if (std::abs(motion[1]) < map_info.linear_resolution / 2.0)
        {
          // No y direction movement
          continue;
        }
        if (motion[0] * motion[1] * motion[2] < 0)
        {
          continue;
        }
        if (prim.sqlen() < 3 * 3)
        {
          continue;
        }
        const std::pair<float, float>& radiuses = rot_cache.getRadiuses(i, d2);
        const float r1 = radiuses.first;
        const float r2 = radiuses.second;
        if (std::abs(r1 - r2) >= map_info.linear_resolution * 1.5)
        {
          // Drifted
          continue;
        }
        const float curv_radius = (r1 + r2) / 2;
        if (std::abs(curv_radius) < cc.min_curve_radius_)
        {
          continue;
        }
        current_primitives.push_back(prim);
      }
    }
  }

  return motion_primitives;
}
}  // namespace planner_3d
}  // namespace planner_cspace
