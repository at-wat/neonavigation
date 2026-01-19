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

#include <cmath>
#include <cstdlib>
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
  if (cc.motion_primitive_type_ == MotionPrimitiveType::BEZIER)
  {
    return buildBezier(map_info, cc, range);
  }
  return buildDefault(map_info, cc, range);
}

std::vector<std::vector<MotionPrimitiveBuilder::Vec>> MotionPrimitiveBuilder::buildDefault(
    const costmap_cspace_msgs::MapMetaData3D& map_info, const CostCoeff& cc, const int range)
{
  RotationCache rot_cache;
  rot_cache.reset(map_info.linear_resolution, map_info.angular_resolution, range);

  // Optional debug hook: set PRIM_DEBUG=1 to log primitives near curvature limits
  const bool debug_curvature = (std::getenv("PRIM_DEBUG") != nullptr);

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
          // No movement
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

std::vector<std::vector<MotionPrimitiveBuilder::Vec>> MotionPrimitiveBuilder::buildBezier(
    const costmap_cspace_msgs::MapMetaData3D& map_info, const CostCoeff& cc, const int range)
{
  RotationCache rot_cache;
  rot_cache.reset(map_info.linear_resolution, map_info.angular_resolution, range);

  // Optional debug hook: set PRIM_DEBUG=1 to log primitives near curvature limits
  const bool debug_curvature = (std::getenv("PRIM_DEBUG") != nullptr);

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
        if (prim[2] != 0)
        {
          // Rotation
          current_primitives.push_back(prim);
        }
        else
        {
          // No movement
        }
        continue;
      }

      int next_angle = i + prim[2];
      if (next_angle >= static_cast<int>(map_info.angle))
      {
        next_angle -= map_info.angle;
      }
      const Vec d2(prim[0] + range, prim[1] + range, next_angle);
      const Vecf motion = rot_cache.getMotion(i, d2);

      const float x0 = 0, y0 = 0;
      const float th0 = 0;  // Relative to start yaw
      const float x3 = motion[0], y3 = motion[1];
      const float th3 = motion[2];  // Relative yaw from RotationCache

      const float dist = std::hypot(x3, y3);
      if (dist < 1e-3)
        continue;

      // Determine if backward (x3 is longitudinal displacement in rotated frame)
      const float sign = (x3 >= 0) ? 1.0f : -1.0f;

      // Helper lambda to compute max curvature for a given cp_dist ratio
      auto computeMaxCurvature = [&](float cp_ratio)
      {
        const float d = sign * cp_ratio * dist;
        const float x1 = x0 + d * std::cos(th0);
        const float y1 = y0 + d * std::sin(th0);
        const float x2 = x3 - d * std::cos(th3);
        const float y2 = y3 - d * std::sin(th3);

        float max_kappa = 0;
        const float dx0 = 3 * (x1 - x0);
        const float dy0 = 3 * (y1 - y0);
        constexpr float EPS_V2 = 1.0e-6f;
        constexpr float EPS_DOT = 1.0e-6f;

        for (float t = 0; t <= 1.001f; t += 0.05f)
        {
          const float cur_t = std::min(t, 1.0f);
          const float mt = 1.0f - cur_t;
          const float mt2 = mt * mt;
          const float t2 = cur_t * cur_t;

          const float dx = 3 * mt2 * (x1 - x0) + 6 * mt * cur_t * (x2 - x1) + 3 * t2 * (x3 - x2);
          const float dy = 3 * mt2 * (y1 - y0) + 6 * mt * cur_t * (y2 - y1) + 3 * t2 * (y3 - y2);
          float v2 = dx * dx + dy * dy;
          if (v2 < EPS_V2)
          {
            // Degenerate velocity; skip this sample instead of exploding curvature
            continue;
          }

          // Reversal check: Tangent should not reverse relative to start tangent
          if (dx * dx0 + dy * dy0 < -EPS_DOT)
            return std::numeric_limits<float>::max();

          const float ddx = 6 * mt * (x2 - 2 * x1 + x0) + 6 * cur_t * (x3 - 2 * x2 + x1);
          const float ddy = 6 * mt * (y2 - 2 * y1 + y0) + 6 * cur_t * (y3 - 2 * y2 + y1);

          const float kappa = std::abs(dx * ddy - dy * ddx) / (v2 * std::sqrt(v2));
          max_kappa = std::max(max_kappa, kappa);
        }
        return max_kappa;
      };

      float optimal_cp_ratio = cc.bezier_cp_dist_;

      if (cc.bezier_cp_mode_ == CostCoeff::BezierCpMode::OPTIMIZE)
      {
        float min_max_curvature = std::numeric_limits<float>::max();
        for (float ratio = 0.1f; ratio <= 0.8f; ratio += 0.05f)
        {
          const float max_kappa = computeMaxCurvature(ratio);
          if (max_kappa < min_max_curvature)
          {
            min_max_curvature = max_kappa;
            optimal_cp_ratio = ratio;
          }
        }
      }

      const float d = sign * optimal_cp_ratio * dist;
      const float x1 = x0 + d * std::cos(th0);
      const float y1 = y0 + d * std::sin(th0);
      const float x2 = x3 - d * std::cos(th3);
      const float y2 = y3 - d * std::sin(th3);

      // Final curvature validation
      const float kappa_limit = 1.0f / cc.min_curve_radius_;
      const float max_kappa = computeMaxCurvature(optimal_cp_ratio);

      const bool near_limit = std::abs(max_kappa - kappa_limit) < 0.05f * kappa_limit;

      if (debug_curvature && (near_limit || max_kappa > kappa_limit))
      {
        std::cerr << "[prim_debug] yaw=" << i
                  << " dxyz=" << prim[0] << "," << prim[1] << "," << prim[2]
                  << " dist=" << dist
                  << " th3=" << th3
                  << " cp_ratio=" << optimal_cp_ratio
                  << " max_kappa=" << max_kappa
                  << " limit=" << kappa_limit
                  << " status=" << ((max_kappa > kappa_limit) ? "reject" : "accept")
                  << std::endl;
      }

      if (max_kappa > kappa_limit)
      {
        continue;
      }

      current_primitives.push_back(prim);
    }
  }

  return motion_primitives;
}
}  // namespace planner_3d
}  // namespace planner_cspace
