/*
 * Copyright (c) 2018-2019, the neonavigation authors
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

#include <algorithm>
#include <cmath>
#include <list>
#include <limits>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/motion_cache.h>

#include <chrono>

namespace planner_cspace
{
namespace planner_3d
{
void MotionCache::reset(
    const float linear_resolution,
    const float angular_resolution,
    const int range,
    const std::function<void(CyclicVecInt<3, 2>, size_t&, size_t&)> gm_addr,
    const float interpolation_resolution,
    const float grid_enumeration_resolution,
    const MotionPrimitiveType type,
    const float bezier_cp_dist,
    const int bezier_cp_mode)
{
  const int angle = std::lround(M_PI * 2 / angular_resolution);

  auto normalizeAngle = [](float a)
  {
    while (a > M_PI)
      a -= 2.0f * M_PI;
    while (a < -M_PI)
      a += 2.0f * M_PI;
    return a;
  };

  auto angleDiffIndexAbs = [angle](const int start_yaw, const int goal_yaw)
  {
    int diff = goal_yaw - start_yaw;
    while (diff > angle / 2)
      diff -= angle;
    while (diff < -angle / 2)
      diff += angle;
    return static_cast<float>(std::abs(diff));
  };

  ROS_ERROR("Building motion cache. range=%d, linear_res=%.3f, type=%d, interpolation_res=%.3f, grid_enum_res=%.3f",
            range, linear_resolution, static_cast<int>(type), interpolation_resolution, grid_enumeration_resolution);
  ROS_ERROR("Type of motion primitive: %s, bezier_cp_dist=%.3f", (type == MotionPrimitiveType::BEZIER) ? "BEZIER" : "DEFAULT", bezier_cp_dist);

  const auto start_system_time = std::chrono::system_clock::now();
  CyclicVecInt<3, 2> max_range(0, 0, 0);
  page_size_ = angle;
  cache_.resize(angle);

  // BEZIER page generation is kept as a local helper to keep reset() readable.
  auto buildBezierPage = [&](
                             Page& page,
                             const int start_yaw_index,
                             const CyclicVecInt<3, 2>& goal,
                             const int interpolation_step,
                             std::unordered_map<CyclicVecInt<3, 2>, bool, CyclicVecInt<3, 2>>& registered)
  {
    const float x0 = 0.0f, y0 = 0.0f;
    const float th0 = start_yaw_index * angular_resolution;
    const float x3 = goal[0] * linear_resolution;
    const float y3 = goal[1] * linear_resolution;
    const float th3 = goal[2] * angular_resolution;

    const float dist = std::hypot(x3, y3);
    if (dist < 1e-3f)
      return false;

    // Determine if backward (projection onto start heading)
    const float dot = std::cos(th0) * x3 + std::sin(th0) * y3;
    const float sign = (dot >= 0.0f) ? 1.0f : -1.0f;

    auto computeMaxCurvature = [&](float cp_ratio)
    {
      const float d_cp = sign * cp_ratio * dist;
      const float x1 = x0 + d_cp * std::cos(th0);
      const float y1 = y0 + d_cp * std::sin(th0);
      const float x2 = x3 - d_cp * std::cos(th3);
      const float y2 = y3 - d_cp * std::sin(th3);

      float max_kappa = 0.0f;
      const float dx0 = 3.0f * (x1 - x0);
      const float dy0 = 3.0f * (y1 - y0);
      constexpr float EPS_V2 = 1.0e-6f;
      constexpr float EPS_DOT = 1.0e-6f;

      for (float t = 0.0f; t <= 1.001f; t += 0.05f)
      {
        const float cur_t = std::min(t, 1.0f);
        const float mt = 1.0f - cur_t;
        const float mt2 = mt * mt;
        const float t2 = cur_t * cur_t;

        const float dx = 3 * mt2 * (x1 - x0) + 6 * mt * cur_t * (x2 - x1) + 3 * t2 * (x3 - x2);
        const float dy = 3 * mt2 * (y1 - y0) + 6 * mt * cur_t * (y2 - y1) + 3 * t2 * (y3 - y2);
        const float v2 = dx * dx + dy * dy;
        if (v2 < EPS_V2)
          continue;

        // Reversal check
        if (dx * dx0 + dy * dy0 < -EPS_DOT)
          return std::numeric_limits<float>::max();

        const float ddx = 6 * mt * (x2 - 2 * x1 + x0) + 6 * cur_t * (x3 - 2 * x2 + x1);
        const float ddy = 6 * mt * (y2 - 2 * y1 + y0) + 6 * cur_t * (y3 - 2 * y2 + y1);
        const float kappa = std::abs(dx * ddy - dy * ddx) / (v2 * std::sqrt(v2));
        max_kappa = std::max(max_kappa, kappa);
      }
      return max_kappa;
    };

    float cp_ratio = bezier_cp_dist;
    if (bezier_cp_mode != 0)
    {
      float min_max_curvature = std::numeric_limits<float>::max();
      for (float ratio = 0.1f; ratio <= 0.8f; ratio += 0.05f)
      {
        const float max_kappa = computeMaxCurvature(ratio);
        if (max_kappa < min_max_curvature)
        {
          min_max_curvature = max_kappa;
          cp_ratio = ratio;
        }
      }
    }

    const float d_cp = sign * cp_ratio * dist;
    const float x1 = x0 + d_cp * std::cos(th0);
    const float y1 = y0 + d_cp * std::sin(th0);
    const float x2 = x3 - d_cp * std::cos(th3);
    const float y2 = y3 - d_cp * std::sin(th3);

    // 1. Approximate total arc length and build LUT
    const int lut_size = 100;
    std::vector<float> lut_t(lut_size + 1);
    std::vector<float> lut_s(lut_size + 1);
    float total_length = 0.0f;
    float px = x0, py = y0;
    for (int i = 0; i <= lut_size; ++i)
    {
      const float t = i / static_cast<float>(lut_size);
      const float mt = 1.0f - t;
      const float x = mt * mt * mt * x0 + 3 * mt * mt * t * x1 + 3 * mt * t * t * x2 + t * t * t * x3;
      const float y = mt * mt * mt * y0 + 3 * mt * mt * t * y1 + 3 * mt * t * t * y2 + t * t * t * y3;
      if (i > 0)
        total_length += std::hypot(x - px, y - py);
      lut_t[i] = t;
      lut_s[i] = total_length;
      px = x;
      py = y;
    }

    // 2. Sample uniformly based on arc length
    const int bezier_total_step = std::max(1, static_cast<int>(std::round(total_length / grid_enumeration_resolution)));
    CyclicVecFloat<3, 2> posf_prev(0, 0, 0);
    float distf = 0.0f;

    float angle_travel_rad = 0.0f;
    bool has_prev_yaw = false;
    float cyaw_prev = 0.0f;

    for (int step = 0; step < bezier_total_step; ++step)
    {
      const float s = total_length * step / static_cast<float>(bezier_total_step);
      auto it = std::lower_bound(lut_s.begin(), lut_s.end(), s);
      float t = 1.0f;
      if (it != lut_s.end())
      {
        const int idx = std::distance(lut_s.begin(), it);
        if (idx == 0)
          t = 0.0f;
        else
        {
          const float s0 = lut_s[idx - 1];
          const float s1 = lut_s[idx];
          const float t0 = lut_t[idx - 1];
          const float t1 = lut_t[idx];
          t = t0 + (t1 - t0) * (s - s0) / (s1 - s0);
        }
      }

      const float mt = 1.0f - t;
      const float x = mt * mt * mt * x0 + 3 * mt * mt * t * x1 + 3 * mt * t * t * x2 + t * t * t * x3;
      const float y = mt * mt * mt * y0 + 3 * mt * mt * t * y1 + 3 * mt * t * t * y2 + t * t * t * y3;

      const float mt2 = mt * mt;
      const float t2 = t * t;
      const float dx = 3 * mt2 * (x1 - x0) + 6 * mt * t * (x2 - x1) + 3 * t2 * (x3 - x2);
      const float dy = 3 * mt2 * (y1 - y0) + 6 * mt * t * (y2 - y1) + 3 * t2 * (y3 - y2);
      float cyaw = std::atan2(dy, dx);
      if (sign < 0)
        cyaw += M_PI;

      if (has_prev_yaw)
        angle_travel_rad += std::abs(normalizeAngle(cyaw - cyaw_prev));
      cyaw_prev = cyaw;
      has_prev_yaw = true;

      const CyclicVecFloat<3, 2> posf(x / linear_resolution, y / linear_resolution, cyaw / angular_resolution);
      CyclicVecInt<3, 2> pos(static_cast<int>(std::lround(posf[0])), static_cast<int>(std::lround(posf[1])), static_cast<int>(std::lround(posf[2])));
      pos.cycleUnsigned(angle);

      if ((pos != goal) && (registered.find(pos) == registered.end()))
      {
        page.motion_.push_back(pos);
        for (int i = 0; i < 3; ++i)
          max_range[i] = std::max(max_range[i], std::abs(pos[i]));
        registered[pos] = true;
      }
      if (step % interpolation_step == 0)
      {
        page.interpolated_motion_.push_back(posf);
      }
      if (step > 0)
        distf += (posf - posf_prev).len();
      posf_prev = posf;
    }

    // Include final yaw at t=1 to account for end behavior
    {
      const float dx_end = 3.0f * (x3 - x2);
      const float dy_end = 3.0f * (y3 - y2);
      float cyaw_end = std::atan2(dy_end, dx_end);
      if (sign < 0)
        cyaw_end += M_PI;

      if (has_prev_yaw)
        angle_travel_rad += std::abs(normalizeAngle(cyaw_end - cyaw_prev));
    }

    distf += (CyclicVecFloat<3, 2>(goal) - posf_prev).len();
    page.distance_ = distf;
    page.angle_travel_ = angle_travel_rad / angular_resolution;
    return true;
  };

  for (int syaw = 0; syaw < angle; syaw++)
  {
    const float yaw = syaw * angular_resolution;
    CyclicVecInt<3, 2> d;
    for (d[0] = -range; d[0] <= range; d[0]++)
    {
      for (d[1] = -range; d[1] <= range; d[1]++)
      {
        if (d[0] == 0 && d[1] == 0)
          continue;
        if (d.sqlen() > range * range)
          continue;
        for (d[2] = 0; d[2] < angle; d[2]++)
        {
          Page page;
          const float yaw_e = d[2] * angular_resolution;
          const float diff_val[3] =
              {
                  d[0] * linear_resolution,
                  d[1] * linear_resolution,
                  d[2] * angular_resolution,
              };
          std::unordered_map<CyclicVecInt<3, 2>, bool, CyclicVecInt<3, 2>> registered;
          registered[d] = true;

          const int total_step = static_cast<int>(std::round(d.len() / (grid_enumeration_resolution / linear_resolution)));
          const int interpolation_step =
              std::max(1, static_cast<int>(std::round(interpolation_resolution / grid_enumeration_resolution)));

          if (type == MotionPrimitiveType::BEZIER)
          {
            if (!buildBezierPage(page, syaw, d, interpolation_step, registered))
              continue;
            cache_[syaw][d] = page;
            continue;
          }

          CyclicVecFloat<3, 2> motion(diff_val[0], diff_val[1], diff_val[2]);
          motion.rotate(-syaw * angular_resolution);
          const float cos_v = cosf(motion[2]);
          const float sin_v = sinf(motion[2]);

          if (std::abs(sin_v) < 0.1)
          {
            for (int step = 0; step < total_step; ++step)
            {
              const float ratio = step / static_cast<float>(total_step);
              const float x = diff_val[0] * ratio;
              const float y = diff_val[1] * ratio;

              const CyclicVecFloat<3, 2> posf(x / linear_resolution, y / linear_resolution, yaw / angular_resolution);
              CyclicVecInt<3, 2> pos(static_cast<int>(std::lround(posf[0])), static_cast<int>(std::lround(posf[1])), static_cast<int>(std::lround(posf[2])));
              pos.cycleUnsigned(angle);

              if ((pos != d) && (registered.find(pos) == registered.end()))
              {
                page.motion_.push_back(pos);
                for (int i = 0; i < 3; ++i)
                  max_range[i] = std::max(max_range[i], std::abs(pos[i]));
                registered[pos] = true;
              }
              if (step % interpolation_step == 0)
              {
                page.interpolated_motion_.push_back(posf);
              }
            }
            page.distance_ = d.len();
            page.angle_travel_ = angleDiffIndexAbs(syaw, d[2]);
            cache_[syaw][d] = page;
            continue;
          }

          float distf = 0.0;
          const float r1 = motion[1] + motion[0] * cos_v / sin_v;
          const float r2 = std::copysign(
              std::sqrt(std::pow(motion[0], 2) + std::pow(motion[0] * cos_v / sin_v, 2)),
              motion[0] * sin_v);

          float dyaw = yaw_e - yaw;
          if (dyaw < -M_PI)
            dyaw += 2 * M_PI;
          else if (dyaw > M_PI)
            dyaw -= 2 * M_PI;

          const float cx = d[0] * linear_resolution + r2 * cosf(yaw_e + M_PI / 2);
          const float cy = d[1] * linear_resolution + r2 * sinf(yaw_e + M_PI / 2);
          const float cx_s = r1 * cosf(yaw + M_PI / 2);
          const float cy_s = r1 * sinf(yaw + M_PI / 2);

          CyclicVecFloat<3, 2> posf_prev(0, 0, 0);

          for (int step = 0; step < total_step; ++step)
          {
            const float ratio = step / static_cast<float>(total_step);
            const float r = r1 * (1.0 - ratio) + r2 * ratio;
            const float cx2 = cx_s * (1.0 - ratio) + cx * ratio;
            const float cy2 = cy_s * (1.0 - ratio) + cy * ratio;
            const float cyaw = yaw + ratio * dyaw;

            const CyclicVecFloat<3, 2> posf((cx2 - r * cosf(cyaw + M_PI / 2)) / linear_resolution,
                                            (cy2 - r * sinf(cyaw + M_PI / 2)) / linear_resolution,
                                            cyaw / angular_resolution);
            CyclicVecInt<3, 2> pos(static_cast<int>(std::lround(posf[0])), static_cast<int>(std::lround(posf[1])), static_cast<int>(std::lround(posf[2])));
            pos.cycleUnsigned(angle);
            if ((pos != d) && (registered.find(pos) == registered.end()))
            {
              page.motion_.push_back(pos);
              registered[pos] = true;
            }
            if (step % interpolation_step == 0)
            {
              page.interpolated_motion_.push_back(posf);
            }
            if (step > 0)
              distf += (posf - posf_prev).len();
            posf_prev = posf;
          }
          distf += (CyclicVecFloat<3, 2>(d) - posf_prev).len();
          page.distance_ = distf;
          page.angle_travel_ = angleDiffIndexAbs(syaw, d[2]);
          cache_[syaw][d] = page;
        }
      }
    }
    // Sort to improve cache hit rate
    for (auto& cache : cache_[syaw])
    {
      auto comp = [this, &gm_addr](const CyclicVecInt<3, 2> a, const CyclicVecInt<3, 2> b)
      {
        size_t a_baddr, a_addr;
        size_t b_baddr, b_addr;
        gm_addr(a, a_baddr, a_addr);
        gm_addr(b, b_baddr, b_addr);
        if (a_baddr == b_baddr)
        {
          return (a_addr < b_addr);
        }
        return (a_baddr < b_baddr);
      };
      std::sort(cache.second.motion_.begin(), cache.second.motion_.end(), comp);
    }
  }
  max_range_ = max_range;

  const auto end_system_time = std::chrono::system_clock::now();
  const double elapsed_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end_system_time - start_system_time).count();
  ROS_ERROR("Motion cache built. time taken: %.3f sec", elapsed_sec);
}

std::list<CyclicVecFloat<3, 2>> MotionCache::interpolatePath(const std::list<CyclicVecInt<3, 2>>& grid_path) const
{
  std::list<CyclicVecFloat<3, 2>> result;
  if (grid_path.size() < 2)
  {
    for (const auto& p : grid_path)
    {
      result.push_back(CyclicVecFloat<3, 2>(p[0], p[1], p[2]));
    }
    return result;
  }
  auto it_prev = grid_path.begin();
  for (auto it = std::next(it_prev); it != grid_path.end(); ++it, ++it_prev)
  {
    if (((*it_prev)[0] == (*it)[0]) && ((*it_prev)[1] == (*it)[1]))
    {
      result.push_back(CyclicVecFloat<3, 2>((*it_prev)[0], (*it_prev)[1], (*it_prev)[2]));
      continue;
    }

    const auto motion_it = find(*it_prev, *it);
    if (motion_it == end((*it)[2]))
    {
      ROS_ERROR("Failed to find motion between [%d %d %d] and [%d %d %d]",
                (*it_prev)[0], (*it_prev)[1], (*it_prev)[2], (*it)[0], (*it)[1], (*it)[2]);
      result.push_back(CyclicVecFloat<3, 2>((*it_prev)[0], (*it_prev)[1], (*it_prev)[2]));
    }
    else
    {
      for (const auto& pos_diff : motion_it->second.getInterpolatedMotion())
      {
        result.push_back(CyclicVecFloat<3, 2>((*it_prev)[0] + pos_diff[0], (*it_prev)[1] + pos_diff[1], pos_diff[2]));
      }
    }
  }
  result.push_back(CyclicVecFloat<3, 2>(grid_path.back()[0], grid_path.back()[1], grid_path.back()[2]));
  return result;
}

}  // namespace planner_3d
}  // namespace planner_cspace
