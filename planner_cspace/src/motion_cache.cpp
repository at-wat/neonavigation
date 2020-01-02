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
#include <unordered_map>
#include <vector>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/motion_cache.h>

namespace planner_cspace
{
namespace planner_3d
{
void MotionCache::reset(
    const float linear_resolution,
    const float angular_resolution,
    const int range,
    const std::function<void(CyclicVecInt<3, 2>, size_t&, size_t&)> gm_addr)
{
  const int angle = std::lround(M_PI * 2 / angular_resolution);

  CyclicVecInt<3, 2> max_range(0, 0, 0);
  page_size_ = angle;
  cache_.resize(angle);
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
                d[2] * angular_resolution
              };
          std::unordered_map<CyclicVecInt<3, 2>, bool, CyclicVecInt<3, 2>> registered;
          registered[d] = true;

          CyclicVecFloat<3, 2> motion(diff_val[0], diff_val[1], diff_val[2]);
          motion.rotate(-syaw * angular_resolution);
          const float cos_v = cosf(motion[2]);
          const float sin_v = sinf(motion[2]);

          const float inter = 1.0 / d.len();

          if (std::abs(sin_v) < 0.1)
          {
            for (float i = 0; i < 1.0; i += inter)
            {
              const float x = diff_val[0] * i;
              const float y = diff_val[1] * i;

              CyclicVecInt<3, 2> pos(
                  x / linear_resolution, y / linear_resolution, yaw / angular_resolution);
              pos.cycleUnsigned(angle);
              if (registered.find(pos) == registered.end())
              {
                page.motion_.push_back(pos);
                for (int i = 0; i < 3; ++i)
                  max_range[i] = std::max(max_range[i], std::abs(pos[i]));
                registered[pos] = true;
              }
            }
            page.distance_ = d.len();
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

          for (float i = 0; i < 1.0; i += inter)
          {
            const float r = r1 * (1.0 - i) + r2 * i;
            const float cx2 = cx_s * (1.0 - i) + cx * i;
            const float cy2 = cy_s * (1.0 - i) + cy * i;
            const float cyaw = yaw + i * dyaw;

            const float posf_raw[3] =
                {
                  (cx2 - r * cosf(cyaw + M_PI / 2)) / linear_resolution,
                  (cy2 - r * sinf(cyaw + M_PI / 2)) / linear_resolution,
                  cyaw / angular_resolution
                };
            const CyclicVecFloat<3, 2> posf(posf_raw[0], posf_raw[1], posf_raw[2]);
            CyclicVecInt<3, 2> pos(posf_raw[0], posf_raw[1], posf_raw[2]);
            pos.cycleUnsigned(angle);
            if (registered.find(pos) == registered.end())
            {
              page.motion_.push_back(pos);
              registered[pos] = true;
            }
            distf += (posf - posf_prev).len();
            posf_prev = posf;
          }
          distf += (CyclicVecFloat<3, 2>(d) - posf_prev).len();
          page.distance_ = distf;
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
}
}  // namespace planner_3d
}  // namespace planner_cspace
