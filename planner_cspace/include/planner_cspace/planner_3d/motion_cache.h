/*
 * Copyright (c) 2018, the neonavigation authors
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

#ifndef PLANNER_CSPACE_PLANNER_3D_MOTION_CACHE_H
#define PLANNER_CSPACE_PLANNER_3D_MOTION_CACHE_H

#include <costmap_cspace_msgs/MapMetaData3D.h>

#include <algorithm>
#include <unordered_map>
#include <vector>

#include <planner_cspace/blockmem_gridmap.h>

template <class VEC_INT, class VEC_FLT>
class MotionCache
{
public:
  class Page
  {
  protected:
    friend class MotionCache;

    std::vector<VEC_INT> motion_;
    float distance_;

  public:
    float getDistance()
    {
      return distance_;
    }
    const std::vector<VEC_INT>& getMotion()
    {
      return motion_;
    }
  };
  using Cache =
      std::unordered_map<VEC_INT, Page, VEC_INT>;

  using Ptr = std::shared_ptr<MotionCache>;

protected:
  std::vector<Cache> cache_;

public:
  const typename Cache::iterator find(
      const size_t start_yaw,
      const VEC_INT& goal)
  {
    return cache_[start_yaw].find(goal);
  }
  const typename Cache::iterator end(
      const size_t start_yaw)
  {
    return cache_[start_yaw].end();
  }

  template <class GRIDMAP>
  MotionCache(
      const costmap_cspace_msgs::MapMetaData3D& map_info,
      const int range,
      const GRIDMAP& gm_optimize)
  {
    cache_.resize(map_info.angle);
    for (int syaw = 0; syaw < static_cast<int>(map_info.angle); syaw++)
    {
      const float yaw = syaw * map_info.angular_resolution;
      VEC_INT d;
      for (d[0] = -range; d[0] <= range; d[0]++)
      {
        for (d[1] = -range; d[1] <= range; d[1]++)
        {
          if (d[0] == 0 && d[1] == 0)
            continue;
          if (d.sqlen() > range * range)
            continue;
          for (d[2] = 0; d[2] < static_cast<int>(map_info.angle); d[2]++)
          {
            Page page;
            const float yaw_e = d[2] * map_info.angular_resolution;
            const float diff_val[3] =
                {
                  d[0] * map_info.linear_resolution,
                  d[1] * map_info.linear_resolution,
                  d[2] * map_info.angular_resolution
                };
            std::unordered_map<VEC_INT, bool, VEC_INT> registered;
            registered[d] = true;

            VEC_FLT motion(diff_val);
            motion.rotate(-syaw * map_info.angular_resolution);
            const float cos_v = cosf(motion[2]);
            const float sin_v = sinf(motion[2]);

            const float inter = 1.0 / d.len();

            if (fabs(sin_v) < 0.1)
            {
              for (float i = 0; i < 1.0; i += inter)
              {
                const float x = diff_val[0] * i;
                const float y = diff_val[1] * i;

                const float pos_raw[3] =
                    {
                      roundf(x / map_info.linear_resolution),
                      roundf(y / map_info.linear_resolution),
                      roundf(yaw / map_info.angular_resolution)
                    };
                VEC_INT pos(pos_raw);
                pos.cycleUnsigned(pos[2], map_info.angle);
                if (registered.find(pos) == registered.end())
                {
                  page.motion_.push_back(pos);
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
                sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0)),
                motion[0] * sin_v);

            float dyaw = yaw_e - yaw;
            if (dyaw < -M_PI)
              dyaw += 2 * M_PI;
            else if (dyaw > M_PI)
              dyaw -= 2 * M_PI;

            const float cx = d[0] * map_info.linear_resolution + r2 * cosf(yaw_e + M_PI / 2);
            const float cy = d[1] * map_info.linear_resolution + r2 * sinf(yaw_e + M_PI / 2);
            const float cx_s = r1 * cosf(yaw + M_PI / 2);
            const float cy_s = r1 * sinf(yaw + M_PI / 2);

            // FIXME(at-wat): remove NOLINT after clang-format or roslint supports it
            VEC_FLT posf_prev({ 0, 0, 0 });  // NOLINT(whitespace/braces)

            for (float i = 0; i < 1.0; i += inter)
            {
              const float r = r1 * (1.0 - i) + r2 * i;
              const float cx2 = cx_s * (1.0 - i) + cx * i;
              const float cy2 = cy_s * (1.0 - i) + cy * i;
              const float cyaw = yaw + i * dyaw;

              const float posf_raw[3] =
                  {
                    (cx2 - r * cosf(cyaw + M_PI / 2)) / map_info.linear_resolution,
                    (cy2 - r * sinf(cyaw + M_PI / 2)) / map_info.linear_resolution,
                    cyaw / map_info.angular_resolution
                  };
              const VEC_FLT posf(posf_raw);
              VEC_INT pos(posf_raw);
              pos.cycleUnsigned(pos[2], map_info.angle);
              if (registered.find(pos) == registered.end())
              {
                page.motion_.push_back(pos);
                registered[pos] = true;
              }
              distf += (posf - posf_prev).len();
              posf_prev = posf;
            }
            distf += (VEC_FLT(d) - posf_prev).len();
            page.distance_ = distf;
            cache_[syaw][d] = page;
          }
        }
      }
      // Sort to improve cache hit rate
      for (auto& cache : cache_[syaw])
      {
        auto comp = [this, &gm_optimize](const VEC_INT a, const VEC_INT b)
        {
          size_t a_baddr, a_addr;
          size_t b_baddr, b_addr;
          gm_optimize.block_addr(a, a_baddr, a_addr);
          gm_optimize.block_addr(b, b_baddr, b_addr);
          if (a_baddr == b_baddr)
          {
            return (a_addr < b_addr);
          }
          return (a_baddr < b_baddr);
        };
        std::sort(cache.second.motion_.begin(), cache.second.motion_.end(), comp);
      }
    }
  }
};

#endif  // PLANNER_CSPACE_PLANNER_3D_MOTION_CACHE_H
