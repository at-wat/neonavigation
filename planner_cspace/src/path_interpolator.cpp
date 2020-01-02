/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <list>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/rotation_cache.h>
#include <planner_cspace/planner_3d/path_interpolator.h>

namespace planner_cspace
{
namespace planner_3d
{
std::list<CyclicVecFloat<3, 2>> PathInterpolator::interpolate(
    const std::list<CyclicVecInt<3, 2>>& path_grid,
    const float interval,
    const int local_range) const
{
  CyclicVecInt<3, 2> p_prev(0, 0, 0);
  bool init = false;

  std::list<CyclicVecFloat<3, 2>> path;

  for (auto p : path_grid)
  {
    p.cycleUnsigned(angle_);

    if (init)
    {
      const CyclicVecInt<3, 2> ds = path_grid.front() - p;
      CyclicVecInt<3, 2> d = p - p_prev;
      d.cycle(angle_);
      const CyclicVecInt<3, 2> d2(d[0] + range_, d[1] + range_, p[2]);

      const float inter = interval / d.len();

      if (d[0] == 0 && d[1] == 0)
      {
        const int yaw_inc = std::copysign(1, d[2]);
        for (int yaw = p_prev[2]; yaw != p_prev[2] + d[2]; yaw += yaw_inc)
        {
          path.push_back(CyclicVecFloat<3, 2>(p[0], p[1], yaw));
        }
      }
      else if (d[2] == 0 || ds.sqlen() > local_range * local_range)
      {
        for (float i = 0; i < 1.0; i += inter)
        {
          const float x2 = p_prev[0] * (1 - i) + p[0] * i;
          const float y2 = p_prev[1] * (1 - i) + p[1] * i;
          const float yaw2 = p_prev[2] + i * d[2];

          path.push_back(CyclicVecFloat<3, 2>(x2, y2, yaw2));
        }
      }
      else
      {
        const auto& radiuses = rot_cache_.getRadiuses(p_prev[2], d2);
        const float r1 = radiuses.first;
        const float r2 = radiuses.second;
        const float yawf = p[2] * M_PI * 2.0 / angle_;
        const float yawf_prev = p_prev[2] * M_PI * 2.0 / angle_;

        const float cx = p[0] + r2 * cosf(yawf + M_PI / 2);
        const float cy = p[1] + r2 * sinf(yawf + M_PI / 2);
        const float cx_prev = p_prev[0] + r1 * cosf(yawf_prev + M_PI / 2);
        const float cy_prev = p_prev[1] + r1 * sinf(yawf_prev + M_PI / 2);

        for (float i = 0; i < 1.0; i += inter)
        {
          const float r = r1 * (1.0 - i) + r2 * i;
          const float cx2 = cx_prev * (1.0 - i) + cx * i;
          const float cy2 = cy_prev * (1.0 - i) + cy * i;
          const float cyaw = p_prev[2] + i * d[2];
          const float cyawf = cyaw * M_PI * 2.0 / angle_;

          const float x2 = cx2 - r * cosf(cyawf + M_PI / 2);
          const float y2 = cy2 - r * sinf(cyawf + M_PI / 2);

          path.push_back(CyclicVecFloat<3, 2>(x2, y2, cyaw));
        }
      }
    }
    p_prev = p;
    init = true;
  }
  path.push_back(CyclicVecFloat<3, 2>(path_grid.back()));
  return path;
}
}  // namespace planner_3d
}  // namespace planner_cspace
