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

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/rotation_cache.h>

namespace planner_cspace
{
namespace planner_3d
{
void RotationCache::Page::reset(const CyclicVecInt<3, 2>& size)
{
  size_t ser_size = 1;
  for (int i = 0; i < 3; i++)
    ser_size *= size[i];

  size_ = size;
  ser_size_ = ser_size;

  c_.reset(new CyclicVecFloat<3, 2>[ser_size]);
  r_.reset(new std::pair<float, float>[ser_size]);
}

void RotationCache::reset(
    const float linear_resolution,
    const float angular_resolution,
    const int range)
{
  const int angle = std::lround(M_PI * 2 / angular_resolution);

  pages_.resize(angle);
  for (int i = 0; i < angle; i++)
  {
    Page& r = pages_[i];
    r.reset(CyclicVecInt<3, 2>(range * 2 + 1, range * 2 + 1, angle));

    CyclicVecInt<3, 2> d;

    for (d[0] = 0; d[0] <= range * 2; d[0]++)
    {
      for (d[1] = 0; d[1] <= range * 2; d[1]++)
      {
        for (d[2] = 0; d[2] < angle; d[2]++)
        {
          auto v = CyclicVecFloat<3, 2>(
              (d[0] - range) * linear_resolution,
              (d[1] - range) * linear_resolution,
              d[2] * angular_resolution);
          v.rotate(-i * angular_resolution);
          r.motion(d) = v;

          const float sin_v = std::sin(v[2]);
          const float cos_v = std::cos(v[2]);
          const float r1 = v[1] + v[0] * cos_v / sin_v;
          const float r2 = std::copysign(
              std::sqrt(std::pow(v[0], 2) + std::pow(v[0] * cos_v / sin_v, 2)),
              v[0] * sin_v);
          r.radiuses(d) = std::pair<float, float>(r1, r2);
        }
      }
    }
  }
}
}  // namespace planner_3d
}  // namespace planner_cspace
