/*
 * Copyright (c) 2023, the neonavigation authors
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

#ifndef PLANNER_CSPACE_DISTANCE_MAP_UTILS_H
#define PLANNER_CSPACE_DISTANCE_MAP_UTILS_H

#include <limits>
#include <string>

#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/distance_map.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>

namespace planner_cspace
{
namespace planner_3d
{
using Astar = GridAstar<3, 2>;

namespace
{
inline std::string xyStr(const float x, const float y)
{
  return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

inline void debugOutput(
    const DistanceMap& dm,
    const Astar::Gridmap<char, 0x80>& cm_rough,
    const Astar::Vec& s, const Astar::Vec& e)
{
  for (int y = 0; y < cm_rough.size()[1]; y++)
  {
    for (int x = 0; x < cm_rough.size()[0]; x++)
    {
      const Astar::Vec pos(x, y, 0);
      const float d = dm[pos];

      const char type = (pos == s ? 's' : (pos == e ? 'e' : ' '));
      if (d == std::numeric_limits<float>::max())
      {
        fprintf(stderr, "xxx%c ", type);
        continue;
      }
      else if (cm_rough[pos] == 100)
      {
        fprintf(stderr, "***%c ", type);
        continue;
      }
      fprintf(stderr, "%3.1f%c ", d, type);
    }
    fprintf(stderr, "\n");
  }
}
}  // namespace
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_DISTANCE_MAP_UTILS_H
