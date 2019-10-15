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

#include <cmath>

#include <planner_cspace/bbf.h>
#include <planner_cspace/blockmem_gridmap.h>
#include <planner_cspace/planner_3d/costmap_bbf.h>

namespace planner_cspace
{
namespace planner_3d
{
void CostmapBBF::updateCostmap()
{
  cm_hist_.clear(0);
  for (VecInternal p(0, 0); p[1] < size_[1]; p[1]++)
  {
    for (p[0] = 0; p[0] < size_[0]; p[0]++)
    {
      cm_hist_[p] = std::lround(cm_hist_bbf_[p].getNormalizedProbability() * 100.0);
    }
  }
}
void CostmapBBF::remember(
    const BlockMemGridmapBase<char, 3, 2>* costmap,
    const Vec& center,
    const float remember_hit_odds, const float remember_miss_odds,
    const int range_min, const int range_max)
{
  const size_t width = size_[0];
  const size_t height = size_[1];
  const int range_min_sq = range_min * range_min;
  const int range_max_sq = range_max * range_max;
  for (VecInternal p(-range_max, 0); p[0] <= range_max; p[0]++)
  {
    for (p[1] = -range_max; p[1] <= range_max; p[1]++)
    {
      const VecInternal gp = VecInternal(center[0], center[1]) + p;
      if (static_cast<size_t>(gp[0]) >= width ||
          static_cast<size_t>(gp[1]) >= height)
        continue;

      const float r_sq = p.sqlen();
      if (r_sq > range_max_sq)
        continue;

      const int c = (*costmap)[Vec(gp[0], gp[1], 0)];

      if (c < 0)
        continue;

      if (c == 100)
      {
        if (r_sq >= range_min_sq)
        {
          cm_hist_bbf_[gp].update(remember_hit_odds);
        }
      }
      else
      {
        cm_hist_bbf_[gp].update(remember_miss_odds);
      }
    }
  }
}
void CostmapBBF::forEach(const std::function<void(const Vec&, bbf::BinaryBayesFilter&)> cb)
{
  for (Vec p(0, 0, 0); p[1] < size_[1]; p[1]++)
  {
    for (p[0] = 0; p[0] < size_[0]; p[0]++)
    {
      cb(p, cm_hist_bbf_[VecInternal(p[0], p[1])]);
    }
  }
}
}  // namespace planner_3d
}  // namespace planner_cspace
