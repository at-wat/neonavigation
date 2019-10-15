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

#ifndef PLANNER_CSPACE_PLANNER_3D_COSTMAP_BBF_H
#define PLANNER_CSPACE_PLANNER_3D_COSTMAP_BBF_H

#include <functional>

#include <planner_cspace/bbf.h>
#include <planner_cspace/blockmem_gridmap.h>

namespace planner_cspace
{
namespace planner_3d
{
class CostmapBBF
{
public:
  using Vec = CyclicVecInt<3, 2>;

private:
  using VecInternal = CyclicVecInt<2, 2>;
  BlockMemGridmap<bbf::BinaryBayesFilter, 2, 2, 0x20> cm_hist_bbf_;
  BlockMemGridmap<char, 2, 2, 0x80> cm_hist_;
  Vec size_;

public:
  inline CostmapBBF()
    : size_(0, 0, 0)
  {
  }
  inline void reset(const Vec& size)
  {
    size_ = size;
    cm_hist_bbf_.reset(VecInternal(size[0], size[1]));
    cm_hist_.reset(VecInternal(size[0], size[1]));
  }
  inline void clear()
  {
    cm_hist_bbf_.clear(bbf::BinaryBayesFilter(bbf::MIN_ODDS));
    cm_hist_.clear(0);
  }
  inline char getCost(const Vec& p) const
  {
    return cm_hist_[VecInternal(p[0], p[1])];
  }

  void remember(
      const BlockMemGridmapBase<char, 3, 2>* const costmap,
      const Vec& center,
      const float remember_hit_odds, const float remember_miss_odds,
      const int range_min, const int range_max);
  void updateCostmap();
  void forEach(const std::function<void(const Vec&, bbf::BinaryBayesFilter&)> cb);
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_COSTMAP_BBF_H
