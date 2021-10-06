/*
 * Copyright (c) 2014-2021, the neonavigation authors
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

#ifndef PLANNER_CSPACE_PLANNER_3D_DISTANCE_MAP_H
#define PLANNER_CSPACE_PLANNER_3D_DISTANCE_MAP_H

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>
#include <planner_cspace/planner_3d/costmap_bbf.h>

namespace planner_cspace
{
namespace planner_3d
{
class DistanceMap
{
public:
  using Astar = GridAstar<3, 2>;

  struct Rect
  {
    Astar::Vec min;
    Astar::Vec max;
    Rect(const Astar::Vec& min, const Astar::Vec& max)
      : min(min)
      , max(max)  // NOLINT(build/include_what_you_use)
    {
    }
  };

  struct DebugData
  {
    size_t search_queue_size;
    size_t search_queue_cap;
    bool has_negative_cost;
  };

  struct Params
  {
    Astar::Vecf euclid_cost;
    int range;
    int local_range;
    int longcut_range;
    Astar::Vec size;
    float resolution;
  };

  struct SearchDiffs
  {
    Astar::Vec d;
    std::vector<Astar::Vec> pos;
    float grid_to_len;
    float euclid_cost;
  };

  DistanceMap(
      const BlockMemGridmapBase<char, 3, 2>& cm_rough,
      const CostmapBBF& bbf_costmap);

  void setParams(const CostCoeff& cc, const int num_cost_estim_task);

  void init(const GridAstarModel3D::Ptr model, const Params& p);

  void update(
      const Astar::Vec& s, const Astar::Vec& e,
      const Rect& rect);

  void create(const Astar::Vec& s, const Astar::Vec& e);

  inline const Astar::Vec& size() const
  {
    return g_.size();
  }
  inline float& operator[](const Astar::Vec& pos)
  {
    return g_.operator[](pos);
  }
  inline const float operator[](const Astar::Vec& pos) const
  {
    return g_.operator[](pos);
  }
  inline const Astar::Gridmap<float>& gridmap() const
  {
    return g_;
  }
  inline const DebugData& getDebugData() const
  {
    return debug_data_;
  }

protected:
  Astar::Gridmap<float> g_;
  Params p_;
  CostCoeff cc_;
  int num_cost_estim_task_;
  const BlockMemGridmapBase<char, 3, 2>& cm_rough_;
  const CostmapBBF& bbf_costmap_;

  std::vector<SearchDiffs> search_diffs_;
  DebugData debug_data_;

  reservable_priority_queue<Astar::PriorityVec> pq_open_;
  reservable_priority_queue<Astar::PriorityVec> pq_erase_;

  void fillCostmap(
      reservable_priority_queue<Astar::PriorityVec>& open,
      const Astar::Vec& s_rough);
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_DISTANCE_MAP_H
