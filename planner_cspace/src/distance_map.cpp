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

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <omp.h>

#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/distance_map.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>

namespace planner_cspace
{
namespace planner_3d
{
void DistanceMap::fillCostmap(
    reservable_priority_queue<Astar::PriorityVec>& open,
    const Astar::Vec& s_rough)
{
  debug_data_.search_queue_size = open.size();
  debug_data_.search_queue_cap = open.capacity();

  std::vector<Astar::PriorityVec> centers;
  centers.reserve(num_cost_estim_task_);

  debug_data_.has_negative_cost = false;

#pragma omp parallel
  {
    std::vector<Astar::GridmapUpdate> updates;
    updates.reserve(num_cost_estim_task_ * search_diffs_.size() / omp_get_num_threads());

    const float range_overshoot = p_.euclid_cost[0] * (p_.range + p_.local_range + p_.longcut_range);
    const float weight_linear = p_.resolution / 100.0;

    while (true)
    {
#pragma omp barrier
#pragma omp single
      {
        centers.clear();
        const float start_cost = g_[s_rough];
        for (size_t i = 0; i < static_cast<size_t>(num_cost_estim_task_);)
        {
          if (open.size() < 1)
            break;
          Astar::PriorityVec center(open.top());
          open.pop();
          if (center.p_raw_ > g_[center.v_])
            continue;
          if (center.p_raw_ - range_overshoot > start_cost)
            continue;
          centers.emplace_back(std::move(center));
          ++i;
        }
      }  // omp single

      if (centers.size() == 0)
        break;
      updates.clear();

#pragma omp for schedule(static)
      for (auto it = centers.cbegin(); it < centers.cend(); ++it)
      {
        const Astar::Vec p = it->v_;

        for (const SearchDiffs& ds : search_diffs_)
        {
          const Astar::Vec d = ds.d;
          const Astar::Vec next = p + d;

          if (static_cast<size_t>(next[0]) >= static_cast<size_t>(p_.size[0]) ||
              static_cast<size_t>(next[1]) >= static_cast<size_t>(p_.size[1]))
          {
            continue;
          }

          float cost = ds.euclid_cost;

          const float gnext = g_[next];

          if (gnext < g_[p] + cost)
          {
            // Skip as this search task has no chance to find better way.
            continue;
          }

          {
            float sum = 0, sum_hist = 0;
            bool collision = false;
            for (const auto& d : ds.pos)
            {
              const Astar::Vec pos = p + d;
              const char c = cm_rough_[pos];
              if (c > 99)
              {
                collision = true;
                break;
              }
              sum += c;
              sum_hist += bbf_costmap_.getCost(pos);
            }
            if (collision)
              continue;
            cost +=
                (weight_linear * ds.grid_to_len) *
                (sum * cc_.weight_costmap_ + sum_hist * cc_.weight_remembered_);

            if (cost < 0)
            {
              cost = 0;
              debug_data_.has_negative_cost = true;
            }
          }

          const float cost_next = it->p_raw_ + cost;
          if (gnext > cost_next)
          {
            updates.emplace_back(p, next, cost_next, cost_next);
          }
        }
      }
#pragma omp barrier
#pragma omp critical
      {
        for (const Astar::GridmapUpdate& u : updates)
        {
          if (g_[u.getPos()] > u.getCost())
          {
            g_[u.getPos()] = u.getCost();
            open.push(std::move(u.getPriorityVec()));
          }
        }
      }  // omp critical
    }
  }  // omp parallel
}

DistanceMap::DistanceMap(
    const BlockMemGridmapBase<char, 3, 2>& cm_rough,
    const CostmapBBF& bbf_costmap)
  : cm_rough_(cm_rough)
  , bbf_costmap_(bbf_costmap)
{
}

void DistanceMap::setParams(const CostCoeff& cc, const int num_cost_estim_task)
{
  cc_ = cc;
  num_cost_estim_task_ = num_cost_estim_task;
}

void DistanceMap::init(const GridAstarModel3D::Ptr model, const Params& p)
{
  if (p.size[0] != p_.size[0] || p.size[1] != p_.size[1])
  {
    // Typical open/erase queue size is be approximated by perimeter of the map
    pq_open_.reserve((p.size[0] + p.size[1]) * 2);
    pq_erase_.reserve((p.size[0] + p.size[1]) * 2);

    g_.reset(Astar::Vec(p.size[0], p.size[1], 1));
  }
  p_ = p;

  const int range_rough = 4;
  for (Astar::Vec d(-range_rough, 0, 0); d[0] <= range_rough; d[0]++)
  {
    for (d[1] = -range_rough; d[1] <= range_rough; d[1]++)
    {
      if (d[0] == 0 && d[1] == 0)
        continue;
      if (d.sqlen() > range_rough * range_rough)
        continue;

      SearchDiffs diffs;

      const float grid_to_len = d.gridToLenFactor();
      const int dist = d.len();
      const float dpx = static_cast<float>(d[0]) / dist;
      const float dpy = static_cast<float>(d[1]) / dist;
      Astar::Vecf pos(0, 0, 0);
      for (int i = 0; i < dist; i++)
      {
        Astar::Vec ipos(pos);
        if (diffs.pos.size() == 0 || diffs.pos.back() != ipos)
        {
          diffs.pos.push_back(std::move(ipos));
        }
        pos[0] += dpx;
        pos[1] += dpy;
      }
      diffs.grid_to_len = grid_to_len;
      diffs.d = d;
      diffs.euclid_cost = model->euclidCostRough(d);
      search_diffs_.push_back(std::move(diffs));
    }
  }
}

void DistanceMap::update(
    const Astar::Vec& s, const Astar::Vec& e,
    const Rect& rect)
{
  Astar::Vec p_cost_min;
  float cost_min = std::numeric_limits<float>::max();
  for (Astar::Vec p(0, rect.min[1], 0); p[1] <= rect.max[1]; p[1]++)
  {
    for (p[0] = rect.min[0]; p[0] <= rect.max[0]; p[0]++)
    {
      if (cost_min > g_[p])
      {
        p_cost_min = p;
        cost_min = g_[p];
      }
    }
  }
  if (cost_min == 0.0)
  {
    // Goal is in the updated area
    create(s, e);
    return;
  }
  pq_open_.clear();
  pq_erase_.clear();

  const Astar::Vec e_rough(e[0], e[1], 0);
  const Astar::Vec s_rough(s[0], s[1], 0);

  if (cost_min != std::numeric_limits<float>::max())
    pq_erase_.emplace(cost_min, cost_min, p_cost_min);

  while (true)
  {
    if (pq_erase_.size() < 1)
      break;
    const Astar::PriorityVec center(pq_erase_.top());
    const Astar::Vec p = center.v_;
    pq_erase_.pop();

    if (g_[p] == std::numeric_limits<float>::max())
      continue;
    g_[p] = std::numeric_limits<float>::max();

    for (Astar::Vec d(-1, 0, 0); d[0] <= 1; d[0]++)
    {
      for (d[1] = -1; d[1] <= 1; d[1]++)
      {
        if (!((d[0] == 0) ^ (d[1] == 0)))
          continue;
        Astar::Vec next = p + d;
        next[2] = 0;
        if ((unsigned int)next[0] >= (unsigned int)p_.size[0] ||
            (unsigned int)next[1] >= (unsigned int)p_.size[1])
          continue;
        const float gn = g_[next];
        if (gn == std::numeric_limits<float>::max())
          continue;
        if (gn < cost_min)
        {
          pq_open_.emplace(gn, gn, next);
          continue;
        }
        pq_erase_.emplace(gn, gn, next);
      }
    }
  }
  if (pq_open_.size() == 0)
  {
    pq_open_.emplace(-p_.euclid_cost[0] * 0.5, -p_.euclid_cost[0] * 0.5, e_rough);
  }

  fillCostmap(pq_open_, s_rough);
}

void DistanceMap::create(const Astar::Vec& s, const Astar::Vec& e)
{
  pq_open_.clear();
  pq_erase_.clear();
  g_.clear(std::numeric_limits<float>::max());

  const Astar::Vec e_rough(e[0], e[1], 0);
  const Astar::Vec s_rough(s[0], s[1], 0);

  g_[e_rough] = -p_.euclid_cost[0] * 0.5;  // Decrement to reduce calculation error
  pq_open_.push(Astar::PriorityVec(g_[e_rough], g_[e_rough], e_rough));
  fillCostmap(pq_open_, s_rough);
  g_[e_rough] = 0;
}

}  // namespace planner_3d
}  // namespace planner_cspace
