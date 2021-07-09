/*
 * Copyright (c) 2014-2020, the neonavigation authors
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

#ifndef PLANNER_CSPACE_GRID_ASTAR_H
#define PLANNER_CSPACE_GRID_ASTAR_H

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/chrono.hpp>

#include <planner_cspace/reservable_priority_queue.h>
#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/blockmem_gridmap.h>
#include <planner_cspace/grid_astar_model.h>

#include <omp.h>

namespace planner_cspace
{
struct SearchStats
{
  size_t num_loop;
  size_t num_search_queue;
  size_t num_prev_updates;
  size_t num_total_updates;
};

template <int DIM = 3, int NONCYCLIC = 2>
class GridAstar
{
public:
  using Vec = CyclicVecInt<DIM, NONCYCLIC>;
  using Vecf = CyclicVecFloat<DIM, NONCYCLIC>;
  using VecWithCost = typename GridAstarModelBase<DIM, NONCYCLIC>::VecWithCost;
  using ProgressCallback = std::function<bool(const std::list<Vec>&, const SearchStats&)>;

  template <class T, int block_width = 0x20>
  class Gridmap : public BlockMemGridmap<T, DIM, NONCYCLIC, block_width>
  {
    using BlockMemGridmap<T, DIM, NONCYCLIC, block_width>::BlockMemGridmap;
  };

  class PriorityVec
  {
  public:
    float p_;
    float p_raw_;
    Vec v_;

    PriorityVec(const float p, const float p_raw, const Vec& v)
      : p_(p)
      , p_raw_(p_raw)
      , v_(v)
    {
    }
    bool operator<(const PriorityVec& b) const
    {
      // smaller first
      return p_ > b.p_;
    }
  };
  class GridmapUpdate
  {
  private:
    const Vec p0_;
    const Vec p1_;
    const float cost_estim_;
    const float cost_;

  public:
    GridmapUpdate(
        const Vec& p0, const Vec& p1,
        const float cost_estim, const float cost)
      : p0_(p0)
      , p1_(p1)
      , cost_estim_(cost_estim)
      , cost_(cost)
    {
    }
    const Vec& getParentPos() const
    {
      return p0_;
    }
    const Vec& getPos() const
    {
      return p1_;
    }
    const float getCost() const
    {
      return cost_;
    }
    const PriorityVec getPriorityVec() const
    {
      return PriorityVec(cost_estim_, cost_, p1_);
    }
  };

public:
  constexpr int getDim() const
  {
    return DIM;
  }
  constexpr int getNoncyclic() const
  {
    return NONCYCLIC;
  }
  void setSearchTaskNum(const size_t& search_task_num)
  {
    search_task_num_ = search_task_num;
  }

  void reset(const Vec size)
  {
    g_.reset(size);
    g_.clear(std::numeric_limits<float>::max());
    parents_.reserve(g_.ser_size() / 16);
    open_.reserve(g_.ser_size() / 16);
  }
  GridAstar()
    : queue_size_limit_(0)
    , search_task_num_(1)
  {
  }
  explicit GridAstar(const Vec size)
  {
    reset(size);
    queue_size_limit_ = 0;
  }
  void setQueueSizeLimit(const size_t size)
  {
    queue_size_limit_ = size;
  }

  bool search(
      const std::vector<VecWithCost>& ss, const Vec& e,
      std::list<Vec>& path,
      const typename GridAstarModelBase<DIM, NONCYCLIC>::Ptr& model,
      ProgressCallback cb_progress,
      const float cost_leave,
      const float progress_interval,
      const bool return_best = false)
  {
    return searchImpl(
        g_, ss, e, path,
        model, cb_progress,
        cost_leave, progress_interval, return_best);
  }

protected:
  bool searchImpl(
      Gridmap<float>& g,
      const std::vector<VecWithCost>& sts, const Vec& en,
      std::list<Vec>& path,
      const typename GridAstarModelBase<DIM, NONCYCLIC>::Ptr& model,
      ProgressCallback cb_progress,
      const float cost_leave,
      const float progress_interval,
      const bool return_best = false)
  {
    if (sts.size() == 0)
      return false;

    auto ts = boost::chrono::high_resolution_clock::now();

    Vec e = en;
    e.cycleUnsigned(g.size());
    g.clear(std::numeric_limits<float>::max());
    open_.clear();
    parents_.clear();

    std::vector<VecWithCost> ss_normalized;
    Vec better;
    int cost_estim_min = std::numeric_limits<int>::max();
    for (const VecWithCost& st : sts)
    {
      if (st.v_ == en)
        return false;

      Vec s = st.v_;
      s.cycleUnsigned(g.size());
      ss_normalized.emplace_back(s, st.c_);
      g[s] = st.c_;

      const int cost_estim = model->costEstim(s, e);
      open_.emplace(cost_estim + st.c_, st.c_, s);
      if (cost_estim_min > cost_estim)
      {
        cost_estim_min = cost_estim;
        better = s;
      }
    }

    std::vector<PriorityVec> centers;
    centers.reserve(search_task_num_);

    size_t num_updates(0);
    size_t num_total_updates(0);
    size_t num_loop(0);

    bool found(false);
    bool abort(false);
#pragma omp parallel
    {
      std::vector<GridmapUpdate> updates;
      // Reserve buffer using example search diff list
      updates.reserve(
          search_task_num_ *
          model->searchGrids(ss_normalized[0].v_, ss_normalized, e).size() /
          omp_get_num_threads());
      std::vector<Vec> dont;
      dont.reserve(search_task_num_);

      while (true)
      {
#pragma omp barrier
#pragma omp single
        {
          const size_t num_search_queue = open_.size();
          num_loop++;

          // Fetch tasks to be paralellized
          centers.clear();
          for (size_t i = 0; i < search_task_num_;)
          {
            if (open_.size() == 0)
              break;
            PriorityVec center(open_.top());
            open_.pop();
            if (center.v_ == e || center.p_ - center.p_raw_ < cost_leave)
            {
              e = center.v_;
              found = true;
              break;
            }
            centers.emplace_back(std::move(center));
            ++i;
          }
          const auto tnow = boost::chrono::high_resolution_clock::now();
          if (boost::chrono::duration<float>(tnow - ts).count() >= progress_interval)
          {
            std::list<Vec> path_tmp;
            ts = tnow;
            findPath(ss_normalized, better, path_tmp);
            const SearchStats stats =
                {
                    .num_loop = num_loop,
                    .num_search_queue = num_search_queue,
                    .num_prev_updates = num_updates,
                    .num_total_updates = num_total_updates,
                };
            if (!cb_progress(path_tmp, stats))
            {
              abort = true;
            }
          }
          num_updates = 0;
        }

        if (centers.size() < 1 || found || abort)
          break;
        updates.clear();
        dont.clear();

#pragma omp for schedule(static)
        for (auto it = centers.cbegin(); it < centers.cend(); ++it)
        {
          const Vec p = it->v_;
          const float c = it->p_raw_;
          const float c_estim = it->p_;
          const float gp = g[p];
          if (c > gp)
            continue;

          if (c_estim - c < cost_estim_min)
          {
            cost_estim_min = c_estim - c;
            better = p;
          }

          const std::vector<Vec> search_list = model->searchGrids(p, ss_normalized, e);

          bool updated(false);
          for (auto it = search_list.cbegin(); it < search_list.cend(); ++it)
          {
            Vec next = p + *it;
            next.cycleUnsigned(g.size());
            if (next.isExceeded(g.size()))
              continue;

            if (g[next] < gp)
            {
              // Skip as this search task has no chance to find better way.
              continue;
            }

            const float cost_estim = model->costEstim(next, e);
            if (cost_estim < 0 || cost_estim == std::numeric_limits<float>::max())
              continue;

            const float cost = model->cost(p, next, ss_normalized, e);
            if (cost < 0 || cost == std::numeric_limits<float>::max())
              continue;

            const float cost_next = c + cost;
            if (g[next] > cost_next)
            {
              updated = true;
              updates.emplace_back(p, next, cost_next + cost_estim, cost_next);
            }
          }
          if (!updated)
            dont.push_back(p);
        }
#pragma omp barrier
#pragma omp critical
        {
          for (const GridmapUpdate& u : updates)
          {
            if (g[u.getPos()] > u.getCost())
            {
              g[u.getPos()] = u.getCost();
              parents_[u.getPos()] = u.getParentPos();
              open_.push(std::move(u.getPriorityVec()));
              if (queue_size_limit_ > 0 && open_.size() > queue_size_limit_)
                open_.pop_back();
            }
          }
          for (const Vec& p : dont)
          {
            g[p] = -1;
          }
          const size_t n = updates.size();
          num_updates += n;
          num_total_updates += n;
        }  // omp critical
      }
    }  // omp parallel

    if (!found)
    {
      // No fesible path
      if (return_best)
      {
        findPath(ss_normalized, better, path);
      }
      return false;
    }
    return findPath(ss_normalized, e, path);
  }
  bool findPath(const std::vector<VecWithCost>& ss, const Vec& e, std::list<Vec>& path) const
  {
    std::unordered_map<Vec, Vec, Vec> parents = parents_;
    Vec n = e;
    while (true)
    {
      path.push_front(n);

      bool found(false);
      for (const VecWithCost& s : ss)
      {
        if (n == s.v_)
        {
          found = true;
          break;
        }
      }
      if (found)
        break;
      if (parents.find(n) == parents.end())
        return false;

      const Vec child = n;
      n = parents[child];
      parents.erase(child);
    }
    return true;
  }

  Gridmap<float> g_;
  std::unordered_map<Vec, Vec, Vec> parents_;
  reservable_priority_queue<PriorityVec> open_;
  size_t queue_size_limit_;
  size_t search_task_num_;
};
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_GRID_ASTAR_H
