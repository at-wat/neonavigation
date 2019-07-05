/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include <boost/chrono.hpp>

#include <planner_cspace/reservable_priority_queue.h>
#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/blockmem_gridmap.h>

#include <omp.h>

template <int DIM = 3, int NONCYCLIC = 2>
class GridAstar
{
public:
  using Vec = CyclicVecInt<DIM, NONCYCLIC>;
  using Vecf = CyclicVecFloat<DIM, NONCYCLIC>;

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

    PriorityVec()
    {
      p_ = 0;
    }
    PriorityVec(const float& p, const float& p_raw, const Vec& v)
    {
      p_ = p;
      p_raw_ = p_raw;
      v_ = v;
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
    g_.clear(FLT_MAX);
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
      const Vec& s, const Vec& e,
      std::list<Vec>& path,
      std::function<float(const Vec&, Vec&, const Vec&, const Vec&)> cb_cost,
      std::function<float(const Vec&, const Vec&)> cb_cost_estim,
      std::function<std::vector<Vec>&(const Vec&, const Vec&, const Vec&)> cb_search,
      std::function<bool(const std::list<Vec>&)> cb_progress,
      const float cost_leave,
      const float progress_interval,
      const bool return_best = false)
  {
    return searchImpl(g_, s, e, path,
                      cb_cost, cb_cost_estim, cb_search, cb_progress,
                      cost_leave, progress_interval, return_best);
  }

protected:
  bool searchImpl(
      Gridmap<float>& g,
      const Vec& st, const Vec& en,
      std::list<Vec>& path,
      std::function<float(const Vec&, Vec&, const Vec&, const Vec&)> cb_cost,
      std::function<float(const Vec&, const Vec&)> cb_cost_estim,
      std::function<std::vector<Vec>&(const Vec&, const Vec&, const Vec&)> cb_search,
      std::function<bool(const std::list<Vec>&)> cb_progress,
      const float cost_leave,
      const float progress_interval,
      const bool return_best = false)
  {
    if (st == en)
    {
      return false;
    }
    Vec s = st;
    Vec e = en;
    for (int i = NONCYCLIC; i < DIM; i++)
    {
      s.cycleUnsigned(g.size());
      e.cycleUnsigned(g.size());
    }
    g.clear(FLT_MAX);
    open_.clear();
    parents_.clear();

    g[s] = 0;
    open_.emplace(cb_cost_estim(s, e), 0, s);

    auto ts = boost::chrono::high_resolution_clock::now();

    Vec better = s;
    int cost_estim_min = cb_cost_estim(s, e);

    std::vector<PriorityVec> centers;
    centers.reserve(search_task_num_);
    std::vector<std::vector<GridmapUpdate>> updates_reserved(num_threads);
    std::vector<std::vector<Vec>> dont_reserved(num_threads);
    for (auto& u : updates_reserved)
      u.reserve(search_task_num_);
    for (auto& d : dont_reserved)
      d.reserve(search_task_num_);

    while (true)
    {
      // Fetch tasks to be paralellized
      if (open_.size() < 1)
      {
        // No fesible path
        if (return_best)
        {
          findPath(s, better, path);
        }
        return false;
      }
      bool found(false);
      centers.clear();
      for (size_t i = 0; i < search_task_num_; ++i)
      {
        if (open_.size() == 0)
          break;
        PriorityVec center = open_.top();
        open_.pop();
        if (center.v_ == e || center.p_ - center.p_raw_ <= cost_leave)
        {
          e = center.v_;
          found = true;
          break;
        }
        centers.push_back(std::move(center));
      }
      if (found)
        break;
      auto tnow = boost::chrono::high_resolution_clock::now();
      if (boost::chrono::duration<float>(tnow - ts).count() >= progress_interval)
      {
        std::list<Vec> path_tmp;
        ts = tnow;
        findPath(s, better, path_tmp);
        cb_progress(path_tmp);
      }

#pragma omp parallel
      {
        const int thread_num = omp_get_thread_num();
        std::vector<GridmapUpdate>& updates = updates_reserved[thread_num];
        std::vector<Vec>& dont = dont_reserved[thread_num];
        updates.clear();
        dont.clear();

#pragma omp for schedule(static)
        for (auto it = centers.cbegin(); it < centers.cend(); ++it)
        {
          const Vec p = it->v_;
          const float c = it->p_raw_;
          const float c_estim = it->p_;
          if (c > g[p])
            continue;

          if (c_estim - c < cost_estim_min)
          {
            cost_estim_min = c_estim - c;
            better = p;
          }

          const std::vector<Vec> search_list = cb_search(p, s, e);

          bool updated(false);
          for (auto it = search_list.cbegin(); it < search_list.cend(); ++it)
          {
            while (1)
            {
              Vec next = p + *it;
              next.cycleUnsigned(g.size());
              if (next.isExceeded(g.size()))
                break;
              if (g[next] < 0)
                break;

              const float cost_estim = cb_cost_estim(next, e);
              if (cost_estim < 0 || cost_estim == FLT_MAX)
                break;

              const float cost = cb_cost(p, next, s, e);
              if (cost < 0 || cost == FLT_MAX)
                break;

              const float cost_next = c + cost;
              if (g[next] > cost_next)
              {
                updated = true;
                updates.emplace_back(p, next, cost_next + cost_estim, cost_next);
              }

              break;
            }
          }
          if (!updated)
            dont.push_back(p);
        }
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
        }  // omp critical
      }    // omp parallel
    }

    return findPath(s, e, path);
  }
  bool findPath(const Vec& s, const Vec& e, std::list<Vec>& path)
  {
    Vec n = e;
    while (true)
    {
      path.push_front(n);
      if (n == s)
        break;
      if (parents_.find(n) == parents_.end())
        return false;

      const Vec child = n;
      n = parents_[child];
      parents_.erase(child);
    }
    return true;
  }

  Gridmap<float> g_;
  std::unordered_map<Vec, Vec, Vec> parents_;
  reservable_priority_queue<PriorityVec> open_;
  size_t queue_size_limit_;
  size_t search_task_num_;
};

#endif  // PLANNER_CSPACE_GRID_ASTAR_H
