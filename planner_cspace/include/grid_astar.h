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

#ifndef GRID_ASTAR_H
#define GRID_ASTAR_H

#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include <boost/chrono.hpp>

#include <reservable_priority_queue.h>
#include <cyclic_vec.h>
#include <blockmem_gridmap.h>

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
    PriorityVec(const float &p, const float &p_raw, const Vec &v)
    {
      this->p_ = p;
      this->p_raw_ = p_raw;
      this->v_ = v;
    }
    bool operator<(const PriorityVec &b) const
    {
      // smaller first
      return this->p_ > b.p_;
    }
  };

protected:
  Gridmap<float> g_;
  std::unordered_map<Vec, Vec, Vec> parents_;
  reservable_priority_queue<PriorityVec> open_;
  size_t queue_size_limit_;

public:
  const int getDim()
  {
    return DIM;
  }
  const int getNoncyclic()
  {
    return NONCYCLIC;
  }

  void reset(const Vec size)
  {
    g_.reset(size);
    g_.clear(FLT_MAX);
    parents_.reserve(g_.ser_size / 16);
    open_.reserve(g_.ser_size / 16);
  }
  GridAstar()
  {
    queue_size_limit_ = 0;
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

  bool search(const Vec &s, const Vec &e,
              std::list<Vec> &path,
              std::function<float(const Vec &, Vec &, const Vec &, const Vec &)> cb_cost,
              std::function<float(const Vec &, const Vec &)> cb_cost_estim,
              std::function<std::vector<Vec> &(const Vec &, const Vec &, const Vec &)> cb_search,
              std::function<bool(const std::list<Vec> &)> cb_progress,
              const float cost_leave,
              const float progress_interval,
              const bool return_best = false)
  {
    return searchImpl(g_, s, e, path,
                      cb_cost, cb_cost_estim, cb_search, cb_progress,
                      cost_leave, progress_interval, return_best);
  }
  bool searchImpl(Gridmap<float> &g_,
                  const Vec &st, const Vec &en,
                  std::list<Vec> &path,
                  std::function<float(const Vec &, Vec &, const Vec &, const Vec &)> cb_cost,
                  std::function<float(const Vec &, const Vec &)> cb_cost_estim,
                  std::function<std::vector<Vec> &(const Vec &, const Vec &, const Vec &)> cb_search,
                  std::function<bool(const std::list<Vec> &)> cb_progress,
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
      s.cycle_unsigned(s[i], g_.size[i]);
      e.cycle_unsigned(e[i], g_.size[i]);
    }
    g_.clear(FLT_MAX);
    open_.clear();
    parents_.clear();

    g_[s] = 0;
    open_.push(PriorityVec(cb_cost_estim(s, e), 0, s));

    auto ts = boost::chrono::high_resolution_clock::now();

    Vec better = s;
    int cost_estim_min = cb_cost_estim(s, e);
    while (true)
    {
      // printf("search queue %d\n", (int)open_.size());
      if (open_.size() < 1)
      {
        // No fesible path
        // printf("No fesible path\n");
        if (return_best)
        {
          findPath(s, better, path);
        }
        return false;
      }
      PriorityVec center = open_.top();
      Vec p = center.v_;
      float c = center.p_raw_;
      float c_estim = center.p_;
      open_.pop();
      if (p == e || c_estim - c <= cost_leave)
      {
        e = p;
        break;
      }

      float &gp = g_[p];
      if (c > gp)
        continue;

      if (c_estim - c < cost_estim_min)
      {
        cost_estim_min = c_estim - c;
        better = p;
      }

      auto tnow = boost::chrono::high_resolution_clock::now();
      if (boost::chrono::duration<float>(tnow - ts).count() >= progress_interval)
      {
        std::list<Vec> path_tmp;
        ts = tnow;
        findPath(s, better, path_tmp);
        cb_progress(path_tmp);
      }

      std::vector<Vec> search_list = cb_search(p, s, e);
      int updates = 0;
      for (auto &diff : search_list)
      {
        Vec next = p + diff;
        for (int i = NONCYCLIC; i < DIM; i++)
        {
          next.cycle_unsigned(next[i], g_.size[i]);
        }
        if ((unsigned int)next[0] >= (unsigned int)g_.size[0] ||
            (unsigned int)next[1] >= (unsigned int)g_.size[1])
          continue;
        if (g_[next] < 0)
          continue;

        float cost_estim = cb_cost_estim(next, e);
        if (cost_estim < 0 || cost_estim == FLT_MAX)
          continue;

        float cost = cb_cost(p, next, s, e);
        if (cost < 0 || cost == FLT_MAX)
          continue;

        float &gnext = g_[next];
        if (gnext > c + cost)
        {
          gnext = c + cost;
          parents_[next] = p;
          open_.push(PriorityVec(c + cost + cost_estim, c + cost, next));
          if (queue_size_limit_ > 0 &&
              open_.size() > queue_size_limit_)
            open_.pop_back();
          updates++;
        }
      }
      if (updates == 0)
      {
        gp = -1;
      }
      // printf("(parents_ %d)\n", (int)parents_.size());
    }
    // printf("AStar search finished (parents_ %d)\n", (int)parents_.size());

    return findPath(s, e, path);
  }
  bool findPath(const Vec &s, const Vec &e, std::list<Vec> &path)
  {
    Vec n = e;
    while (true)
    {
      path.push_front(n);
      // printf("p- %d %d %d   %0.4f\n", n[0], n[1], n[2], g_[n]);
      if (n == s)
        break;
      if (parents_.find(n) == parents_.end())
      {
        n = parents_[n];
        // printf("px %d %d %d\n", n[0], n[1], n[2]);
        return false;
      }
      n = parents_[n];
    }
    return true;
  }
};

#endif  // GRID_ASTAR_H
