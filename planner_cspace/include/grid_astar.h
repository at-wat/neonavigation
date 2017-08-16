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
  const int getDim()
  {
    return DIM;
  }
  const int getNoncyclic()
  {
    return NONCYCLIC;
  }

  using Vec = CyclicVecInt<DIM, NONCYCLIC>;
  using Vecf = CyclicVecFloat<DIM, NONCYCLIC>;

  template <class T, int block_width = 0x20>
  class Gridmap : public BlockMemGridmap<T, DIM, NONCYCLIC, block_width>
  {
    using BlockMemGridmap<T, DIM, NONCYCLIC, block_width>::BlockMemGridmap;
  };

  class pq
  {
  public:
    float p;
    float p_raw;
    Vec v;
    pq()
    {
      p = 0;
    }
    pq(const float &p, const float &p_raw, const Vec &v)
    {
      this->p = p;
      this->p_raw = p_raw;
      this->v = v;
    }
    bool operator<(const pq &b) const
    {
      // smaller first
      return this->p > b.p;
    }
  };

  Gridmap<float> g;
  std::unordered_map<Vec, Vec, Vec> parents;
  reservable_priority_queue<pq> open;
  size_t queue_size_limit;

  void reset(const Vec size)
  {
    g.reset(size);
    g.clear(FLT_MAX);
    parents.reserve(g.ser_size / 16);
    open.reserve(g.ser_size / 16);
  }
  GridAstar()
  {
    queue_size_limit = 0;
  }
  explicit GridAstar(const Vec size)
  {
    reset(size);
    queue_size_limit = 0;
  }
  void setQueueSizeLimit(const size_t size)
  {
    queue_size_limit = size;
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
    return searchImpl(g, s, e, path,
                      cb_cost, cb_cost_estim, cb_search, cb_progress,
                      cost_leave, progress_interval, return_best);
  }
  bool searchImpl(Gridmap<float> &g,
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
      s.cycle_unsigned(s[i], g.size[i]);
      e.cycle_unsigned(e[i], g.size[i]);
    }
    g.clear(FLT_MAX);
    open.clear();
    parents.clear();

    g[s] = 0;
    open.push(pq(cb_cost_estim(s, e), 0, s));

    auto ts = boost::chrono::high_resolution_clock::now();

    Vec better = s;
    int cost_estim_min = cb_cost_estim(s, e);
    while (true)
    {
      // printf("search queue %d\n", (int)open.size());
      if (open.size() < 1)
      {
        // No fesible path
        // printf("No fesible path\n");
        if (return_best)
        {
          findPath(s, better, path);
        }
        return false;
      }
      pq center = open.top();
      Vec p = center.v;
      float c = center.p_raw;
      float c_estim = center.p;
      open.pop();
      if (p == e || c_estim - c <= cost_leave)
      {
        e = p;
        break;
      }

      float &gp = g[p];
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
          next.cycle_unsigned(next[i], g.size[i]);
        }
        if ((unsigned int)next[0] >= (unsigned int)g.size[0] ||
            (unsigned int)next[1] >= (unsigned int)g.size[1])
          continue;
        if (g[next] < 0)
          continue;

        float cost_estim = cb_cost_estim(next, e);
        if (cost_estim < 0 || cost_estim == FLT_MAX)
          continue;

        float cost = cb_cost(p, next, s, e);
        if (cost < 0 || cost == FLT_MAX)
          continue;

        float &gnext = g[next];
        if (gnext > c + cost)
        {
          gnext = c + cost;
          parents[next] = p;
          open.push(pq(c + cost + cost_estim, c + cost, next));
          if (queue_size_limit > 0 &&
              open.size() > queue_size_limit)
            open.pop_back();
          updates++;
        }
      }
      if (updates == 0)
      {
        gp = -1;
      }
      // printf("(parents %d)\n", (int)parents.size());
    }
    // printf("AStar search finished (parents %d)\n", (int)parents.size());

    return findPath(s, e, path);
  }
  bool findPath(const Vec &s, const Vec &e, std::list<Vec> &path)
  {
    Vec n = e;
    while (true)
    {
      path.push_front(n);
      // printf("p- %d %d %d   %0.4f\n", n[0], n[1], n[2], g[n]);
      if (n == s)
        break;
      if (parents.find(n) == parents.end())
      {
        n = parents[n];
        // printf("px %d %d %d\n", n[0], n[1], n[2]);
        return false;
      }
      n = parents[n];
    }
    return true;
  }
};

#endif  // GRID_ASTAR_H
