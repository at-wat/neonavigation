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

template <int DIM = 3, int NONCYCLIC = 2>
class grid_astar
{
public:
  const int get_dim()
  {
    return DIM;
  }
  const int get_noncyclic()
  {
    return NONCYCLIC;
  }
  class vec
  {
  public:
    int e[DIM];

    explicit vec(const int *v)
    {
      for (int i = 0; i < DIM; i++)
        e[i] = v[i];
    }
    explicit vec(const float *v)
    {
      for (int i = 0; i < DIM; i++)
        e[i] = lroundf(v[i]);
    }
    vec()
    {
    }
    void cycle(int &v, const int c)
    {
      v = v % c;
      if (v < 0)
        v += c;
      if (v > c / 2)
        v -= c;
    }
    void cycle_unsigned(int &v, const int c)
    {
      v = v % c;
      if (v < 0)
        v += c;
    }
    bool operator==(const vec &v) const
    {
      for (int i = 0; i < DIM; i++)
        if (v.e[i] != this->e[i])
          return false;
      return true;
    }
    bool operator!=(const vec &v) const
    {
      return !(*this == v);
    }
    vec operator+(const vec &v) const
    {
      vec out = *this;
      for (int i = 0; i < DIM; i++)
      {
        out[i] += v[i];
      }
      return out;
    }
    vec operator-(const vec &v) const
    {
      vec out = *this;
      for (int i = 0; i < DIM; i++)
      {
        out[i] -= v[i];
      }
      return out;
    }
    int cross(const vec &a) const
    {
      return (*this)[0] * a[1] - (*this)[1] * a[0];
    }
    int dot(const vec &a) const
    {
      return (*this)[0] * a[0] + (*this)[1] * a[1];
    }
    float dist_line(const vec &a, const vec &b) const
    {
      return (b - a).cross((*this) - a) / (b - a).len();
    }
    float dist_linestrip(const vec &a, const vec &b) const
    {
      auto to_a = (*this) - a;
      if ((b - a).dot(to_a) <= 0)
        return to_a.len();
      auto to_b = (*this) - b;
      if ((a - b).dot(to_b) <= 0)
        return to_b.len();
      return fabs(this->dist_line(a, b));
    }
    int &operator[](const int &x)
    {
      return e[x];
    }
    const int &operator[](const int &x) const
    {
      return e[x];
    }
    void set(const int *init)
    {
      for (int i = 0; i < DIM; i++)
      {
        e[i] = init[i];
      }
    }
    int sqlen() const
    {
      int out = 0;
      for (int i = 0; i < NONCYCLIC; i++)
      {
        out += e[i] * e[i];
      }
      return out;
    }
    float len() const
    {
      return sqrtf(sqlen());
    }
    float norm() const
    {
      float out = 0;
      for (int i = 0; i < DIM; i++)
      {
        out += powf(e[i], 2.0);
      }
      return sqrtf(out);
    }
    // Hash
  private:
    size_t rotl(const std::size_t x, const std::size_t n) const
    {
      return (x << n) | (x >> ((8 * sizeof(std::size_t)) - n));
    }

  public:
    size_t operator()(const vec &key) const
    {
      std::size_t hash = key.e[0];
      for (int i = 1; i < DIM; i++)
        hash ^= rotl(key.e[i], 8 * sizeof(std::size_t) * i / DIM);
      return hash;
    }
  };
  class vecf
  {
  public:
    float e[DIM];
    explicit vecf(const float *v)
    {
      for (int i = 0; i < DIM; i++)
        e[i] = v[i];
    }
    explicit vecf(const vec &v)
    {
      for (int i = 0; i < DIM; i++)
        e[i] = v[i];
    }
    vecf()
    {
    }
    float &operator[](const int &x)
    {
      return e[x];
    }
    const float &operator[](const int &x) const
    {
      return e[x];
    }
    vecf operator*(const vecf &v) const
    {
      vecf out;
      for (int i = 0; i < DIM; i++)
      {
        out[i] = e[i] * v[i];
      }
      return out;
    }
    vecf operator*(const vec &v) const
    {
      vecf out;
      for (int i = 0; i < DIM; i++)
      {
        out[i] = e[i] * v[i];
      }
      return out;
    }
    vecf operator-(const vecf &v) const
    {
      vecf out = *this;
      for (int i = 0; i < DIM; i++)
      {
        out[i] -= v[i];
      }
      return out;
    }
    float sqlen() const
    {
      float out = 0;
      for (int i = 0; i < NONCYCLIC; i++)
      {
        out += e[i] * e[i];
      }
      return out;
    }
    float len() const
    {
      return sqrtf(sqlen());
    }
    float norm() const
    {
      float out = 0;
      for (int i = 0; i < DIM; i++)
      {
        out += powf(e[i], 2.0);
      }
      return sqrtf(out);
    }
  };
  class pq
  {
  public:
    float p;
    float p_raw;
    vec v;
    pq()
    {
      p = 0;
    }
    pq(const float &p, const float &p_raw, const vec &v)
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
  template <class T, int block_width = 0x20>
  class gridmap
  {
  public:
    std::unique_ptr<T[]> c;
    vec size;
    vec block_size;
    size_t ser_size;
    size_t block_ser_size;
    size_t block_num;
    void clear(const T zero)
    {
      for (size_t i = 0; i < ser_size; i++)
      {
        c[i] = zero;
      }
    }
    void clear_positive(const T zero)
    {
      for (size_t i = 0; i < ser_size; i++)
      {
        if (c[i] >= 0)
          c[i] = zero;
      }
    }
    void reset(const vec &size)
    {
      this->size = size;

      for (int i = 0; i < NONCYCLIC; i++)
      {
        if (this->size[i] < static_cast<int>(block_width))
          this->size[i] = block_width;
      }

      block_ser_size = 1;
      block_num = 1;
      for (int i = 0; i < DIM; i++)
      {
        int width;
        if (i < NONCYCLIC)
        {
          width = block_width;
          block_size[i] = (this->size[i] + width - 1) / width;
        }
        else
        {
          width = this->size[i];
          block_size[i] = 1;
        }

        block_ser_size *= width;
        block_num *= block_size[i];
      }
      ser_size = block_ser_size * block_num;

      c.reset(new T[ser_size]);
    }
    explicit gridmap(const vec &size)
    {
      reset(size);
    }
    gridmap()
    {
    }
    void block_addr(const vec &pos, size_t &baddr, size_t &addr)
    {
      addr = 0;
      baddr = 0;
      for (int i = 0; i < NONCYCLIC; i++)
      {
        addr *= block_width;
        addr += pos[i] % block_width;
        baddr *= block_size[i];
        baddr += pos[i] / block_width;
      }
      for (int i = NONCYCLIC; i < DIM; i++)
      {
        addr *= size[i];
        addr += pos[i];
      }
    }
    T &operator[](const vec &pos)
    {
      size_t baddr, addr;
      block_addr(pos, baddr, addr);
      return c[baddr * block_ser_size + addr];
    }
    const gridmap<T, block_width> &operator=(const gridmap<T, block_width> &gm)
    {
      reset(gm.size);
      memcpy(c.get(), gm.c.get(), ser_size);

      return *this;
    }
  };

  gridmap<float> g;
  std::unordered_map<vec, vec, vec> parents;
  reservable_priority_queue<pq> open;
  size_t queue_size_limit;

  void reset(const vec size)
  {
    g.reset(size);
    g.clear(FLT_MAX);
    parents.reserve(g.ser_size / 16);
    open.reserve(g.ser_size / 16);
  }
  grid_astar()
  {
    queue_size_limit = 0;
  }
  explicit grid_astar(const vec size)
  {
    reset(size);
    queue_size_limit = 0;
  }
  void set_queue_size_limit(const size_t size)
  {
    queue_size_limit = size;
  }

  bool search(const vec &s, const vec &e,
              std::list<vec> &path,
              std::function<float(const vec &, vec &, const vec &, const vec &)> cb_cost,
              std::function<float(const vec &, const vec &)> cb_cost_estim,
              std::function<std::vector<vec> &(const vec &, const vec &, const vec &)> cb_search,
              std::function<bool(const std::list<vec> &)> cb_progress,
              const float cost_leave,
              const float progress_interval,
              const bool return_best = false)
  {
    return search_impl(g, s, e, path,
                       cb_cost, cb_cost_estim, cb_search, cb_progress,
                       cost_leave, progress_interval, return_best);
  }
  bool search_impl(gridmap<float> &g,
                   const vec &st, const vec &en,
                   std::list<vec> &path,
                   std::function<float(const vec &, vec &, const vec &, const vec &)> cb_cost,
                   std::function<float(const vec &, const vec &)> cb_cost_estim,
                   std::function<std::vector<vec> &(const vec &, const vec &, const vec &)> cb_search,
                   std::function<bool(const std::list<vec> &)> cb_progress,
                   const float cost_leave,
                   const float progress_interval,
                   const bool return_best = false)
  {
    if (st == en)
    {
      return false;
    }
    auto s = st;
    auto e = en;
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

    vec better = s;
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
          find_path(s, better, path);
        }
        return false;
      }
      pq center = open.top();
      auto p = center.v;
      auto c = center.p_raw;
      auto c_estim = center.p;
      open.pop();
      if (p == e || c_estim - c <= cost_leave)
      {
        e = p;
        break;
      }

      auto &gp = g[p];
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
        std::list<vec> path_tmp;
        ts = tnow;
        find_path(s, better, path_tmp);
        cb_progress(path_tmp);
      }

      auto search_list = cb_search(p, s, e);
      int updates = 0;
      for (auto &diff : search_list)
      {
        auto next = p + diff;
        for (int i = NONCYCLIC; i < DIM; i++)
        {
          next.cycle_unsigned(next[i], g.size[i]);
        }
        if ((unsigned int)next[0] >= (unsigned int)g.size[0] ||
            (unsigned int)next[1] >= (unsigned int)g.size[1])
          continue;
        if (g[next] < 0)
          continue;

        auto cost_estim = cb_cost_estim(next, e);
        if (cost_estim < 0 || cost_estim == FLT_MAX)
          continue;

        auto cost = cb_cost(p, next, s, e);
        if (cost < 0 || cost == FLT_MAX)
          continue;

        auto &gnext = g[next];
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

    return find_path(s, e, path);
  }
  bool find_path(const vec &s, const vec &e, std::list<vec> &path)
  {
    vec n = e;
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
