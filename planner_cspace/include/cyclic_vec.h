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

#ifndef CYCLIC_VEC_H
#define CYCLIC_VEC_H

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

template <int DIM, int NONCYCLIC, typename T>
class CyclicVecBase
{
public:
  T e[DIM];

  explicit CyclicVecBase(const T *v)
  {
    for (int i = 0; i < DIM; i++)
      e[i] = v[i];
  }
  CyclicVecBase()
  {
  }
  bool operator==(const CyclicVecBase &v) const
  {
    for (int i = 0; i < DIM; i++)
      if (v.e[i] != this->e[i])
        return false;
    return true;
  }
  bool operator!=(const CyclicVecBase &v) const
  {
    return !(*this == v);
  }
  CyclicVecBase operator+(const CyclicVecBase &v) const
  {
    CyclicVecBase out = *this;
    for (int i = 0; i < DIM; i++)
    {
      out[i] += v[i];
    }
    return out;
  }
  CyclicVecBase operator-(const CyclicVecBase &v) const
  {
    CyclicVecBase out = *this;
    for (int i = 0; i < DIM; i++)
    {
      out[i] -= v[i];
    }
    return out;
  }
  CyclicVecBase operator*(const CyclicVecBase &v) const
  {
    CyclicVecBase out;
    for (int i = 0; i < DIM; i++)
    {
      out[i] = e[i] * v[i];
    }
    return out;
  }
  int cross(const CyclicVecBase &a) const
  {
    return (*this)[0] * a[1] - (*this)[1] * a[0];
  }
  int dot(const CyclicVecBase &a) const
  {
    return (*this)[0] * a[0] + (*this)[1] * a[1];
  }
  float dist_line(const CyclicVecBase &a, const CyclicVecBase &b) const
  {
    return (b - a).cross((*this) - a) / (b - a).len();
  }
  float dist_linestrip(const CyclicVecBase &a, const CyclicVecBase &b) const
  {
    auto to_a = (*this) - a;
    if ((b - a).dot(to_a) <= 0)
      return to_a.len();
    auto to_b = (*this) - b;
    if ((a - b).dot(to_b) <= 0)
      return to_b.len();
    return fabs(this->dist_line(a, b));
  }
  T &operator[](const int &x)
  {
    return e[x];
  }
  const T &operator[](const int &x) const
  {
    return e[x];
  }
  void set(const T *init)
  {
    for (int i = 0; i < DIM; i++)
    {
      e[i] = init[i];
    }
  }
  int sqlen() const
  {
    T out = 0;
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

  // Cyclic operations
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

  // Hash
  size_t operator()(const CyclicVecBase &key) const
  {
    size_t hash = static_cast<size_t>(key.e[0]);
    for (int i = 1; i < DIM; i++)
    {
      const size_t n = 8 * sizeof(std::size_t) * i / DIM;
      const size_t x = static_cast<size_t>(key.e[i]);
      hash ^= (x << n) | (x >> ((8 * sizeof(size_t)) - n));
    }
    return hash;
  }
};

template <int DIM, int NONCYCLIC>
class CyclicVecInt : public CyclicVecBase<DIM, NONCYCLIC, int>
{
public:
  using CyclicVecBase<DIM, NONCYCLIC, int>::CyclicVecBase;

public:
  CyclicVecInt()
  {
  }
  explicit CyclicVecInt(const float *v)
  {
    for (int i = 0; i < DIM; i++)
      this->e[i] = lroundf(v[i]);
  }
  CyclicVecInt(const CyclicVecBase<DIM, NONCYCLIC, int> &c)
  {
    for (int i = 0; i < DIM; i++)
      this->e[i] = c.e[i];
  }
};

template <int DIM, int NONCYCLIC>
class CyclicVecFloat : public CyclicVecBase<DIM, NONCYCLIC, float>
{
public:
  using CyclicVecBase<DIM, NONCYCLIC, float>::CyclicVecBase;

public:
  CyclicVecFloat()
  {
  }
  CyclicVecFloat(const CyclicVecBase<DIM, NONCYCLIC, float> &c)
  {
    for (int i = 0; i < DIM; i++)
      this->e[i] = c.e[i];
  }
};

#endif  // CYCLIC_VEC_H
