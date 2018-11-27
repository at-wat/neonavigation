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

#ifndef PLANNER_CSPACE_CYCLIC_VEC_H
#define PLANNER_CSPACE_CYCLIC_VEC_H

#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <initializer_list>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include <boost/chrono.hpp>

#include <planner_cspace/reservable_priority_queue.h>

template <int DIM, int NONCYCLIC, typename T>
class CyclicVecBase
{
protected:
  T e_[DIM];

public:
  explicit CyclicVecBase(const T* v) noexcept
  {
    for (int i = 0; i < DIM; i++)
      e_[i] = v[i];
  }
  explicit CyclicVecBase(const std::initializer_list<T> vl) noexcept
  {
    assert(vl.size() == DIM);
    auto it = vl.begin();
    for (int i = 0; i < DIM; ++i, ++it)
      e_[i] = *it;
  }
  CyclicVecBase() noexcept
  {
  }
  template <typename C>
  bool operator==(const C& v) const
  {
    for (int i = 0; i < DIM; i++)
      if (v.e_[i] != e_[i])
        return false;
    return true;
  }
  template <typename C>
  bool operator!=(const C& v) const
  {
    return !(*this == v);
  }
  template <typename C>
  C operator+(const C& v) const
  {
    C out(*this);
    for (int i = 0; i < DIM; i++)
    {
      out[i] += v[i];
    }
    return out;
  }
  template <typename C>
  C operator-(const C& v) const
  {
    C out(*this);
    for (int i = 0; i < DIM; i++)
    {
      out[i] -= v[i];
    }
    return out;
  }
  template <typename C>
  C operator*(const C& v) const
  {
    C out;
    for (int i = 0; i < DIM; i++)
    {
      out[i] = e_[i] * v[i];
    }
    return out;
  }
  template <typename C>
  int cross2d(const C& a) const
  {
    return (*this)[0] * a[1] - (*this)[1] * a[0];
  }
  template <typename C>
  int dot2d(const C& a) const
  {
    return (*this)[0] * a[0] + (*this)[1] * a[1];
  }
  template <typename C>
  float distLine2d(const C& a, const C& b) const
  {
    return (b - a).cross2d((*this) - a) / (b - a).len();
  }
  template <typename C>
  float distLinestrip2d(const C& a, const C& b) const
  {
    auto to_a = (*this) - a;
    if ((b - a).dot2d(to_a) <= 0)
      return to_a.len();
    auto to_b = (*this) - b;
    if ((a - b).dot2d(to_b) <= 0)
      return to_b.len();
    return fabs(distLine2d(a, b));
  }
  T& operator[](const int& x)
  {
    return e_[x];
  }
  const T& operator[](const int& x) const
  {
    return e_[x];
  }
  void set(const T* init)
  {
    for (int i = 0; i < DIM; i++)
    {
      e_[i] = init[i];
    }
  }
  T sqlen() const
  {
    T out = 0;
    for (int i = 0; i < NONCYCLIC; i++)
    {
      out += e_[i] * e_[i];
    }
    return out;
  }
  float len() const
  {
    return sqrtf(sqlen());
  }
  float gridToLenFactor() const
  {
    const auto l = len();
    return l / static_cast<int>(l);
  }
  float norm() const
  {
    float out = 0;
    for (int i = 0; i < DIM; i++)
    {
      out += powf(e_[i], 2.0);
    }
    return sqrtf(out);
  }

  // Cyclic operations
  void cycle(int& v, const int c)
  {
    v = v % c;
    if (v < 0)
      v += c;
    if (v > c / 2)
      v -= c;
  }
  void cycleUnsigned(int& v, const int c)
  {
    v = v % c;
    if (v < 0)
      v += c;
  }

  // Hash
  size_t operator()(const CyclicVecBase& key) const
  {
    size_t hash = static_cast<size_t>(key.e_[0]);
    for (int i = 1; i < DIM; i++)
    {
      const size_t n = 8 * sizeof(std::size_t) * i / DIM;
      const size_t x = static_cast<size_t>(key.e_[i]);
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
  CyclicVecInt() noexcept
  {
  }
  explicit CyclicVecInt(const float* v) noexcept
  {
    for (int i = 0; i < DIM; i++)
      this->e_[i] = lroundf(v[i]);
  }
  explicit CyclicVecInt(const CyclicVecBase<DIM, NONCYCLIC, int>& c) noexcept
  {
    for (int i = 0; i < DIM; i++)
      this->e_[i] = c[i];
  }
};

template <int DIM, int NONCYCLIC>
class CyclicVecFloat : public CyclicVecBase<DIM, NONCYCLIC, float>
{
public:
  using CyclicVecBase<DIM, NONCYCLIC, float>::CyclicVecBase;

public:
  CyclicVecFloat() noexcept
  {
  }
  explicit CyclicVecFloat(const CyclicVecBase<DIM, NONCYCLIC, float>& c) noexcept
  {
    for (int i = 0; i < DIM; i++)
      this->e_[i] = c[i];
  }
  explicit CyclicVecFloat(const CyclicVecBase<DIM, NONCYCLIC, int>& c) noexcept
  {
    for (int i = 0; i < DIM; i++)
      this->e_[i] = c[i];
  }
  void rotate(const float ang)
  {
    const auto tmp = *this;
    const float cos_v = cosf(ang);
    const float sin_v = sinf(ang);

    this->e_[0] = cos_v * tmp[0] - sin_v * tmp[1];
    this->e_[1] = sin_v * tmp[0] + cos_v * tmp[1];
    this->e_[2] = tmp[2] + ang;
    if (this->e_[2] > M_PI)
      this->e_[2] -= 2 * M_PI;
    else if (this->e_[2] < -M_PI)
      this->e_[2] += 2 * M_PI;
  }
};

#endif  // PLANNER_CSPACE_CYCLIC_VEC_H
