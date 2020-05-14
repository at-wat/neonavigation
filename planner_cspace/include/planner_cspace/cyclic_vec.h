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

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <initializer_list>
#include <list>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <boost/chrono.hpp>

#include <planner_cspace/reservable_priority_queue.h>

namespace planner_cspace
{
namespace cyclic_vec_type_conversion_rule
{
template <typename T>
void convert(const T val, float& ret)
{
  ret = val;
}
inline void convert(const int val, float& ret)
{
  ret = val;
}
inline void convert(const float val, int& ret)
{
  ret = std::lround(val);
}

inline void normalizeFloatAngle(float& val)
{
  if (val > M_PI)
    val -= 2 * M_PI;
  else if (val < -M_PI)
    val += 2 * M_PI;
}
inline void normalizeFloatAngle(int&)
{
}
}  // namespace cyclic_vec_type_conversion_rule

template <int DIM, int NONCYCLIC, typename T>
class CyclicVecBase
{
protected:
  static_assert(
      std::is_same<float, T>() || std::is_same<int, T>(), "T must be float or int");
  T e_[DIM];

  template <typename T2, typename... ArgList>
  void setElements(const int i, const T2& first, const ArgList&... rest) noexcept
  {
    assert(i < DIM);

    cyclic_vec_type_conversion_rule::convert(first, e_[i]);
    setElements(i + 1, rest...);
  }
  void setElements(const int i) noexcept
  {
    assert(i == DIM);
  }

  template <typename... ArgList>
  void cycleElements(
      const int i,
      const int res, const ArgList&... rest)
  {
    cycleElement(i, res);
    cycleElements(i + 1, rest...);
  }
  void cycleElement(const int i, const int res)
  {
    assert(i < DIM);

    e_[i] = e_[i] % res;
    if (e_[i] < res / 2 - res)
      e_[i] += res;
    else if (e_[i] >= res / 2)
      e_[i] -= res;
  }
  void cycleElements(const int i)
  {
    assert(i == DIM);
  }
  template <typename... ArgList>
  void cycleUnsignedElements(
      const int i,
      const int res, const ArgList&... rest)
  {
    cycleUnsignedElement(i, res);
    cycleUnsignedElements(i + 1, rest...);
  }
  void cycleUnsignedElement(const int i, const int res)
  {
    assert(i < DIM);

    e_[i] = e_[i] % res;
    if (e_[i] < 0)
      e_[i] += res;
  }
  void cycleUnsignedElements(const int i)
  {
    assert(i == DIM);
  }

public:
  template <typename T2>
  explicit CyclicVecBase(const CyclicVecBase<DIM, NONCYCLIC, T2>& c) noexcept
  {
    for (int i = 0; i < DIM; i++)
      cyclic_vec_type_conversion_rule::convert(c[i], e_[i]);
  }
  template <typename... ArgList>
  explicit CyclicVecBase(const float& v, const ArgList&... args) noexcept
  {
    setElements(0, v, args...);
  }
  template <typename... ArgList>
  explicit CyclicVecBase(const int& v, const ArgList&... args) noexcept
  {
    setElements(0, v, args...);
  }
  CyclicVecBase() noexcept
  {
    for (int i = 0; i < DIM; i++)
      e_[i] = 0;
  }
  template <typename T2>
  bool operator==(const CyclicVecBase<DIM, NONCYCLIC, T2>& v) const
  {
    for (int i = 0; i < DIM; i++)
      if (v.e_[i] != e_[i])
        return false;
    return true;
  }
  template <typename T2>
  bool operator!=(const CyclicVecBase<DIM, NONCYCLIC, T2>& v) const
  {
    return !(*this == v);
  }
  template <typename T2>
  CyclicVecBase<DIM, NONCYCLIC, T2> operator+(const CyclicVecBase<DIM, NONCYCLIC, T2>& v) const
  {
    CyclicVecBase<DIM, NONCYCLIC, T2> out(*this);
    for (int i = 0; i < DIM; i++)
    {
      out[i] += v[i];
    }
    return out;
  }
  template <typename T2>
  CyclicVecBase<DIM, NONCYCLIC, T2> operator-(const CyclicVecBase<DIM, NONCYCLIC, T2>& v) const
  {
    CyclicVecBase<DIM, NONCYCLIC, T2> out(*this);
    for (int i = 0; i < DIM; i++)
    {
      out[i] -= v[i];
    }
    return out;
  }
  template <typename T2>
  CyclicVecBase<DIM, NONCYCLIC, T2> operator*(const CyclicVecBase<DIM, NONCYCLIC, T2>& v) const
  {
    CyclicVecBase<DIM, NONCYCLIC, T2> out;
    for (int i = 0; i < DIM; i++)
    {
      out[i] = e_[i] * v[i];
    }
    return out;
  }
  float cross2d(const CyclicVecBase<DIM, NONCYCLIC, T>& a) const
  {
    return (*this)[0] * a[1] - (*this)[1] * a[0];
  }
  float dot2d(const CyclicVecBase<DIM, NONCYCLIC, T>& a) const
  {
    return (*this)[0] * a[0] + (*this)[1] * a[1];
  }
  float distLine2d(
      const CyclicVecBase<DIM, NONCYCLIC, T>& a,
      const CyclicVecBase<DIM, NONCYCLIC, T>& b) const
  {
    return (b - a).cross2d((*this) - a) / (b - a).len();
  }
  float distLinestrip2d(
      const CyclicVecBase<DIM, NONCYCLIC, T>& a,
      const CyclicVecBase<DIM, NONCYCLIC, T>& b) const
  {
    const auto to_a = (*this) - a;
    if ((b - a).dot2d(to_a) <= 0)
      return to_a.len();
    const auto to_b = (*this) - b;
    if ((a - b).dot2d(to_b) <= 0)
      return to_b.len();
    return std::abs(distLine2d(a, b));
  }
  T& operator[](const int& x)
  {
    return e_[x];
  }
  const T& operator[](const int& x) const
  {
    return e_[x];
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
    return std::sqrt(sqlen());
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
      out += std::pow(e_[i], 2);
    }
    return std::sqrt(out);
  }

  void rotate(const float ang)
  {
    const auto tmp = *this;
    const float cos_v = cosf(ang);
    const float sin_v = sinf(ang);

    cyclic_vec_type_conversion_rule::convert(cos_v * tmp[0] - sin_v * tmp[1], e_[0]);
    cyclic_vec_type_conversion_rule::convert(sin_v * tmp[0] + cos_v * tmp[1], e_[1]);
    this->e_[2] = tmp[2] + ang;
    cyclic_vec_type_conversion_rule::normalizeFloatAngle(this->e_[2]);
  }

  // Cyclic operations
  template <typename... ArgList>
  void cycle(const int res, const ArgList&... rest)
  {
    static_assert(
        std::is_same<int, T>(), "cycle is provided only for int");
    cycleElements(NONCYCLIC, res, rest...);
  }
  template <typename... ArgList>
  void cycleUnsigned(const int res, const ArgList&... rest)
  {
    static_assert(
        std::is_same<int, T>(), "cycle is provided only for int");
    cycleUnsignedElements(NONCYCLIC, res, rest...);
  }
  void cycle(const CyclicVecBase<DIM, NONCYCLIC, T>& res)
  {
    static_assert(
        std::is_same<int, T>(), "cycle is provided only for int");
    for (int i = NONCYCLIC; i < DIM; ++i)
      cycleElement(i, res[i]);
  }
  void cycleUnsigned(const CyclicVecBase<DIM, NONCYCLIC, T>& res)
  {
    static_assert(
        std::is_same<int, T>(), "cycle is provided only for int");
    for (int i = NONCYCLIC; i < DIM; ++i)
      cycleUnsignedElement(i, res[i]);
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

  bool isExceeded(const CyclicVecBase<DIM, NONCYCLIC, int>& v) const
  {
    static_assert(
        std::is_same<int, T>(), "isExceeded is provided only for T=int");
    for (int i = 0; i < NONCYCLIC; ++i)
    {
      if (static_cast<unsigned int>((*this)[i]) >= static_cast<unsigned int>(v[i]))
        return true;
    }
    return false;
  }
};

template <int DIM, int NONCYCLIC>
using CyclicVecInt = CyclicVecBase<DIM, NONCYCLIC, int>;
template <int DIM, int NONCYCLIC>
using CyclicVecFloat = CyclicVecBase<DIM, NONCYCLIC, float>;

}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_CYCLIC_VEC_H
