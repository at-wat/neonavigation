/*
 * Copyright (c) 2018-2019, the neonavigation authors
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

#ifndef PLANNER_CSPACE_PLANNER_3D_ROTATION_CACHE_H
#define PLANNER_CSPACE_PLANNER_3D_ROTATION_CACHE_H

#include <cmath>
#include <list>
#include <memory>
#include <utility>
#include <vector>

#include <planner_cspace/cyclic_vec.h>

namespace planner_cspace
{
namespace planner_3d
{
class RotationCache
{
private:
  class Page
  {
  private:
    std::unique_ptr<CyclicVecFloat<3, 2>[]> c_;
    std::unique_ptr<std::pair<float, float>[]> r_;
    CyclicVecInt<3, 2> size_;
    int ser_size_;

    inline size_t addr(const CyclicVecInt<3, 2>& pos) const
    {
      size_t addr = pos[2];
      for (int i = 1; i >= 0; i--)
        addr = addr * size_[i] + pos[i];
      return addr;
    }

  public:
    void reset(const CyclicVecInt<3, 2>& size);
    inline CyclicVecFloat<3, 2>& motion(const CyclicVecInt<3, 2>& pos)
    {
      return c_[addr(pos)];
    }
    inline const CyclicVecFloat<3, 2>& motion(const CyclicVecInt<3, 2>& pos) const
    {
      return c_[addr(pos)];
    }
    inline std::pair<float, float>& radiuses(const CyclicVecInt<3, 2>& pos)
    {
      return r_[addr(pos)];
    }
    inline const std::pair<float, float>& radiuses(const CyclicVecInt<3, 2>& pos) const
    {
      return r_[addr(pos)];
    }
  };

  std::vector<Page> pages_;

public:
  void reset(const float linear_resolution, const float angular_resolution, const int range);
  inline const CyclicVecFloat<3, 2>& getMotion(
      const int start_angle,
      const CyclicVecInt<3, 2>& end) const
  {
    return pages_[start_angle].motion(end);
  }
  inline const std::pair<float, float>& getRadiuses(
      const int start_angle,
      const CyclicVecInt<3, 2>& end) const
  {
    return pages_[start_angle].radiuses(end);
  }

  std::list<CyclicVecFloat<3, 2>> interpolate(
      const std::list<CyclicVecInt<3, 2>>& path_grid,
      const float interval,
      const int local_range) const;
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_ROTATION_CACHE_H
