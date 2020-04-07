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

#ifndef PLANNER_CSPACE_PLANNER_3D_MOTION_CACHE_H
#define PLANNER_CSPACE_PLANNER_3D_MOTION_CACHE_H

#include <memory>
#include <unordered_map>
#include <vector>

#include <planner_cspace/cyclic_vec.h>

namespace planner_cspace
{
namespace planner_3d
{
class MotionCache
{
public:
  class Page
  {
  protected:
    friend class MotionCache;

    std::vector<CyclicVecInt<3, 2>> motion_;
    float distance_;

  public:
    inline float getDistance() const
    {
      return distance_;
    }
    const std::vector<CyclicVecInt<3, 2>>& getMotion() const
    {
      return motion_;
    }
  };

  using Cache =
      std::unordered_map<CyclicVecInt<3, 2>, Page, CyclicVecInt<3, 2>>;

  using Ptr = std::shared_ptr<MotionCache>;

  inline const typename Cache::const_iterator find(
      const int start_yaw,
      const CyclicVecInt<3, 2>& goal) const
  {
    int i = start_yaw % page_size_;
    if (i < 0)
      i += page_size_;
    return cache_[i].find(goal);
  }
  inline const typename Cache::const_iterator end(
      const int start_yaw) const
  {
    int i = start_yaw % page_size_;
    if (i < 0)
      i += page_size_;
    return cache_[i].cend();
  }

  inline const CyclicVecInt<3, 2>& getMaxRange() const
  {
    return max_range_;
  }

  void reset(
      const float linear_resolution,
      const float angular_resolution,
      const int range,
      const std::function<void(CyclicVecInt<3, 2>, size_t&, size_t&)> gm_addr);

protected:
  std::vector<Cache> cache_;
  int page_size_;
  CyclicVecInt<3, 2> max_range_;
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_MOTION_CACHE_H
