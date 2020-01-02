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

#ifndef PLANNER_CSPACE_GRID_ASTAR_MODEL_H
#define PLANNER_CSPACE_GRID_ASTAR_MODEL_H

#include <memory>
#include <vector>

#include <planner_cspace/cyclic_vec.h>

namespace planner_cspace
{
template <int DIM = 3, int NONCYCLIC = 2>
class GridAstarModelBase
{
public:
  using Ptr = typename std::shared_ptr<GridAstarModelBase<DIM, NONCYCLIC>>;
  using Vec = CyclicVecInt<DIM, NONCYCLIC>;
  using Vecf = CyclicVecFloat<DIM, NONCYCLIC>;

  class VecWithCost
  {
  public:
    Vec v_;
    float c_;
    explicit VecWithCost(const Vec& v, const float c = 0.0)
      : v_(v)
      , c_(c)
    {
    }
  };

  virtual float cost(
      const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const = 0;
  virtual float costEstim(
      const Vec& cur, const Vec& next) const = 0;
  virtual const std::vector<Vec>& searchGrids(
      const Vec& cur, const std::vector<VecWithCost>& start, const Vec& goal) const = 0;
};
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_GRID_ASTAR_MODEL_H
