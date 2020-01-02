/*
 * Copyright (c) 2019-2020, the neonavigation authors
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

#include <cmath>
#include <utility>
#include <vector>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/grid_astar_model.h>
#include <planner_cspace/blockmem_gridmap.h>

#include <planner_cspace/planner_2dof_serial_joints/grid_astar_model.h>

namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
GridAstarModel2DoFSerialJoint::GridAstarModel2DoFSerialJoint(
    const Vecf& euclid_cost_coef,
    const int resolution,
    BlockMemGridmapBase<char, 2, 0>& cm,
    const CostCoeff& cc,
    const int range)
  : euclid_cost_coef_(euclid_cost_coef)
  , resolution_(resolution)
  , cm_(cm)
  , cc_(cc)
  , range_(range)
{
  for (Vec p(-range, -range); p[0] <= range; p[0]++)
  {
    for (p[1] = -range; p[1] <= range; p[1]++)
    {
      search_list_.push_back(p);
    }
  }
}
float GridAstarModel2DoFSerialJoint::euclidCost(const Vec& v) const
{
  auto vc = v;
  float cost = 0;
  for (int i = 0; i < 2; i++)
  {
    cost += std::abs(euclid_cost_coef_[i] * vc[i]);
  }
  return cost;
}
float GridAstarModel2DoFSerialJoint::cost(
    const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const
{
  if ((unsigned int)next[0] >= (unsigned int)resolution_ * 2 ||
      (unsigned int)next[1] >= (unsigned int)resolution_ * 2)
    return -1;
  Vec d = next - cur;
  d.cycle(resolution_, resolution_);

  float cost = euclidCost(d);

  float distf = std::hypot(static_cast<float>(d[0]), static_cast<float>(d[1]));
  float v[2], dp[2];
  int sum = 0;
  const int dist = distf;
  distf /= dist;
  v[0] = cur[0];
  v[1] = cur[1];
  dp[0] = static_cast<float>(d[0]) / dist;
  dp[1] = static_cast<float>(d[1]) / dist;
  Vec pos;
  for (int i = 0; i < dist; i++)
  {
    pos[0] = std::lround(v[0]);
    pos[1] = std::lround(v[1]);
    pos.cycleUnsigned(resolution_, resolution_);
    const auto c = cm_[pos];
    if (c > 99)
      return -1;
    sum += c;
    v[0] += dp[0];
    v[1] += dp[1];
  }
  cost += sum * cc_.weight_cost_ / 100.0;
  return cost;
}

float GridAstarModel2DoFSerialJoint::costEstim(
    const Vec& cur, const Vec& goal) const
{
  const Vec d = goal - cur;
  return euclidCost(d);
}
const std::vector<GridAstarModel2DoFSerialJoint::Vec>& GridAstarModel2DoFSerialJoint::searchGrids(
    const Vec& p,
    const std::vector<VecWithCost>& ss,
    const Vec& es) const
{
  return search_list_;
}
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace
