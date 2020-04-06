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

#ifndef PLANNER_CSPACE_PLANNER_2DOF_SERIAL_JOINTS_GRID_ASTAR_MODEL_H
#define PLANNER_CSPACE_PLANNER_2DOF_SERIAL_JOINTS_GRID_ASTAR_MODEL_H

#include <memory>
#include <utility>
#include <vector>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/grid_astar_model.h>
#include <planner_cspace/blockmem_gridmap.h>

namespace planner_cspace
{
namespace planner_2dof_serial_joints
{
class CostCoeff
{
public:
  float weight_cost_;
  float expand_;
};

class GridAstarModel2DoFSerialJoint : public GridAstarModelBase<2, 0>
{
public:
  using Ptr = std::shared_ptr<GridAstarModel2DoFSerialJoint>;
  using ConstPtr = std::shared_ptr<const GridAstarModel2DoFSerialJoint>;
  using Vec = CyclicVecInt<2, 0>;
  using Vecf = CyclicVecFloat<2, 0>;

protected:
  std::vector<Vec> search_list_;
  Vecf euclid_cost_coef_;
  int resolution_;
  BlockMemGridmapBase<char, 2, 0>& cm_;
  CostCoeff cc_;
  int range_;

public:
  GridAstarModel2DoFSerialJoint(
      const Vecf& euclid_cost_coef,
      const int resolution,
      BlockMemGridmapBase<char, 2, 0>& cm,
      const CostCoeff& cc,
      const int range);
  float euclidCost(const Vec& v) const;
  float cost(
      const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const override;
  float costEstim(
      const Vec& cur, const Vec& goal) const override;
  const std::vector<Vec>& searchGrids(
      const Vec& p,
      const std::vector<VecWithCost>& ss,
      const Vec& es) const override;
};
}  // namespace planner_2dof_serial_joints
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_2DOF_SERIAL_JOINTS_GRID_ASTAR_MODEL_H
