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

#ifndef PLANNER_CSPACE_PLANNER_3D_GRID_ASTAR_MODEL_H
#define PLANNER_CSPACE_PLANNER_3D_GRID_ASTAR_MODEL_H

#include <memory>
#include <utility>
#include <vector>

#include <costmap_cspace_msgs/MapMetaData3D.h>

#include <planner_cspace/blockmem_gridmap.h>
#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/grid_astar_model.h>
#include <planner_cspace/planner_3d/motion_cache.h>
#include <planner_cspace/planner_3d/path_interpolator.h>
#include <planner_cspace/planner_3d/rotation_cache.h>

namespace planner_cspace
{
namespace planner_3d
{
class CostCoeff
{
public:
  float weight_decel_;
  float weight_backward_;
  float weight_ang_vel_;
  float weight_costmap_;
  float weight_costmap_turn_;
  float weight_remembered_;
  float weight_hysteresis_;
  float in_place_turn_;
  float hysteresis_max_dist_;
  float hysteresis_expand_;
  float min_curve_radius_;
  float max_vel_;
  float max_ang_vel_;
  float angle_resolution_aspect_;
};

class GridAstarModel3D : public GridAstarModelBase<3, 2>
{
public:
  friend class GridAstarModel2D;
  using Ptr = std::shared_ptr<GridAstarModel3D>;
  using ConstPtr = std::shared_ptr<const GridAstarModel3D>;
  using Vec = CyclicVecInt<3, 2>;
  using Vecf = CyclicVecFloat<3, 2>;

  PathInterpolator path_interpolator_;

protected:
  bool hysteresis_;
  costmap_cspace_msgs::MapMetaData3D map_info_;
  Vecf euclid_cost_coef_;
  Vecf resolution_;
  std::vector<std::vector<Vec>> motion_primitives_;
  std::vector<Vec> search_list_rough_;
  int local_range_;
  BlockMemGridmapBase<float, 3, 2>& cost_estim_cache_;
  BlockMemGridmapBase<char, 3, 2>& cm_;
  BlockMemGridmapBase<char, 3, 2>& cm_hyst_;
  BlockMemGridmapBase<char, 3, 2>& cm_rough_;
  const CostCoeff& cc_;
  int range_;
  RotationCache rot_cache_;
  MotionCache motion_cache_;
  MotionCache motion_cache_linear_;
  Vec min_boundary_;
  Vec max_boundary_;
  std::array<float, 1024> euclid_cost_lin_cache_;

public:
  explicit GridAstarModel3D(
      const costmap_cspace_msgs::MapMetaData3D& map_info,
      const Vecf& euclid_cost_coef,
      const int local_range,
      BlockMemGridmapBase<float, 3, 2>& cost_estim_cache,
      BlockMemGridmapBase<char, 3, 2>& cm,
      BlockMemGridmapBase<char, 3, 2>& cm_hyst,
      BlockMemGridmapBase<char, 3, 2>& cm_rough,
      const CostCoeff& cc,
      const int range);
  void enableHysteresis(const bool enable);
  void createEuclidCostCache();
  float euclidCost(const Vec& v) const;
  float euclidCostRough(const Vec& v) const;
  float cost(
      const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const override;

  float costEstim(
      const Vec& cur, const Vec& goal) const override;
  const std::vector<Vec>& searchGrids(
      const Vec& p,
      const std::vector<VecWithCost>& ss,
      const Vec& es) const override;
};

class GridAstarModel2D : public GridAstarModelBase<3, 2>
{
public:
  using Ptr = std::shared_ptr<GridAstarModel2D>;
  const GridAstarModel3D::ConstPtr base_;

  inline explicit GridAstarModel2D(const GridAstarModel3D::ConstPtr base)
    : base_(base)
  {
  }

  float cost(
      const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const final;
  float costEstim(
      const Vec& cur, const Vec& goal) const final;
  const std::vector<Vec>& searchGrids(
      const Vec& cur, const std::vector<VecWithCost>& start, const Vec& goal) const final;
};
}  // namespace planner_3d
}  // namespace planner_cspace

#endif  // PLANNER_CSPACE_PLANNER_3D_GRID_ASTAR_MODEL_H
