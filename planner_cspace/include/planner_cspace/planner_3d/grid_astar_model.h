/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <utility>
#include <vector>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/motion_cache.h>
#include <planner_cspace/planner_3d/rotation_cache.h>
#include <planner_cspace/planner_3d/path_interpolator.h>

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
  float min_curve_raduis_;
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
  std::vector<Vec> search_list_;
  std::vector<Vec> search_list_rough_;
  int local_range_;
  BlockMemGridmap<float, 3, 2>& cost_estim_cache_;
  BlockMemGridmap<char, 3, 2, 0x40>& cm_;
  BlockMemGridmap<char, 3, 2, 0x80>& cm_hyst_;
  BlockMemGridmap<char, 3, 2, 0x80>& cm_rough_;
  CostCoeff cc_;
  int range_;
  RotationCache rot_cache_;
  MotionCache motion_cache_;
  MotionCache motion_cache_linear_;

public:
  explicit GridAstarModel3D(
      const costmap_cspace_msgs::MapMetaData3D& map_info,
      const Vecf& euclid_cost_coef,
      const Vecf& resolution,
      const int local_range,
      BlockMemGridmap<float, 3, 2>& cost_estim_cache,
      BlockMemGridmap<char, 3, 2, 0x40>& cm,
      BlockMemGridmap<char, 3, 2, 0x80>& cm_hyst,
      BlockMemGridmap<char, 3, 2, 0x80>& cm_rough,
      const CostCoeff& cc,
      const int range)
    : hysteresis_(false)
    , map_info_(map_info)
    , euclid_cost_coef_(euclid_cost_coef)
    , resolution_(resolution)
    , local_range_(local_range)
    , cost_estim_cache_(cost_estim_cache)
    , cm_(cm)
    , cm_hyst_(cm_hyst)
    , cm_rough_(cm_rough)
    , cc_(cc)
    , range_(range)
  {
    rot_cache_.reset(map_info_.linear_resolution, map_info_.angular_resolution, range_);

    costmap_cspace_msgs::MapMetaData3D map_info_linear(map_info_);
    map_info_linear.angle = 1;

    motion_cache_linear_.reset(
        map_info_linear.linear_resolution,
        map_info_linear.angular_resolution,
        range_,
        cm_rough_.getAddressor());
    motion_cache_.reset(
        map_info_.linear_resolution,
        map_info_.angular_resolution,
        range_,
        cm_.getAddressor());

    Vec d;
    search_list_.clear();
    for (d[0] = -range_; d[0] <= range_; d[0]++)
    {
      for (d[1] = -range_; d[1] <= range_; d[1]++)
      {
        if (d.sqlen() > range_ * range_)
          continue;
        for (d[2] = 0; d[2] < static_cast<int>(map_info_.angle); d[2]++)
        {
          search_list_.push_back(d);
        }
      }
    }
    search_list_rough_.clear();
    for (d[0] = -range_; d[0] <= range_; d[0]++)
    {
      for (d[1] = -range_; d[1] <= range_; d[1]++)
      {
        if (d.sqlen() > range_ * range_)
          continue;
        d[2] = 0;
        search_list_rough_.push_back(d);
      }
    }
    path_interpolator_.reset(map_info_.angular_resolution, range_);
  }

  void enableHysteresis(const bool enable)
  {
    hysteresis_ = enable;
  }
  float euclidCost(const Vec& v) const
  {
    float cost = euclidCostRough(v);

    for (int i = 2; i < 3; i++)
      cost += fabs(euclid_cost_coef_[i] * v[i]);

    return cost;
  }
  float euclidCostRough(const Vec& v) const
  {
    float cost = 0;
    for (int i = 0; i < 2; i++)
      cost += powf(euclid_cost_coef_[i] * v[i], 2.0);

    return sqrtf(cost);
  }
  float cost(
      const Vec& cur, const Vec& next, const Vec& start, const Vec& goal) const override
  {
    Vec d_raw = next - cur;
    d_raw.cycle(map_info_.angle);
    const Vec d = d_raw;
    float cost = euclidCost(d);

    if (d[0] == 0 && d[1] == 0)
    {
      // In-place turn
      int sum = 0;
      const int dir = d[2] < 0 ? -1 : 1;
      Vec pos = cur;
      for (int i = 0; i < abs(d[2]); i++)
      {
        pos[2] += dir;
        if (pos[2] < 0)
          pos[2] += map_info_.angle;
        else if (pos[2] >= static_cast<int>(map_info_.angle))
          pos[2] -= map_info_.angle;
        const auto c = cm_[pos];
        if (c > 99)
          return -1;
        sum += c;
      }

      const float cost =
          sum * map_info_.angular_resolution * euclid_cost_coef_[2] / euclid_cost_coef_[0] +
          sum * map_info_.angular_resolution * cc_.weight_costmap_turn_ / 100.0;
      // simplified from sum * map_info_.angular_resolution * abs(d[2]) * cc_.weight_costmap_turn_ / (100.0 * abs(d[2]))
      return cc_.in_place_turn_ + cost;
    }

    Vec d2;
    d2[0] = d[0] + range_;
    d2[1] = d[1] + range_;
    d2[2] = next[2];

    const Vecf motion = rot_cache_.getMotion(cur[2], d2);
    const Vecf motion_grid = motion * resolution_;

    if (lroundf(motion_grid[0]) == 0 && lroundf(motion_grid[1]) != 0)
    {
      // Not non-holonomic
      return -1;
    }

    if (fabs(motion[2]) >= 2.0 * M_PI / 4.0)
    {
      // Over 90 degree turn
      // must be separated into two curves
      return -1;
    }

    const float dist = motion.len();

    if (motion[0] < 0)
    {
      // Going backward
      cost *= 1.0 + cc_.weight_backward_;
    }

    if (d[2] == 0)
    {
      if (lroundf(motion_grid[0]) == 0)
        return -1;  // side slip
      const float aspect = motion[0] / motion[1];
      if (fabs(aspect) < cc_.angle_resolution_aspect_)
        return -1;  // large y offset

      // Go-straight
      int sum = 0, sum_hyst = 0;
      Vec d_index(d[0], d[1], next[2]);
      d_index.cycleUnsigned(map_info_.angle);

      const auto cache_page = motion_cache_.find(cur[2], d_index);
      if (cache_page == motion_cache_.end(cur[2]))
        return -1;
      const int num = cache_page->second.getMotion().size();
      for (const auto& pos_diff : cache_page->second.getMotion())
      {
        const Vec pos(
            cur[0] + pos_diff[0], cur[1] + pos_diff[1], pos_diff[2]);
        const auto c = cm_[pos];
        if (c > 99)
          return -1;
        sum += c;

        if (hysteresis_)
          sum_hyst += cm_hyst_[pos];
      }
      const float distf = cache_page->second.getDistance();
      cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / (100.0 * num);
      cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / (100.0 * num);
    }
    else
    {
      // Curve
      if (motion[0] * motion[1] * motion[2] < 0)
        return -1;

      if (d.sqlen() < 3 * 3)
        return -1;

      const std::pair<float, float>& radiuses = rot_cache_.getRadiuses(cur[2], d2);
      const float r1 = radiuses.first;
      const float r2 = radiuses.second;

      // curveture at the start_ pose and the end pose must be same
      if (fabs(r1 - r2) >= map_info_.linear_resolution * 1.5)
      {
        // Drifted
        return -1;
      }

      const float curv_radius = (r1 + r2) / 2;
      if (std::abs(curv_radius) < cc_.min_curve_raduis_)
        return -1;

      if (fabs(cc_.max_vel_ / r1) > cc_.max_ang_vel_)
      {
        const float vel = fabs(curv_radius) * cc_.max_ang_vel_;

        // Curve deceleration penalty
        cost += dist * fabs(vel / cc_.max_vel_) * cc_.weight_decel_;
      }

      {
        int sum = 0, sum_hyst = 0;
        Vec d_index(d[0], d[1], next[2]);
        d_index.cycleUnsigned(map_info_.angle);

        const auto cache_page = motion_cache_.find(cur[2], d_index);
        if (cache_page == motion_cache_.end(cur[2]))
          return -1;
        const int num = cache_page->second.getMotion().size();
        for (const auto& pos_diff : cache_page->second.getMotion())
        {
          const Vec pos(
              cur[0] + pos_diff[0], cur[1] + pos_diff[1], pos_diff[2]);
          const auto c = cm_[pos];
          if (c > 99)
            return -1;
          sum += c;

          if (hysteresis_)
            sum_hyst += cm_hyst_[pos];
        }
        const float distf = cache_page->second.getDistance();
        cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / (100.0 * num);
        cost += sum * map_info_.angular_resolution * abs(d[2]) * cc_.weight_costmap_turn_ / (100.0 * num);
        cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / (100.0 * num);
      }
    }

    return cost;
  }
  float costEstim(
      const Vec& cur, const Vec& goal) const override
  {
    Vec s2(cur[0], cur[1], 0);
    float cost = cost_estim_cache_[s2];
    if (cost == FLT_MAX)
      return FLT_MAX;

    int diff = cur[2] - goal[2];
    while (diff > static_cast<int>(map_info_.angle) / 2)
      diff -= static_cast<int>(map_info_.angle);
    while (diff < -static_cast<int>(map_info_.angle) / 2)
      diff += static_cast<int>(map_info_.angle);

    cost += euclid_cost_coef_[2] * abs(diff);

    return cost;
  }
  const std::vector<Vec>& searchGrids(
      const Vec& cur, const Vec& start, const Vec& goal) const override
  {
    const Vec ds = start - cur;
    if (ds.sqlen() < local_range_ * local_range_)
    {
      return search_list_;
    }
    return search_list_rough_;
  }
};

class GridAstarModel2D : public GridAstarModelBase<3, 2>
{
public:
  using Ptr = std::shared_ptr<GridAstarModel2D>;
  const GridAstarModel3D::ConstPtr base_;

  explicit GridAstarModel2D(const GridAstarModel3D::ConstPtr base)
    : base_(base)
  {
  }

  float cost(
      const Vec& cur, const Vec& next, const Vec& start, const Vec& goal) const override
  {
    Vec d = next - cur;
    d[2] = 0;
    float cost = base_->euclidCostRough(d);

    int sum = 0;
    const auto cache_page = base_->motion_cache_linear_.find(0, d);
    if (cache_page == base_->motion_cache_linear_.end(0))
      return -1;
    const int num = cache_page->second.getMotion().size();
    for (const auto& pos_diff : cache_page->second.getMotion())
    {
      const Vec pos(cur[0] + pos_diff[0], cur[1] + pos_diff[1], 0);
      const auto c = base_->cm_rough_[pos];
      if (c > 99)
        return -1;
      sum += c;
    }
    const float distf = cache_page->second.getDistance();
    cost += sum * base_->map_info_.linear_resolution *
            distf * base_->cc_.weight_costmap_ / (100.0 * num);

    return cost;
  }
  float costEstim(
      const Vec& cur, const Vec& goal) const override
  {
    const Vec d = goal - cur;
    const float cost = base_->euclidCostRough(d);

    return cost;
  }
  const std::vector<Vec>& searchGrids(
      const Vec& cur, const Vec& start, const Vec& goal) const override
  {
    return base_->search_list_rough_;
  }
};

#endif  // PLANNER_CSPACE_PLANNER_3D_GRID_ASTAR_MODEL_H
