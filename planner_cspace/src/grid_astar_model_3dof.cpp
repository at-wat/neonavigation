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
#include <limits>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace_msgs/MapMetaData3D.h>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>
#include <planner_cspace/planner_3d/motion_cache.h>
#include <planner_cspace/planner_3d/motion_primitive_builder.h>
#include <planner_cspace/planner_3d/path_interpolator.h>
#include <planner_cspace/planner_3d/rotation_cache.h>

namespace planner_cspace
{
namespace planner_3d
{
GridAstarModel3D::GridAstarModel3D(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    const Vecf& euclid_cost_coef,
    const int local_range,
    BlockMemGridmapBase<float, 3, 2>& cost_estim_cache,
    BlockMemGridmapBase<char, 3, 2>& cm,
    BlockMemGridmapBase<char, 3, 2>& cm_hyst,
    BlockMemGridmapBase<char, 3, 2>& cm_rough,
    const CostCoeff& cc,
    const int range)
  : hysteresis_(false)
  , map_info_(map_info)
  , euclid_cost_coef_(euclid_cost_coef)
  , resolution_(
        1.0f / map_info.linear_resolution,
        1.0f / map_info.linear_resolution,
        1.0f / map_info.angular_resolution)
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

  // Make boundary check threshold
  min_boundary_ = motion_cache_.getMaxRange();
  max_boundary_ =
      Vec(static_cast<int>(map_info_.width),
          static_cast<int>(map_info_.height),
          static_cast<int>(map_info_.angle)) -
      min_boundary_;
  ROS_INFO("x:%d, y:%d grids around the boundary is ignored on path search", min_boundary_[0], min_boundary_[1]);

  createEuclidCostCache();

  motion_primitives_ = MotionPrimitiveBuilder::build(map_info_, cc_, range_);
  search_list_rough_.clear();
  Vec d;
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

void GridAstarModel3D::enableHysteresis(const bool enable)
{
  hysteresis_ = enable;
}
void GridAstarModel3D::createEuclidCostCache()
{
  for (int rootsum = 0;
       rootsum < static_cast<int>(euclid_cost_lin_cache_.size()); ++rootsum)
  {
    euclid_cost_lin_cache_[rootsum] = std::sqrt(rootsum) * euclid_cost_coef_[0];
  }
}
float GridAstarModel3D::euclidCost(const Vec& v) const
{
  float cost = euclidCostRough(v);

  int angle = v[2];
  while (angle > static_cast<int>(map_info_.angle) / 2)
    angle -= static_cast<int>(map_info_.angle);
  while (angle < -static_cast<int>(map_info_.angle) / 2)
    angle += static_cast<int>(map_info_.angle);
  cost += std::abs(euclid_cost_coef_[2] * angle);
  return cost;
}
float GridAstarModel3D::euclidCostRough(const Vec& v) const
{
  int rootsum = 0;
  for (int i = 0; i < 2; ++i)
    rootsum += v[i] * v[i];

  if (rootsum < static_cast<int>(euclid_cost_lin_cache_.size()))
    return euclid_cost_lin_cache_[rootsum];

  return std::sqrt(rootsum) * euclid_cost_coef_[0];
}
float GridAstarModel3D::cost(
    const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const
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
    for (int i = 0; i < std::abs(d[2]); i++)
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

    cost +=
        sum * map_info_.angular_resolution * euclid_cost_coef_[2] / euclid_cost_coef_[0] +
        sum * map_info_.angular_resolution * cc_.weight_costmap_turn_ / 100.0;
    // simplified from sum * map_info_.angular_resolution * abs(d[2]) * cc_.weight_costmap_turn_ / (100.0 * abs(d[2]))
    return cc_.in_place_turn_ + cost;
  }

  const Vec d2(d[0] + range_, d[1] + range_, next[2]);
  const Vecf motion = rot_cache_.getMotion(cur[2], d2);
  const float dist = motion.len();

  if (motion[0] < 0)
  {
    // Going backward
    cost *= 1.0 + cc_.weight_backward_;
  }

  if (d[2] == 0)
  {
    const float aspect = motion[0] / motion[1];
    cost += euclid_cost_coef_[2] * std::abs(1.0 / aspect) * map_info_.angular_resolution / (M_PI * 2.0);

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
    const std::pair<float, float>& radiuses = rot_cache_.getRadiuses(cur[2], d2);
    const float r1 = radiuses.first;
    const float r2 = radiuses.second;
    const float curv_radius = (r1 + r2) / 2;

    // Ignore boundary
    if (cur[0] < min_boundary_[0] || cur[1] < min_boundary_[1] ||
        cur[0] >= max_boundary_[0] || cur[1] >= max_boundary_[1])
      return -1;

    if (std::abs(cc_.max_vel_ / r1) > cc_.max_ang_vel_)
    {
      const float vel = std::abs(curv_radius) * cc_.max_ang_vel_;
      // Curve deceleration penalty
      cost += dist * std::abs(vel / cc_.max_vel_) * cc_.weight_decel_;
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
      cost += sum * map_info_.angular_resolution * std::abs(d[2]) * cc_.weight_costmap_turn_ / (100.0 * num);
      cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / (100.0 * num);
    }
  }

  return cost;
}
float GridAstarModel3D::costEstim(
    const Vec& cur, const Vec& goal) const
{
  Vec s2(cur[0], cur[1], 0);
  float cost = cost_estim_cache_[s2];
  if (cost == std::numeric_limits<float>::max())
    return std::numeric_limits<float>::max();

  int diff = cur[2] - goal[2];
  while (diff > static_cast<int>(map_info_.angle) / 2)
    diff -= static_cast<int>(map_info_.angle);
  while (diff < -static_cast<int>(map_info_.angle) / 2)
    diff += static_cast<int>(map_info_.angle);

  cost += euclid_cost_coef_[2] * std::abs(diff);

  return cost;
}
const std::vector<GridAstarModel3D::Vec>& GridAstarModel3D::searchGrids(
    const Vec& p,
    const std::vector<VecWithCost>& ss,
    const Vec& es) const
{
  const float local_range_sq = local_range_ * local_range_;
  for (const VecWithCost& s : ss)
  {
    const Vec ds = s.v_ - p;

    if (ds.sqlen() < local_range_sq)
    {
      return motion_primitives_[p[2]];
    }
  }
  return search_list_rough_;
}

float GridAstarModel2D::cost(
    const Vec& cur, const Vec& next, const std::vector<VecWithCost>& start, const Vec& goal) const
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
float GridAstarModel2D::costEstim(
    const Vec& cur, const Vec& goal) const
{
  const Vec d = goal - cur;
  const float cost = base_->euclidCostRough(d);

  return cost;
}
const std::vector<GridAstarModel3D::Vec>& GridAstarModel2D::searchGrids(
    const Vec& cur, const std::vector<VecWithCost>& start, const Vec& goal) const
{
  return base_->search_list_rough_;
}
}  // namespace planner_3d
}  // namespace planner_cspace
