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

#include <list>
#include <vector>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>

#include <gtest/gtest.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(GridAstarModel3D, Cost)
{
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.width = 100;
  map_info.height = 100;
  map_info.angle = 16;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI * 2 / map_info.angle;
  BlockMemGridmap<char, 3, 2, 0x40> cm(GridAstarModel3D::Vec(100, 100, 16));
  BlockMemGridmap<float, 3, 2> cost_estim_cache(GridAstarModel3D::Vec(100, 100, 16));
  cm.clear(0);
  cost_estim_cache.clear(0.0);
  CostCoeff cc;
  cc.weight_decel_ = 0.1;
  cc.weight_backward_ = 0.1;
  cc.weight_ang_vel_ = 1.0;
  cc.weight_costmap_ = 0.1;
  cc.weight_costmap_turn_ = 0.1;
  cc.weight_remembered_ = 0.0;
  cc.weight_hysteresis_ = 0.0;
  cc.in_place_turn_ = 0.0;
  cc.hysteresis_max_dist_ = 0.0;
  cc.hysteresis_expand_ = 0.0;
  cc.min_curve_radius_ = 0.0;
  cc.max_vel_ = 1.0;
  cc.max_ang_vel_ = 1.0;
  cc.angle_resolution_aspect_ = 1.0;

  GridAstarModel3D model(
      map_info,
      GridAstarModel3D::Vecf(1.0f, 1.0f, 0.1f),
      100.0,
      cost_estim_cache, cm, cm, cm,
      cc, 10);

  const GridAstarModel3D::Vec start(54, 50, 8);
  const GridAstarModel3D::Vec goal_straight(50, 50, 8);
  const GridAstarModel3D::Vec goal_curve(50, 50, 7);
  const GridAstarModel3D::Vec goal_drift(50, 51, 8);
  const GridAstarModel3D::Vec goal_curve_drift(50, 51, 7);
  const GridAstarModel3D::Vec goal_curve_sign_err(50, 51, 9);
  std::vector<GridAstarModel3D::VecWithCost> starts;
  starts.emplace_back(start);

  const float c_straight = model.cost(start, goal_straight, starts, goal_straight);
  const float c_curve = model.cost(start, goal_curve, starts, goal_curve);
  const float c_drift = model.cost(start, goal_drift, starts, goal_drift);
  const float c_drift_curve = model.cost(start, goal_curve_drift, starts, goal_curve_drift);

  EXPECT_LT(c_straight, c_curve);
  EXPECT_LT(c_straight, c_drift);
  EXPECT_LT(c_straight, c_drift_curve);
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
