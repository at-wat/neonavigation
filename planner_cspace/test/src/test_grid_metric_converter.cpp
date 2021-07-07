/*
 * Copyright (c) 2018, the neonavigation authors
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

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/planner_3d/grid_metric_converter.h>
#include <tf2/utils.h>

#include <gtest/gtest.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(GridMetricConverter, SinglePose)
{
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.linear_resolution = 0.1;
  map_info.angular_resolution = M_PI / 8;
  map_info.origin.position.x = 100.0;
  map_info.origin.position.y = 100.0;

  const float metric[][3] =
      {
          {100.01, 100.01, M_PI / 8},
          {100.01, 100.01, -M_PI / 8},
          {105.01, 102.09, M_PI / 2},
          {100.21, 102.51, -M_PI / 2},
          {99.99, 100.01, 0},
          {100.01, 100.01, 2.1 * M_PI / 8},
          {100.01, 100.01, 1.9 * M_PI / 8},
      };
  const int grid[][3] =
      {
          {0, 0, 1},
          {0, 0, -1},
          {50, 20, 4},
          {2, 25, -4},  // angle must not normalized
          {-1, 0, 0},   // x, y must be rounded downword
          {0, 0, 2},    // angle must be rounded to nearest integer
          {0, 0, 2}     // angle must be rounded to nearest integer
      };
  for (size_t i = 0; i < sizeof(grid) / sizeof(grid[0]); ++i)
  {
    // Convert metric to grid
    int output_grid[3];
    grid_metric_converter::metric2Grid(
        map_info,
        output_grid[0], output_grid[1], output_grid[2],
        metric[i][0], metric[i][1], metric[i][2]);
    for (size_t j = 0; j < 3; ++j)
      ASSERT_EQ(output_grid[j], grid[i][j]);

    // Convert grid to metric
    float output_metric[3];
    grid_metric_converter::grid2Metric(
        map_info,
        grid[i][0], grid[i][1], grid[i][2],
        output_metric[0], output_metric[1], output_metric[2]);
    for (size_t j = 0; j < 3; ++j)
      ASSERT_NEAR(output_metric[j], metric[i][j], map_info.linear_resolution / 2);
  }
}

TEST(GridMetricConverter, Path)
{
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.linear_resolution = 0.1;
  map_info.angle = 8;
  map_info.angular_resolution = M_PI / map_info.angle;
  map_info.origin.position.x = 100.0;
  map_info.origin.position.y = 100.0;

  const float metric_expected[][3] =
      {
          {100.15, 100.15, M_PI / 8},
          {100.15, 100.25, M_PI / 8},
          {100.25, 100.35, M_PI / 8},
          {100.35, 100.35, 2 * M_PI / 8},
      };
  const int grid[][3] =
      {
          {1, 1, 1},
          {1, 2, 1},
          {2, 3, 1},
          {3, 3, 2},
      };
  std::vector<CyclicVecFloat<3, 2>> path_grid;
  for (const auto& g : grid)
    path_grid.push_back(CyclicVecFloat<3, 2>(g[0], g[1], g[2]));

  nav_msgs::Path path;
  grid_metric_converter::grid2MetricPath(map_info, path_grid, path);
  size_t ref = 0;
  for (const geometry_msgs::PoseStamped& p : path.poses)
  {
    const double diff_dist = std::hypot(
        metric_expected[ref][0] - p.pose.position.x,
        metric_expected[ref][1] - p.pose.position.y);
    double diff_yaw = std::abs(tf2::getYaw(p.pose.orientation) - metric_expected[ref][2]);

    if (diff_yaw < -M_PI)
      diff_yaw += 2.0 * M_PI;
    else if (diff_yaw > M_PI)
      diff_yaw -= 2.0 * M_PI;

    ASSERT_LT(diff_dist, map_info.linear_resolution * 2);
    ASSERT_LT(std::abs(diff_yaw), map_info.angular_resolution * 2);

    if (diff_dist < map_info.linear_resolution / 2 &&
        std::abs(diff_yaw) < map_info.angular_resolution / 2)
    {
      ++ref;
      if (ref == sizeof(metric_expected) / sizeof(metric_expected[0]))
        return;
    }
  }
  // some reference points are not met
  ASSERT_TRUE(false);
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
