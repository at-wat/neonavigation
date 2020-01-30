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

#include <algorithm>
#include <cstddef>
#include <string>

#include <gtest/gtest.h>

#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>

namespace
{
double getRemainedDistance(const trajectory_tracker::Path2D& path, const Eigen::Vector2d& p)
{
  const auto nearest = path.findNearest(path.begin(), path.end(), p);
  const Eigen::Vector2d pos_on_line =
      trajectory_tracker::projection2d((nearest - 1)->pos_, nearest->pos_, p);
  return path.remainedDistance(path.begin(), nearest, path.end(), pos_on_line);
}
}  // namespace

TEST(Path2D, RemainedDistance)
{
  trajectory_tracker::Path2D path_full;
  for (double x = 0.0; x < 10.0 - 1e-3; x += 0.2)
    path_full.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(x, 0), 0, 1));
  path_full.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(10.0, 0), 0, 1));

  // Single pose path means orientation control mode. More than two poses should be line following.
  for (size_t l = 2; l < path_full.size(); ++l)
  {
    trajectory_tracker::Path2D path;
    path.resize(l);
    std::copy(path_full.end() - l, path_full.end(), path.begin());

    std::string err_msg = "failed for " + std::to_string(l) + " pose(s) path";

    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(0, 0)), 10.0, 1e-2) << err_msg;

    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(8.25, 0)), 1.75, 1e-2) << err_msg;
    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(10.25, 0)), -0.25, 1e-2) << err_msg;

    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(8.25, 0.1)), 1.75, 1e-2) << err_msg;
    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(8.25, -0.1)), 1.75, 1e-2) << err_msg;
    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(10.25, 0.1)), -0.25, 1e-2) << err_msg;
    ASSERT_NEAR(getRemainedDistance(path, Eigen::Vector2d(10.25, -0.1)), -0.25, 1e-2) << err_msg;
  }
}

TEST(Path2D, Curvature)
{
  for (float c = 1.0; c < 4.0; c += 0.4)
  {
    trajectory_tracker::Path2D path;
    for (double a = 0; a < 1.57; a += 0.1)
      path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(std::cos(a), std::sin(a)) * c, a + M_PI / 2, 1));
    ASSERT_NEAR(path.getCurvature(path.begin(), path.end(), path[0].pos_, 10.0), 1.0 / c, 1e-2);
  }

  for (float c = 1.0; c < 4.0; c += 0.4)
  {
    trajectory_tracker::Path2D path;
    for (double a = 0; a < 0.2; a += 0.1)
      path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(std::cos(a), std::sin(a)) * c, a + M_PI / 2, 1));
    ASSERT_NEAR(path.getCurvature(path.begin(), path.end(), path[0].pos_, 10.0), 1.0 / c, 1e-2);
  }
}

TEST(Path2D, LocalGoalWithoutSwitchBack)
{
  for (int yaw_i = -1; yaw_i <= 1; ++yaw_i)
  {
    const double yaw_diff = yaw_i * 0.1;

    trajectory_tracker::Path2D path;
    Eigen::Vector2d p(0, 0);
    double yaw(0);
    for (int i = 0; i < 10; ++i)
    {
      p -= Eigen::Vector2d(std::cos(yaw), std::sin(yaw)) * 0.1;
      yaw += yaw_diff;
      path.push_back(trajectory_tracker::Pose2D(p, yaw, 1));
    }
    ASSERT_EQ(path.findLocalGoal(path.begin(), path.end(), true), path.end());
    ASSERT_EQ(path.findLocalGoal(path.begin(), path.end(), false), path.end());
  }
}

TEST(Path2D, LocalGoalWithSwitchBack)
{
  for (int yaw_i = -1; yaw_i <= 1; ++yaw_i)
  {
    const double yaw_diff = yaw_i * 0.1;

    trajectory_tracker::Path2D path;
    Eigen::Vector2d p(0, 0);
    double yaw(0);
    for (int i = 0; i < 5; ++i)
    {
      p -= Eigen::Vector2d(std::cos(yaw), std::sin(yaw)) * 0.1;
      yaw += yaw_diff;
      path.push_back(trajectory_tracker::Pose2D(p, yaw, 1));
    }
    for (int i = 0; i < 5; ++i)
    {
      p += Eigen::Vector2d(std::cos(yaw), std::sin(yaw)) * 0.1;
      yaw += yaw_diff;
      path.push_back(trajectory_tracker::Pose2D(p, yaw, 1));
    }
    const auto it_local_goal = path.findLocalGoal(path.begin(), path.end(), true);
    ASSERT_EQ(it_local_goal, path.begin() + 5);
    ASSERT_EQ(path.findLocalGoal(it_local_goal, path.end(), true), path.end());

    // no switch back motion under (allow_switchback == false)
    ASSERT_EQ(path.findLocalGoal(path.begin(), path.end(), false), path.end());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
