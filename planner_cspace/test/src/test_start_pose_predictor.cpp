/*
 * Copyright (c) 2023, the neonavigation authors
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

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <list>
#include <string>
#include <unordered_map>
#include <utility>

#include <nav_msgs/Path.h>
#include <planner_cspace/planner_3d/path_interpolator.h>
#include <planner_cspace/planner_3d/start_pose_predictor.h>

#include <ros/ros.h>

namespace planner_cspace
{
namespace planner_3d
{
class StartPosePredictorTester : public ::testing::Test
{
protected:
  void SetUp() final
  {
    config_ =
        {
            .prediction_sec_ = 0.5,
            .switch_back_prediction_sec_ = 2.0,
            .dist_stop_ = 0.2,
            .lin_vel_ = 1.5,
            .ang_vel_ = M_PI / 2,
        };
    predictor_.setConfig(config_);

    map_info_.width = 11;
    map_info_.height = 11;
    map_info_.angle = 4;
    map_info_.origin.position.x = -5.5;
    map_info_.origin.position.y = -5.5;
    map_info_.origin.orientation.w = 1.0;
    map_info_.linear_resolution = 1.0;
    map_info_.angular_resolution = 2 * M_PI / map_info_.angle;
    interpolator_.reset(map_info_.linear_resolution, 1);

    const GridAstar<3, 2>::Vec size3d(
        static_cast<int>(map_info_.width),
        static_cast<int>(map_info_.height),
        static_cast<int>(map_info_.angle));
    cm_.reset(size3d);
    for (int a = 0; a < static_cast<int>(map_info_.angle); ++a)
    {
      for (int y = 0; y < static_cast<int>(map_info_.height); ++y)
      {
        for (int x = 0; x < static_cast<int>(map_info_.width); ++x)
        {
          cm_[GridAstar<3, 2>::Vec(x, y, a)] = 0;
        }
      }
    }

    const std::list<StartPosePredictor::Astar::Vec> path_grid_straight =
        {
            StartPosePredictor::Astar::Vec(5, 5, 0),  // Start is (0.0, 0.0, 0.0)
            StartPosePredictor::Astar::Vec(6, 5, 0),
            StartPosePredictor::Astar::Vec(7, 5, 0),  // Moves 2 meters ahead
        };
    paths_["straight"] = std::make_pair(path_grid_straight, convertToMetricPath(path_grid_straight));

    const std::list<StartPosePredictor::Astar::Vec> path_grid_switch_back =
        {
            StartPosePredictor::Astar::Vec(5, 5, 0),  // Start is (0.0, 0.0, 0.0)
            StartPosePredictor::Astar::Vec(6, 5, 0),
            StartPosePredictor::Astar::Vec(7, 5, 0),
            StartPosePredictor::Astar::Vec(8, 6, 1),   // Curv from (2.0, 0.0, 0.0) to (3.0, 1.0, pi/2)
            StartPosePredictor::Astar::Vec(9, 5, 2),   // Curv from (3.0, 1.0, 0.0) to (4.0, 0.0, pi)
            StartPosePredictor::Astar::Vec(10, 5, 2),  // goal is (5.0, 0.0, pi)
        };
    paths_["switch_back"] = std::make_pair(path_grid_switch_back, convertToMetricPath(path_grid_switch_back));
  }

  nav_msgs::Path convertToMetricPath(const std::list<GridAstar<3, 2>::Vec>& path_grid) const
  {
    const auto path_interpolated = interpolator_.interpolate(path_grid, 0.1, 10);

    nav_msgs::Path result_path;
    result_path.header.frame_id = "map";
    for (const auto& pose_grid : path_interpolated)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      float x, y, yaw;
      grid_metric_converter::grid2Metric(map_info_, pose_grid[0], pose_grid[1], pose_grid[2], x, y, yaw);
      pose.pose = getPose(x, y, yaw);
      result_path.poses.push_back(pose);
    }
    return result_path;
  }

  geometry_msgs::Pose getPose(const double x, const double y, const double yaw) const
  {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.z = std::sin(yaw / 2);
    pose.orientation.w = std::cos(yaw / 2);
    return pose;
  }

  StartPosePredictor predictor_;
  StartPosePredictor::Config config_;
  GridAstar<3, 2>::Gridmap<char, 0x40> cm_;
  costmap_cspace_msgs::MapMetaData3D map_info_;
  PathInterpolator interpolator_;
  std::unordered_map<std::string, std::pair<std::list<StartPosePredictor::Astar::Vec>, nav_msgs::Path>> paths_;
};

TEST_F(StartPosePredictorTester, EdgeCases)
{
  StartPosePredictor::Astar::Vec start_grid;
  // Empty path
  EXPECT_FALSE(predictor_.process(getPose(0.0, 0.0, 0.0), cm_, map_info_, nav_msgs::Path(), start_grid));

  // Far from path
  EXPECT_FALSE(predictor_.process(getPose(0.0, 0.5, 0.0), cm_, map_info_,
                                  paths_["straight"].second, start_grid));

  // Only the last pose is remaining
  nav_msgs::Path rotating_path;
  rotating_path.header.frame_id = "map";
  rotating_path.poses.resize(1);
  rotating_path.poses[0].header.frame_id = "map";
  rotating_path.poses[0].pose = getPose(0.01, 1.01, M_PI / 2);

  EXPECT_TRUE(predictor_.process(getPose(0.0, 1.0, 1.3), cm_, map_info_, rotating_path, start_grid));
  EXPECT_EQ(StartPosePredictor::Astar::Vec(5, 6, 1), start_grid);
  EXPECT_TRUE(predictor_.getPreservedPath().poses.empty());
}

TEST_F(StartPosePredictorTester, Normal)
{
  {
    StartPosePredictor::Astar::Vec start_grid;
    EXPECT_TRUE(predictor_.process(getPose(0.05, 0.05, 0.0), cm_, map_info_, paths_["straight"].second, start_grid));
    EXPECT_EQ(StartPosePredictor::Astar::Vec(6, 5, 0), start_grid);
    const auto preserved_path = predictor_.getPreservedPath();
    // Only (0, 0, 0) is removed as the nearest is the first line strip, that is from (0, 0, 0) to (0.1, 0, 0).
    ASSERT_EQ(preserved_path.poses.size(), 9);
    for (size_t i = 0; i < preserved_path.poses.size(); ++i)
    {
      const auto& path_pose = preserved_path.poses[i].pose;
      EXPECT_NEAR(path_pose.position.x, 0.1f * (i + 1), 1.0e-6);
      EXPECT_NEAR(path_pose.position.y, 0.0, 1.0e-6);
      EXPECT_NEAR(tf2::getYaw(path_pose.orientation), 0.0, 1.0e-6);
    }
    EXPECT_NEAR(predictor_.getPreservedPathLength(), 0.9, 1.0e-6);
  }
  {
    StartPosePredictor::Astar::Vec start_grid;
    EXPECT_TRUE(predictor_.process(getPose(-0.05, 0.05, 0.0), cm_, map_info_, paths_["straight"].second, start_grid));
    EXPECT_EQ(StartPosePredictor::Astar::Vec(6, 5, 0), start_grid);
    const auto preserved_path = predictor_.getPreservedPath();
    // No points are removed as the robot have not reached the first line strip.
    ASSERT_EQ(preserved_path.poses.size(), 10);
    for (size_t i = 0; i < preserved_path.poses.size(); ++i)
    {
      const auto& path_pose = preserved_path.poses[i].pose;
      EXPECT_NEAR(path_pose.position.x, 0.1f * i, 1.0e-6);
      EXPECT_NEAR(path_pose.position.y, 0.0, 1.0e-6);
      EXPECT_NEAR(tf2::getYaw(path_pose.orientation), 0.0, 1.0e-6);
    }
    EXPECT_NEAR(predictor_.getPreservedPathLength(), 1.0, 1.0e-6);
  }

  {
    // The robot will collide with an obstacle during the planning.
    cm_[GridAstar<3, 2>::Vec(6, 5, 0)] = 100;

    StartPosePredictor::Astar::Vec start_grid;
    EXPECT_FALSE(predictor_.process(getPose(0.05, 0.05, 0.0), cm_, map_info_, paths_["straight"].second, start_grid));
    EXPECT_TRUE(predictor_.getPreservedPath().poses.empty());
    EXPECT_EQ(predictor_.getPreservedPathLength(), 0.0);
  }
}

TEST_F(StartPosePredictorTester, SwitchBack)
{
  {
    StartPosePredictor::Astar::Vec start_grid;
    EXPECT_TRUE(predictor_.process(getPose(0.25, 0.0, 0.0), cm_, map_info_, paths_["switch_back"].second, start_grid));
    // The robot will not reach the switch back within 2.0 seconds, and the start grid will be on the first line.
    EXPECT_EQ(StartPosePredictor::Astar::Vec(7, 5, 0), start_grid);
    const auto preserved_path = predictor_.getPreservedPath();
    ASSERT_EQ(preserved_path.poses.size(), 17);
    for (size_t i = 0; i < preserved_path.poses.size(); ++i)
    {
      const auto& path_pose = preserved_path.poses[i].pose;
      const auto& expected_path_pose = paths_["switch_back"].second.poses[i + 3].pose;
      EXPECT_NEAR(path_pose.position.x, expected_path_pose.position.x, 1.0e-6);
      EXPECT_NEAR(path_pose.position.y, expected_path_pose.position.y, 1.0e-6);
      EXPECT_NEAR(tf2::getYaw(path_pose.orientation), tf2::getYaw(expected_path_pose.orientation), 1.0e-6);
    }
  }

  {
    StartPosePredictor::Astar::Vec start_grid;
    EXPECT_TRUE(predictor_.process(getPose(0.55, 0.0, 0.0), cm_, map_info_, paths_["switch_back"].second, start_grid));
    // The robot will reach the switch back within 2.0 seconds, and the start grid is the switch back point.
    EXPECT_EQ(StartPosePredictor::Astar::Vec(8, 6, 1), start_grid);
    const auto preserved_path = predictor_.getPreservedPath();
    // A part of the first forward, the second forward, and the first turn.
    ASSERT_EQ(preserved_path.poses.size(), 5 + 10 + 13);
    for (size_t i = 0; i < preserved_path.poses.size(); ++i)
    {
      const auto& path_pose = preserved_path.poses[i].pose;
      const auto& expected_path_pose = paths_["switch_back"].second.poses[i + 6].pose;
      EXPECT_NEAR(path_pose.position.x, expected_path_pose.position.x, 1.0e-6);
      EXPECT_NEAR(path_pose.position.y, expected_path_pose.position.y, 1.0e-6);
      EXPECT_NEAR(tf2::getYaw(path_pose.orientation), tf2::getYaw(expected_path_pose.orientation), 1.0e-6);
    }
  }
}

}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
