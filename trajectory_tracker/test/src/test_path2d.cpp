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
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nav_msgs/msg/path.hpp>
#include <trajectory_tracker/eigen_line.h>
#include <trajectory_tracker/path2d.h>
#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>

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

    {
      const auto it_local_goal = path.findLocalGoal(path.begin(), path.end(), true, true);
      ASSERT_EQ(it_local_goal, path.begin() + 5);
      ASSERT_EQ(path.findLocalGoal(it_local_goal, path.end(), true, true), path.end());
    }
    {
      const auto it_local_goal = path.findLocalGoal(path.begin(), path.end(), true, false);
      ASSERT_EQ(it_local_goal, path.begin() + 5);
      ASSERT_EQ(path.findLocalGoal(it_local_goal, path.end(), true, false), path.end());
    }
    // no switch back motion under (allow_switchback == false)
    ASSERT_EQ(path.findLocalGoal(path.begin(), path.end(), false, true), path.end());
  }
}

TEST(Path2D, LocalGoalTest)
{
  trajectory_tracker::Path2D path;
  Eigen::Vector2d p(0, 0);
  for (size_t i = 0; i <= 10; ++i)
  {
    const double yaw = M_PI / 2 / 10;
    p.x() = std::sin(i * yaw);
    p.y() = 1.0 - std::cos(i * yaw);
    path.push_back(trajectory_tracker::Pose2D(p, yaw, 0));
  }
  // Turn in-place and go back
  for (size_t i = 0; i <= 10; ++i)
  {
    path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(1.0 - i * 0.1, 1.0), 0, 0));
  }
  {
    const auto it_local_goal = path.findLocalGoal(path.begin(), path.end(), true, true);
    ASSERT_EQ(it_local_goal, path.begin() + 11);
    ASSERT_EQ(path.findLocalGoal(it_local_goal, path.end(), true, true), path.end());
  }
  {
    ASSERT_EQ(path.findLocalGoal(path.begin(), path.end(), true, false), path.end());
  }
}

TEST(Path2D, EnumerateLocalGoals)
{
  trajectory_tracker::Path2D path;
  const std::vector<float> yaws = {0.5, -0.2, 1.0};
  double x = 0.0;
  double y = 0.0;
  for (const float yaw : yaws)
  {
    for (size_t i = 0; i < 10; ++i)
    {
      path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(x, y), yaw, 1.0));
      x += std::cos(yaw) * 0.1;
      y += std::sin(yaw) * 0.1;
    }
    path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(x, y), yaw, 1.0));
  }
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(x, y), 0.3, 1.0));

  const auto local_goals = path.enumerateLocalGoals(path.begin(), path.end(), true);
  ASSERT_EQ(local_goals.size(), 3);
  ASSERT_EQ(local_goals.at(0) - path.begin(), 11);
  ASSERT_EQ(local_goals.at(1) - path.begin(), 22);
  ASSERT_EQ(local_goals.at(2) - path.begin(), 33);
}

TEST(Path2D, FindNearestWithDistance)
{
  trajectory_tracker::Path2D path;
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.5, 0.5), 0, 1));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.6, 0.5), 0, 1));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.7, 0.5), 0, 1));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.8, 0.6), M_PI / 4, 1));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.9, 0.7), M_PI / 4, 1));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.9, 0.8), M_PI / 2, 1));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.9, 0.9), M_PI / 2, 1));

  {
    // The nearest line is (0.8, 0.6) - (0.9, 0.7), and the nearest point on the line is (0.85, 0.65).
    const auto nearest_with_dist = path.findNearestWithDistance(path.begin(), path.end(), Eigen::Vector2d(1.0, 0.5));
    EXPECT_EQ(nearest_with_dist.first, path.begin() + 4);
    EXPECT_NEAR(nearest_with_dist.second, std::sqrt(std::pow(1.0 - 0.85, 2) + std::pow(0.5 - 0.65, 2)), 1.0e-6);
  }
  {
    // The nearest point is (0.7, 0.5).
    const auto nearest_with_dist =
        path.findNearestWithDistance(path.begin(), path.begin() + 3, Eigen::Vector2d(1.0, 0.5));
    EXPECT_EQ(nearest_with_dist.first, path.begin() + 2);
    EXPECT_NEAR(nearest_with_dist.second, std::sqrt(std::pow(1.0 - 0.7, 2) + std::pow(0.5 - 0.5, 2)), 1.0e-6);
  }
  {
    // Test edge cases
    const auto nearest_with_dist =
        path.findNearestWithDistance(path.begin() + 5, path.begin() + 5, Eigen::Vector2d(1.0, 0.5));
    EXPECT_EQ(nearest_with_dist.first, path.begin() + 5);
    EXPECT_NEAR(nearest_with_dist.second, std::sqrt(std::pow(1.0 - 0.9, 2) + std::pow(0.5 - 0.8, 2)), 1.0e-6);

    const auto invalid_result =
        path.findNearestWithDistance(path.end(), path.end(), Eigen::Vector2d(1.0, 0.5));
    EXPECT_EQ(invalid_result.first, path.end());
    EXPECT_EQ(invalid_result.second, std::numeric_limits<double>::max());
  }
}

TEST(Path2D, Conversions)
{
  nav_msgs::msg::Path path_msg_org;
  path_msg_org.poses.resize(8);
  path_msg_org.poses[0].pose.position.x = 0.0;
  path_msg_org.poses[0].pose.position.y = 0.0;
  path_msg_org.poses[0].pose.orientation.w = 1.0;
  path_msg_org.poses[1].pose.position.x = 1.0;  // Start of in-place turning
  path_msg_org.poses[1].pose.position.y = 0.0;
  path_msg_org.poses[1].pose.orientation.w = 1.0;
  path_msg_org.poses[2].pose.position.x = 1.0;
  path_msg_org.poses[2].pose.position.y = 0.0;
  path_msg_org.poses[2].pose.orientation.z = std::sin(M_PI / 8);
  path_msg_org.poses[2].pose.orientation.w = std::cos(M_PI / 8);
  path_msg_org.poses[3].pose.position.x = 1.0;  // End of in-place turning
  path_msg_org.poses[3].pose.position.y = 0.0;
  path_msg_org.poses[3].pose.orientation.z = std::sin(M_PI / 4);
  path_msg_org.poses[3].pose.orientation.w = std::cos(M_PI / 4);
  path_msg_org.poses[4].pose.position.x = 1.0;
  path_msg_org.poses[4].pose.position.y = 1.0;
  path_msg_org.poses[4].pose.orientation.z = std::sin(M_PI / 4);
  path_msg_org.poses[4].pose.orientation.w = std::cos(M_PI / 4);
  path_msg_org.poses[5].pose.position.x = 1.0;  // Start of in-place turning
  path_msg_org.poses[5].pose.position.y = 2.0;
  path_msg_org.poses[5].pose.orientation.z = std::sin(M_PI / 4);
  path_msg_org.poses[5].pose.orientation.w = std::cos(M_PI / 4);
  path_msg_org.poses[6].pose.position.x = 1.0;
  path_msg_org.poses[6].pose.position.y = 2.0;
  path_msg_org.poses[6].pose.orientation.z = std::sin(M_PI / 4 + M_PI / 6);
  path_msg_org.poses[6].pose.orientation.w = std::cos(M_PI / 4 + M_PI / 6);
  path_msg_org.poses[7].pose.position.x = 1.0;  // End of in-place turning
  path_msg_org.poses[7].pose.position.y = 2.0;
  path_msg_org.poses[7].pose.orientation.z = std::sin(M_PI / 4 + M_PI / 3);
  path_msg_org.poses[7].pose.orientation.w = std::cos(M_PI / 4 + M_PI / 3);

  trajectory_tracker::Path2D path;
  path.fromMsg(path_msg_org);

  const std::vector<size_t> expected_org_indexes = {0, 1, 3, 4, 5, 7};
  ASSERT_EQ(path.size(), 6);
  for (size_t i = 0; i < path.size(); ++i)
  {
    const size_t org_index = expected_org_indexes[i];
    EXPECT_EQ(path[i].pos_.x(), path_msg_org.poses[org_index].pose.position.x) << "i: " << i << " org: " << org_index;
    EXPECT_EQ(path[i].pos_.y(), path_msg_org.poses[org_index].pose.position.y) << "i: " << i << " org: " << org_index;
    EXPECT_NEAR(path[i].yaw_, tf2::getYaw(path_msg_org.poses[org_index].pose.orientation), 1.0e-6)
        << "i: " << i << " org: " << org_index;
    EXPECT_TRUE(std::isnan(path[i].velocity_));
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = rclcpp::Time(123.456);
  path.toMsg(path_msg);
  ASSERT_EQ(path_msg.poses.size(), 6);
  for (size_t i = 0; i < path.size(); ++i)
  {
    EXPECT_EQ(path[i].pos_.x(), path_msg.poses[i].pose.position.x) << "i: " << i;
    EXPECT_EQ(path[i].pos_.y(), path_msg.poses[i].pose.position.y) << "i: " << i;
    EXPECT_NEAR(path[i].yaw_, tf2::getYaw(path_msg.poses[i].pose.orientation), 1.0e-6) << "i: " << i;
    EXPECT_EQ(path_msg.poses[i].header.frame_id, path_msg.header.frame_id);
    EXPECT_EQ(path_msg.poses[i].header.stamp, path_msg.header.stamp);
  }

  trajectory_tracker_msgs::msg::PathWithVelocity path_with_vel_msg_org;
  path_with_vel_msg_org.poses.resize(path_msg_org.poses.size());
  for (size_t i = 0; i < path_msg_org.poses.size(); ++i)
  {
    path_with_vel_msg_org.poses[i].pose = path_msg_org.poses[i].pose;
    path_with_vel_msg_org.poses[i].linear_velocity.x = i * 0.1;
  }

  trajectory_tracker::Path2D path_with_vel;
  path_with_vel.fromMsg(path_with_vel_msg_org);

  ASSERT_EQ(path_with_vel.size(), 6);
  for (size_t i = 0; i < path_with_vel.size(); ++i)
  {
    const size_t org_index = expected_org_indexes[i];
    EXPECT_EQ(path_with_vel[i].pos_.x(), path_with_vel_msg_org.poses[org_index].pose.position.x)
        << "i: " << i << " org: " << org_index;
    EXPECT_EQ(path_with_vel[i].pos_.y(), path_with_vel_msg_org.poses[org_index].pose.position.y)
        << "i: " << i << " org: " << org_index;
    EXPECT_NEAR(path_with_vel[i].yaw_, tf2::getYaw(path_with_vel_msg_org.poses[org_index].pose.orientation), 1.0e-6)
        << "i: " << i << " org: " << org_index;
    EXPECT_NEAR(path_with_vel[i].velocity_, org_index * 0.1, 1.0e-6) << "i: " << i << " org: " << org_index;
  }

  trajectory_tracker_msgs::msg::PathWithVelocity path_with_vel_msg;
  path_with_vel_msg.header.frame_id = "map";
  path_with_vel_msg.header.stamp = rclcpp::Time(123.456);
  path_with_vel.toMsg(path_with_vel_msg);
  ASSERT_EQ(path_with_vel_msg.poses.size(), 6);
  for (size_t i = 0; i < path_with_vel.size(); ++i)
  {
    EXPECT_EQ(path_with_vel[i].pos_.x(), path_with_vel_msg.poses[i].pose.position.x) << "i: " << i;
    EXPECT_EQ(path_with_vel[i].pos_.y(), path_with_vel_msg.poses[i].pose.position.y) << "i: " << i;
    EXPECT_NEAR(path_with_vel[i].yaw_, tf2::getYaw(path_with_vel_msg.poses[i].pose.orientation), 1.0e-6)
        << "i: " << i;
    EXPECT_EQ(path_with_vel_msg.poses[i].header.frame_id, path_with_vel_msg.header.frame_id);
    EXPECT_EQ(path_with_vel_msg.poses[i].header.stamp, path_with_vel_msg.header.stamp);
  }
}

TEST(Path2D, EstimatedTimeOfArrivals)
{
  trajectory_tracker::Path2D path;
  // Straight
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.0, 0.0), 135.0 / 180.0 * M_PI, 0));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.1, -0.1), 135.0 / 180.0 * M_PI, 0));
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.2, -0.2), 135.0 / 180.0 * M_PI, 0));
  // In-place-turn
  path.push_back(trajectory_tracker::Pose2D(Eigen::Vector2d(0.2, -0.2), -M_PI / 2, 0));
  // Curve
  for (int i = 1; i <= 4; ++i)
  {
    const double angle = M_PI / 8 * i;
    path.push_back(trajectory_tracker::Pose2D(
        Eigen::Vector2d(0.2 * std::cos(angle), -0.2 + 0.2 * std::sin(angle)), -M_PI / 2 - angle, 0));
  }
  const double linear_speed = 0.5;
  const double angular_speed = M_PI;
  const std::vector<double> etas =
      path.getEstimatedTimeOfArrivals(path.begin(), path.end(), linear_speed, angular_speed, 1.0);
  ASSERT_EQ(etas.size(), 8);
  double expected_eta = 1.0;
  EXPECT_NEAR(etas[0], expected_eta, 1.0e-6);
  expected_eta += std::hypot(0.1, 0.1) / linear_speed;
  EXPECT_NEAR(etas[1], expected_eta, 1.0e-6);
  expected_eta += std::hypot(0.1, 0.1) / linear_speed;
  EXPECT_NEAR(etas[2], expected_eta, 1.0e-6);
  expected_eta += 135.0 / (180.0 / M_PI) / angular_speed;
  EXPECT_NEAR(etas[3], expected_eta, 1.0e-6);
  const double turn_dist = std::hypot(0.2 - 0.2 * std::cos(M_PI / 8), 0.2 * std::sin(M_PI / 8));
  for (size_t p = 4; p < etas.size(); ++p)
  {
    EXPECT_NEAR(etas[p], expected_eta + (p - 3) * turn_dist / linear_speed, 1.0e-6);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
