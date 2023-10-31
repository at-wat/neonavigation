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

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <dynamic_reconfigure/client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <planner_cspace/Planner3DConfig.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <planner_cspace/action_test_base.h>

class DynamicParameterChangeTest
  : public ActionTestBase<move_base_msgs::MoveBaseAction, ACTION_TOPIC_MOVE_BASE>
{
public:
  void SetUp() final
  {
    path_ = nullptr;
    planner_3d_client_.reset(
        new dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>("/planner_3d/"));
    sub_path_ = node_.subscribe("path", 1, &DynamicParameterChangeTest::cbPath, this);
    pub_map_overlay_ = node_.advertise<nav_msgs::OccupancyGrid>("map_overlay", 1, true);
    pub_odom_ = node_.advertise<nav_msgs::Odometry>("odom", 1, true);

    map_overlay_.header.frame_id = "map";
    map_overlay_.info.resolution = 0.1;
    map_overlay_.info.width = 32;
    map_overlay_.info.height = 32;
    map_overlay_.info.origin.position.x = 0.0;
    map_overlay_.info.origin.position.y = 0.0;
    map_overlay_.info.origin.position.z = 0.0;
    map_overlay_.info.origin.orientation.x = 0.0;
    map_overlay_.info.origin.orientation.y = 0.0;
    map_overlay_.info.origin.orientation.z = 0.0;
    map_overlay_.info.origin.orientation.w = 1.0;
    map_overlay_.data.resize(map_overlay_.info.width * map_overlay_.info.height, 0);
    publishMapAndRobot(0, 0, 0);
    ActionTestBase<move_base_msgs::MoveBaseAction, ACTION_TOPIC_MOVE_BASE>::SetUp();

    default_config_ = planner_cspace::Planner3DConfig::__getDefault__();
    default_config_.max_retry_num = 5;
    default_config_.tolerance_range = 0.0;
    default_config_.temporary_escape = false;
    default_config_.goal_tolerance_lin = 0.05;
    default_config_.cost_in_place_turn = 3.0;
    default_config_.min_curve_radius = 0.4;
    default_config_.max_vel = 0.3;
    default_config_.max_ang_vel = 1.0;
    ASSERT_TRUE(planner_3d_client_->setConfiguration(default_config_));
  }

protected:
  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = msg;
    ++path_received_count_;
    last_path_received_time_ = ros::Time::now();
  }

  move_base_msgs::MoveBaseGoal CreateGoalInFree()
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 1.25;
    goal.target_pose.pose.position.y = 1.05;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = std::sin(M_PI / 4);
    goal.target_pose.pose.orientation.w = std::cos(M_PI / 4);
    return goal;
  }

  bool isPathIncludingCurves() const
  {
    for (size_t i = 0; i < path_->poses.size() - 1; ++i)
    {
      const auto& pose1 = path_->poses[i];
      const auto& pose2 = path_->poses[i + 1];
      const double distance = std::hypot(pose1.pose.position.x - pose2.pose.position.x,
                                         pose1.pose.position.y - pose2.pose.position.y);
      const double yaw_diff = std::abs(tf2::getYaw(pose1.pose.orientation) - tf2::getYaw(pose2.pose.orientation));
      if (distance > 1.0e-3 && yaw_diff > 1.0e-3)
      {
        return true;
      }
    }
    return false;
  }

  ::testing::AssertionResult comparePath(const nav_msgs::Path& path1, const nav_msgs::Path& path2)
  {
    if (path1.poses.size() != path2.poses.size())
    {
      return ::testing::AssertionFailure()
             << "Path size different: " << path1.poses.size() << " != " << path2.poses.size();
    }
    for (size_t i = 0; i < path1.poses.size(); ++i)
    {
      const geometry_msgs::Point& pos1 = path1.poses[i].pose.position;
      const geometry_msgs::Point& pos2 = path2.poses[i].pose.position;
      if (std::abs(pos1.x - pos2.x) > 1.0e-6)
      {
        return ::testing::AssertionFailure() << "X different at #" << i << ": " << pos1.x << " != " << pos2.x;
      }
      if (std::abs(pos1.y - pos2.y) > 1.0e-6)
      {
        return ::testing::AssertionFailure() << "Y different at #" << i << ": " << pos1.y << " != " << pos2.y;
      }
      const double yaw1 = tf2::getYaw(path1.poses[i].pose.orientation);
      const double yaw2 = tf2::getYaw(path2.poses[i].pose.orientation);
      if (std::abs(yaw1 - yaw2) > 1.0e-6)
      {
        return ::testing::AssertionFailure() << "Yaw different at #" << i << ": " << yaw1 << " != " << yaw2;
      }
    }
    return ::testing::AssertionSuccess();
  }

  void sendGoalAndWaitForPath()
  {
    path_ = nullptr;
    const ros::Time start_time = ros::Time::now();
    ros::Time deadline = start_time + ros::Duration(1.0);
    move_base_->sendGoal(CreateGoalInFree());
    while (ros::ok())
    {
      if (path_ && (path_->header.stamp > start_time) && (path_->poses.size() > 0))
      {
        break;
      }
      ASSERT_LT(ros::Time::now(), deadline)
          << "Faile to plan:" << move_base_->getState().toString() << statusString();
      ros::spinOnce();
    }
  }

  void publishMapAndRobot(const double x, const double y, const double yaw)
  {
    const ros::Time current_time = ros::Time::now();
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = current_time;
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation = tf2::toMsg(tf2::Vector3(x, y, 0.0));
    trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    tfb_.sendTransform(trans);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = current_time;
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    pub_odom_.publish(odom);

    pub_map_overlay_.publish(map_overlay_);
  }

  double getPathHz(const ros::Duration costmap_publishing_interval)
  {
    publishMapAndRobot(2.55, 0.45, M_PI);
    ros::Duration(0.3).sleep();
    move_base_->sendGoal(CreateGoalInFree());
    while (ros::ok() && (move_base_->getState() != actionlib::SimpleClientGoalState::ACTIVE))
    {
      ros::spinOnce();
    }

    last_path_received_time_ = ros::Time();
    publishMapAndRobot(2.55, 0.45, M_PI);
    ros::Time last_costmap_publishing_time = ros::Time::now();
    ros::Rate r(100);
    while (ros::ok() && (last_path_received_time_ == ros::Time()))
    {
      if ((ros::Time::now() - last_costmap_publishing_time) > costmap_publishing_interval)
      {
        publishMapAndRobot(2.55, 0.45, M_PI);
        last_costmap_publishing_time = ros::Time::now();
      };
      ros::spinOnce();
      r.sleep();
    }
    const ros::Time initial_path_received_time_ = last_path_received_time_;
    const int prev_path_received_count = path_received_count_;
    while (ros::ok() && (path_received_count_ < prev_path_received_count + 10))
    {
      if ((ros::Time::now() - last_costmap_publishing_time) > costmap_publishing_interval)
      {
        publishMapAndRobot(2.55, 0.45, M_PI);
        last_costmap_publishing_time = ros::Time::now();
      }
      ros::spinOnce();
      r.sleep();
    }
    const double elapesed_time_sec = (last_path_received_time_ - initial_path_received_time_).toSec();
    const double hz = (path_received_count_ - prev_path_received_count) / elapesed_time_sec;
    return hz;
  }

  tf2_ros::TransformBroadcaster tfb_;
  ros::Subscriber sub_path_;
  nav_msgs::Path::ConstPtr path_;
  std::unique_ptr<dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>> planner_3d_client_;
  ros::Publisher pub_map_overlay_;
  ros::Publisher pub_odom_;
  nav_msgs::OccupancyGrid map_overlay_;
  planner_cspace::Planner3DConfig default_config_;
  int path_received_count_;
  ros::Time last_path_received_time_;
};

TEST_F(DynamicParameterChangeTest, DisableCurves)
{
  publishMapAndRobot(2.55, 0.45, M_PI);
  ros::Duration(0.5).sleep();

  sendGoalAndWaitForPath();
  // The default path is including curves.
  EXPECT_TRUE(isPathIncludingCurves());

  planner_cspace::Planner3DConfig config = default_config_;
  // Large min_curve_radius disables curves.
  config.min_curve_radius = 10.0;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  sendGoalAndWaitForPath();
  // The default path is including curves.
  EXPECT_FALSE(isPathIncludingCurves());
}

TEST_F(DynamicParameterChangeTest, StartPosePrediction)
{
  publishMapAndRobot(1.65, 0.65, M_PI);
  ros::Duration(0.5).sleep();
  sendGoalAndWaitForPath();
  const nav_msgs::Path initial_path = *path_;

  // The path is changed to keep distance from the obstacle.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 100;
  publishMapAndRobot(1.65, 0.65, M_PI);
  ros::Duration(0.5).sleep();
  sendGoalAndWaitForPath();
  EXPECT_FALSE(comparePath(initial_path, *path_));

  // Enable start pose prediction.
  move_base_->cancelAllGoals();
  planner_cspace::Planner3DConfig config = default_config_;
  config.keep_a_part_of_previous_path = true;
  config.dist_stop_to_previous_path = 0.1;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  // No obstacle and the path is same as the first one.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 0;
  publishMapAndRobot(1.65, 0.65, M_PI);
  ros::Duration(0.5).sleep();
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(initial_path, *path_));

  // The path does not change as a part of the previous path is kept and it is not possible to keep distance from
  // the obstacle.
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 100;
  publishMapAndRobot(1.65, 0.65, M_PI);
  ros::Duration(0.5).sleep();
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(initial_path, *path_));

  // It is expected that the robot reaches the goal during the path planning.
  move_base_->cancelAllGoals();
  map_overlay_.data[13 + 5 * map_overlay_.info.width] = 0;
  publishMapAndRobot(1.25, 0.95, M_PI / 2);
  ros::Duration(0.5).sleep();
  sendGoalAndWaitForPath();
  const nav_msgs::Path short_path = *path_;
  // In the second path planning after cancel, the exptected start pose is same as the goal.
  sendGoalAndWaitForPath();
  EXPECT_TRUE(comparePath(short_path, *path_));
}

TEST_F(DynamicParameterChangeTest, TriggerPlanByCostmapUpdate)
{
  publishMapAndRobot(2.55, 0.45, M_PI);
  ros::Duration(0.5).sleep();
  sendGoalAndWaitForPath();

  const double hz = getPathHz(ros::Duration(0.1));
  EXPECT_LT(3.0, hz);
  EXPECT_LT(hz, 5.0);

  planner_cspace::Planner3DConfig config = default_config_;
  config.trigger_plan_by_costmap_update = true;
  config.costmap_watchdog = 0.5;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  const double hz2 = getPathHz(ros::Duration(0.1));
  EXPECT_LT(9.0, hz2);
  EXPECT_LT(hz2, 10.0);

  const double hz3 = getPathHz(ros::Duration(100));
  EXPECT_LT(1.0, hz3);
  EXPECT_LT(hz3, 3.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_dynamic_parameter_change");
  return RUN_ALL_TESTS();
}
