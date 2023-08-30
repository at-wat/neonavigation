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
#include <planner_cspace/Planner3DConfig.h>
#include <ros/ros.h>

#include <planner_cspace/action_test_base.h>

class DynamicParameterChangeTest
  : public ActionTestBase<move_base_msgs::MoveBaseAction, ACTION_TOPIC_MOVE_BASE>
{
public:
  void SetUp() final
  {
    ActionTestBase<move_base_msgs::MoveBaseAction, ACTION_TOPIC_MOVE_BASE>::SetUp();
    path_ = nullptr;
    planner_3d_client_.reset(
        new dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>("/planner_3d/"));
    sub_path_ = node_.subscribe("path", 1, &DynamicParameterChangeTest::cbPath, this);
  }

protected:
  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = msg;
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

  ros::Subscriber sub_path_;
  nav_msgs::Path::ConstPtr path_;
  std::unique_ptr<dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>> planner_3d_client_;
};

TEST_F(DynamicParameterChangeTest, DisableCurves)
{
  ros::Time deadline = ros::Time::now() + ros::Duration(1.0);
  move_base_->sendGoal(CreateGoalInFree());
  while (ros::ok())
  {
    if (path_ && (path_->poses.size() > 0))
    {
      break;
    }
    ASSERT_LT(ros::Time::now(), deadline)
        << "Faile to plan:" << move_base_->getState().toString() << statusString();
    ros::spinOnce();
  }
  // The default path is including curves.
  EXPECT_TRUE(isPathIncludingCurves());

  planner_cspace::Planner3DConfig config;
  ASSERT_TRUE(planner_3d_client_->getCurrentConfiguration(config, ros::Duration(0.1)));
  // Large min_curve_radius disables curves.
  config.min_curve_radius = 10.0;
  ASSERT_TRUE(planner_3d_client_->setConfiguration(config));

  const ros::Time config_sent_time = ros::Time::now();
  deadline = config_sent_time + ros::Duration(1.0);
  path_ = nullptr;
  while (ros::ok())
  {
    if (path_ && (path_->poses.size() > 0) && (path_->header.stamp > config_sent_time))
    {
      break;
    }
    ASSERT_LT(ros::Time::now(), deadline)
        << "Faile to plan:" << move_base_->getState().toString() << statusString();
    ros::spinOnce();
  }
  EXPECT_FALSE(isPathIncludingCurves());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_dynamic_parameter_change");
  return RUN_ALL_TESTS();
}
