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

#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planner_cspace_msgs/MoveWithToleranceAction.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>

class TolerantActionTest : public ::testing::Test
{
public:
  TolerantActionTest()
    : node_()
    , tfl_(tfbuf_)

  {
    move_base_ = std::make_shared<ActionClient>("/tolerant_move");
    status_sub_ = node_.subscribe(
        "/planner_3d/status", 10, &TolerantActionTest::cbStatus, this);
    if (!move_base_->waitForServer(ros::Duration(30.0)))
    {
      ROS_ERROR("Failed to connect move_base action");
      exit(EXIT_FAILURE);
    }
  }

protected:
  using ActionClient = actionlib::SimpleActionClient<planner_cspace_msgs::MoveWithToleranceAction>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;

  void cbStatus(const planner_cspace_msgs::PlannerStatus::ConstPtr& msg)
  {
    planner_status_ = msg;
  }

  planner_cspace_msgs::MoveWithToleranceGoal createGoalInFree()
  {
    planner_cspace_msgs::MoveWithToleranceGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 2.1;
    goal.target_pose.pose.position.y = 0.45;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goal.is_continuous_movement_mode = true;
    goal.goal_tolerance_ang = 0.1;
    goal.goal_tolerance_ang_finish = 0.05;
    goal.goal_tolerance_lin = 0.2;
    return goal;
  }
  std::string statusString() const
  {
    if (!planner_status_)
    {
      return "(no status)";
    }
    return "(status: " + std::to_string(planner_status_->status) +
           ", error: " + std::to_string(planner_status_->error) + ")";
  }

  double getDistBetweenRobotAndGoal(const planner_cspace_msgs::MoveWithToleranceGoal& goal)
  {
    try
    {
      const geometry_msgs::TransformStamped map_to_robot =
          tfbuf_.lookupTransform("map", "base_link", ros::Time(), ros::Duration(0.1));
      return std::hypot(map_to_robot.transform.translation.x - goal.target_pose.pose.position.x,
                        map_to_robot.transform.translation.y - goal.target_pose.pose.position.y);
    }
    catch (std::exception&)
    {
      return std::numeric_limits<double>::max();
    }
  }

  ros::NodeHandle node_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  ros::Subscriber status_sub_;
  ActionClientPtr move_base_;
  planner_cspace_msgs::PlannerStatus::ConstPtr planner_status_;
};

TEST_F(TolerantActionTest, GoalWithTolerance)
{
  const ros::Time deadline = ros::Time::now() + ros::Duration(10);
  const ros::Duration wait(1.0);

  // Assure that goal is received after map in planner_3d.
  ros::Duration(0.5).sleep();
  const planner_cspace_msgs::MoveWithToleranceGoal goal = createGoalInFree();
  move_base_->sendGoal(goal);

  while (ros::ok() && move_base_->getState().state_ != actionlib::SimpleClientGoalState::ACTIVE)
  {
    ASSERT_LT(ros::Time::now(), deadline)
        << "Action didn't get active: " << move_base_->getState().toString()
        << " " << statusString();
    ros::spinOnce();
  }

  while (ros::ok() && move_base_->getState().state_ != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ASSERT_LT(ros::Time::now(), deadline)
        << "Action didn't succeeded: " << move_base_->getState().toString()
        << " " << statusString();
    ros::spinOnce();
  }

  const double dist_to_goal = getDistBetweenRobotAndGoal(goal);
  // distance_remains is less than updated goal_tolerance_lin (set in planner_cspace_msgs::MoveWithToleranceGoal).
  EXPECT_LT(dist_to_goal, goal.goal_tolerance_lin);
  // distance_remains is greater than default goal_tolerance_lin (set in actionlib_common_rostest.test).
  EXPECT_GT(dist_to_goal, 0.05);
  // Navigation still continues after ActionClient succeeded.
  EXPECT_EQ(planner_status_->status, planner_cspace_msgs::PlannerStatus::DOING);

  while (ros::ok() && planner_status_->status != planner_cspace_msgs::PlannerStatus::DONE)
  {
    ASSERT_LT(ros::Time::now(), deadline)
        << "Navigation didn't finished: " << move_base_->getState().toString()
        << " " << statusString();
    ros::spinOnce();
  }
  EXPECT_LT(getDistBetweenRobotAndGoal(goal), 0.05);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_tolerant_action");
  return RUN_ALL_TESTS();
}
