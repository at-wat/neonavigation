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

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <ros/ros.h>

class AbortTest : public ::testing::Test
{
public:
  AbortTest()
    : node_()
  {
    move_base_ = std::make_shared<ActionClient>("/move_base");
    status_sub_ = node_.subscribe(
        "/planner_3d/status", 10, &AbortTest::cbStatus, this);
    if (!move_base_->waitForServer(ros::Duration(30.0)))
    {
      ROS_ERROR("Failed to connect move_base action");
      exit(EXIT_FAILURE);
    }
  }
  ~AbortTest()
  {
  }

protected:
  using ActionClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;

  void cbStatus(const planner_cspace_msgs::PlannerStatus::ConstPtr& msg)
  {
    status_ = msg;
  }
  move_base_msgs::MoveBaseGoal createGoalInRock()
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 1.19;
    goal.target_pose.pose.position.y = 1.90;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
  }
  move_base_msgs::MoveBaseGoal createGoalInFree()
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 2.1;
    goal.target_pose.pose.position.y = 0.45;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    return goal;
  }
  std::string statusString() const
  {
    if (!status_)
    {
      return "(no status)";
    }
    return "(status: " + std::to_string(status_->status) +
           ", error: " + std::to_string(status_->error) + ")";
  }

  ros::NodeHandle node_;
  ros::Subscriber status_sub_;
  ActionClientPtr move_base_;
  planner_cspace_msgs::PlannerStatus::ConstPtr status_;
};

TEST_F(AbortTest, AbortByGoalInRock)
{
  const ros::Time deadline = ros::Time::now() + ros::Duration(10);
  const ros::Duration wait(1.0);

  // Assure that goal is received after map in planner_3d.
  ros::Duration(0.5).sleep();
  // Send a goal which is in Rock
  move_base_->sendGoal(createGoalInRock());
  while (move_base_->getState().state_ !=
         actionlib::SimpleClientGoalState::ACTIVE)
  {
    wait.sleep();
    ASSERT_LT(ros::Time::now(), deadline)
        << "Action didn't get active: " << move_base_->getState().toString()
        << " " << statusString();
  }

  // Try to replan
  while (move_base_->getState().state_ ==
         actionlib::SimpleClientGoalState::ACTIVE)
  {
    wait.sleep();
    ASSERT_LT(ros::Time::now(), deadline)
        << "Action didn't get inactive: " << move_base_->getState().toString()
        << " " << statusString();
  }
  wait.sleep();

  ASSERT_TRUE(status_);

  // Abort after exceeding max_retry_num
  ASSERT_EQ(actionlib::SimpleClientGoalState::ABORTED,
            move_base_->getState().state_);
  ASSERT_EQ(planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND,
            status_->error);

  // Send another goal which is not in Rock
  move_base_->sendGoal(createGoalInFree());
  while (move_base_->getState().state_ !=
         actionlib::SimpleClientGoalState::ACTIVE)
  {
    wait.sleep();
    ASSERT_LT(ros::Time::now(), deadline)
        << "Action didn't get active: " << move_base_->getState().toString()
        << " " << statusString();
  }
  while (move_base_->getState().state_ ==
         actionlib::SimpleClientGoalState::ACTIVE)
  {
    wait.sleep();
    ASSERT_LT(ros::Time::now(), deadline)
        << "Action didn't get inactive: " << move_base_->getState().toString()
        << " " << statusString();
  }
  wait.sleep();

  // Succeed
  ASSERT_EQ(actionlib::SimpleClientGoalState::SUCCEEDED,
            move_base_->getState().state_);
  ASSERT_EQ(planner_cspace_msgs::PlannerStatus::GOING_WELL,
            status_->error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_abort");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
