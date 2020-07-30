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

#include <cstddef>
#include <memory>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <gtest/gtest.h>

class NavigateBoundary : public ::testing::Test
{
protected:
  using ActionClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;

  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster tfb_;
  ros::Subscriber sub_path_;
  nav_msgs::Path::ConstPtr path_;
  ActionClientPtr move_base_;

  NavigateBoundary()
  {
    move_base_ = std::make_shared<ActionClient>("/move_base");
    if (!move_base_->waitForServer(ros::Duration(10.0)))
    {
      ROS_ERROR("Failed to connect move_base action");
      exit(EXIT_FAILURE);
    }
  }

  void publishTransform(const double x, const double y)
  {
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = x;
    trans.transform.translation.y = y;
    trans.transform.rotation.w = 1.0;
    tfb_.sendTransform(trans);
  }
  virtual void SetUp()
  {
    sub_path_ = nh_.subscribe("path", 1, &NavigateBoundary::cbPath, this);

    publishTransform(1.0, 0.6);
    ros::Duration(0.5).sleep();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation.w = 1;
    goal.target_pose.pose.position.x = 1.4;
    goal.target_pose.pose.position.y = 0.6;
    move_base_->sendGoal(goal);
    ros::Duration(0.5).sleep();
  }
  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = msg;
  }
};

TEST_F(NavigateBoundary, StartPositionScan)
{
  // map width/height is 32px * 0.1m = 3.2m
  for (double x = -10; x < 13; x += 2.0)
  {
    for (double y = -10; y < 13; y += 2.0)
    {
      publishTransform(x, y);

      path_ = nullptr;
      for (int i = 0; i < 100; ++i)
      {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        if (path_)
          break;
      }
      // Planner must publish at least empty path if alive.
      ASSERT_TRUE(static_cast<bool>(path_));
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate_boundary");

  return RUN_ALL_TESTS();
}
