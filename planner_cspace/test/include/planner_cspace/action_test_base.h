/*
 * Copyright (c) 2018-2023, the neonavigation authors
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

#ifndef PLANNER_CSPACE_ACTION_TEST_BASE_H
#define PLANNER_CSPACE_ACTION_TEST_BASE_H

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

constexpr const char ACTION_TOPIC_MOVE_BASE[] = "/move_base";
constexpr const char ACTION_TOPIC_TOLERANT_MOVE[] = "/tolerant_move";

template <typename ACTION, char const* TOPIC>
class ActionTestBase : public ::testing::Test
{
public:
  ActionTestBase()
    : tfl_(tfbuf_)
    , map_ready_(false)
  {
    move_base_ = std::make_shared<ActionClient>(TOPIC);
    sub_status_ = node_.subscribe(
        "/planner_3d/status", 10, &ActionTestBase::cbStatus, this);
  }
  void SetUp()
  {
    if (!move_base_->waitForServer(ros::Duration(30.0)))
    {
      FAIL() << "Failed to connect move_base action";
    }

    ros::ServiceClient srv_plan =
        node_.serviceClient<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>(
            "/planner_3d/make_plan");

    const ros::Time deadline = ros::Time::now() + ros::Duration(10.0);
    while (ros::ok())
    {
      nav_msgs::GetPlanRequest req;
      nav_msgs::GetPlanResponse res;
      req.tolerance = 10.0;
      req.start.header.frame_id = "map";
      req.start.pose.position.x = 1.24;
      req.start.pose.position.y = 0.65;
      req.start.pose.orientation.w = 1;
      req.goal.header.frame_id = "map";
      req.goal.pose.position.x = 1.25;
      req.goal.pose.position.y = 0.75;
      req.goal.pose.orientation.w = 1;
      if (srv_plan.call(req, res))
      {
        // Planner is ready.
        break;
      }
      if (ros::Time::now() > deadline)
      {
        FAIL() << "planner_3d didn't receive map";
      }
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
  }
  ~ActionTestBase()
  {
  }

protected:
  using ActionClient = actionlib::SimpleActionClient<ACTION>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;

  void cbStatus(const planner_cspace_msgs::PlannerStatus::ConstPtr& msg)
  {
    planner_status_ = msg;
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

  ros::NodeHandle node_;
  ros::Subscriber sub_status_;
  ActionClientPtr move_base_;
  planner_cspace_msgs::PlannerStatus::ConstPtr planner_status_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  bool map_ready_;
};

#endif  // PLANNER_CSPACE_ACTION_TEST_BASE_H
