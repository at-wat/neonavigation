/*
 * Copyright (c) 2016-2017, the neonavigation authors
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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planner_cspace_msgs/MoveWithToleranceAction.h>
#include <nav_msgs/Path.h>

#include <neonavigation_common/compatibility.h>

class PatrolActionNode
{
protected:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using MoveWithToleranceClient = actionlib::SimpleActionClient<planner_cspace_msgs::MoveWithToleranceAction>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_path_;
  std::shared_ptr<MoveBaseClient> act_cli_;
  std::shared_ptr<MoveWithToleranceClient> act_cli_tolerant_;

  nav_msgs::Path path_;
  size_t pos_;
  bool with_tolerance_;
  double tolerance_lin_;
  double tolerance_ang_;
  double tolerance_ang_finish_;

  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = *msg;
    pos_ = 0;
  }

public:
  PatrolActionNode()
    : nh_()
    , pnh_("~")
  {
    neonavigation_common::compat::checkCompatMode();
    sub_path_ = neonavigation_common::compat::subscribe(
        nh_, "patrol_nodes",
        pnh_, "path", 1, &PatrolActionNode::cbPath, this);

    pnh_.param("with_tolerance", with_tolerance_, false);
    pnh_.param("tolerance_lin", tolerance_lin_, 0.1);
    pnh_.param("tolerance_ang", tolerance_ang_, 0.1);
    pnh_.param("tolerance_ang_finish", tolerance_ang_finish_, 0.05);

    if (with_tolerance_)
    {
      act_cli_tolerant_.reset(new MoveWithToleranceClient("tolerant_move", false));
    }
    else
    {
      act_cli_.reset(new MoveBaseClient("move_base", false));
    }

    pos_ = 0;
  }
  bool sendNextGoal()
  {
    if (path_.poses.size() <= pos_)
    {
      ROS_WARN("Patrol finished. Waiting next path.");
      path_.poses.clear();

      return false;
    }

    if (with_tolerance_)
    {
      planner_cspace_msgs::MoveWithToleranceGoal goal;

      goal.target_pose.header = path_.poses[pos_].header;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = path_.poses[pos_].pose;
      goal.goal_tolerance_lin = tolerance_lin_;
      goal.goal_tolerance_ang = tolerance_ang_;
      goal.goal_tolerance_ang_finish = tolerance_ang_finish_;

      act_cli_tolerant_->sendGoal(goal);
    }
    else
    {
      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header = path_.poses[pos_].header;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = path_.poses[pos_].pose;

      act_cli_->sendGoal(goal);
    }
    pos_++;

    return true;
  }
  void spin()
  {
    ros::Rate rate(1.0);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();

      if (path_.poses.size() == 0)
      {
        continue;
      }

      if (pos_ == 0)
      {
        sendNextGoal();
        continue;
      }

      actionlib::SimpleClientGoalState state =
          with_tolerance_ ?
              act_cli_tolerant_->getState() :
              act_cli_->getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Action has been finished.");
        sendNextGoal();
      }
      else if (state == actionlib::SimpleClientGoalState::ABORTED)
      {
        ROS_ERROR("Action has been aborted. Skipping.");
        sendNextGoal();
      }
      else if (state == actionlib::SimpleClientGoalState::LOST)
      {
        ROS_WARN_ONCE("Action server is not ready.");
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "patrol");

  PatrolActionNode pa;
  pa.spin();

  return 0;
}
