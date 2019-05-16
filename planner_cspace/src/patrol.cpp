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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>

#include <neonavigation_common/compatibility.h>

class PatrolActionNode
{
protected:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_path_;
  std::shared_ptr<MoveBaseClient> act_cli_;

  nav_msgs::Path path_;
  size_t pos_;

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
    act_cli_.reset(new MoveBaseClient("move_base", false));

    pos_ = 0;
  }
  bool sendNextGoal()
  {
    move_base_msgs::MoveBaseGoal goal;

    if (path_.poses.size() <= pos_)
    {
      ROS_WARN("Patrol finished. Waiting next path.");
      path_.poses.clear();

      return false;
    }

    goal.target_pose.header = path_.poses[pos_].header;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = path_.poses[pos_].pose;

    act_cli_->sendGoal(goal);
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

      actionlib::SimpleClientGoalState state = act_cli_->getState();
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
