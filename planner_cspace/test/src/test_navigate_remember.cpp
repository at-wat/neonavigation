/*
 * Copyright (c) 2024, the neonavigation authors
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
#include <cstddef>
#include <string>
#include <unistd.h>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <std_srvs/Empty.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <planner_cspace/planner_status.h>

#include <gtest/gtest.h>

class NavigateWithRememberUpdates : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  planner_cspace_msgs::PlannerStatus::ConstPtr planner_status_;
  costmap_cspace_msgs::CSpace3D::ConstPtr costmap_;
  nav_msgs::Path::ConstPtr path_;
  ros::Subscriber sub_costmap_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_path_;
  ros::ServiceClient srv_forget_;
  ros::Publisher pub_initial_pose_;
  ros::Publisher pub_patrol_nodes_;
  std::vector<tf2::Stamped<tf2::Transform>> traj_;
  std::string test_scope_;

  NavigateWithRememberUpdates()
    : pnh_("~")
    , tfl_(tfbuf_)
  {
    sub_costmap_ = nh_.subscribe("costmap", 1, &NavigateWithRememberUpdates::cbCostmap, this);
    sub_status_ = nh_.subscribe(
        "/planner_3d/status", 10, &NavigateWithRememberUpdates::cbStatus, this);
    sub_path_ = nh_.subscribe("path", 1, &NavigateWithRememberUpdates::cbPath, this);
    srv_forget_ =
        nh_.serviceClient<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
            "forget_planning_cost");
    pub_initial_pose_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    pub_patrol_nodes_ = nh_.advertise<nav_msgs::Path>("patrol_nodes", 1, true);
  }

  void SetUp() override
  {
    test_scope_ = "[" + std::to_string(getpid()) + "] ";

    ros::Rate rate(10.0);

    const ros::Time deadline = ros::Time::now() + ros::Duration(15);
    while (ros::ok())
    {
      rate.sleep();
      ASSERT_LT(ros::Time::now(), deadline)
          << test_scope_ << "Initialization timeout: "
          << "sub_costmap:" << sub_costmap_.getNumPublishers() << " "
          << "sub_status:" << sub_status_.getNumPublishers() << " "
          << "pub_initial_pose:" << pub_initial_pose_.getNumSubscribers() << " "
          << "pub_patrol_nodes:" << pub_patrol_nodes_.getNumSubscribers() << " ";
      if (sub_costmap_.getNumPublishers() > 0 &&
          sub_status_.getNumPublishers() > 0 &&
          pub_initial_pose_.getNumSubscribers() > 0 &&
          pub_patrol_nodes_.getNumSubscribers() > 0)
      {
        break;
      }
    }
    ASSERT_TRUE(srv_forget_.waitForExistence(ros::Duration(10.0)));

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 2.1;
    pose.pose.pose.position.y = 3.0;
    pose.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
    pub_initial_pose_.publish(pose);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      const ros::Time now = ros::Time::now();
      if (now > deadline)
      {
        FAIL() << test_scope_ << now << " SetUp: transform timeout" << std::endl;
      }
      if (tfbuf_.canTransform("map", "base_link", now, ros::Duration(0.5)))
      {
        break;
      }
    }

    while (ros::ok() && !costmap_)
    {
      ros::spinOnce();
      rate.sleep();
      ASSERT_LT(ros::Time::now(), deadline) << test_scope_ << "Initial costmap timeout";
    }

    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    srv_forget_.call(req, res);

    ros::Duration(1.0).sleep();
  }
  void cbCostmap(const costmap_cspace_msgs::CSpace3D::ConstPtr& msg)
  {
    costmap_ = msg;
    std::cerr << test_scope_ << msg->header.stamp << " Costmap received." << std::endl;
  }
  void cbStatus(const planner_cspace_msgs::PlannerStatus::ConstPtr& msg)
  {
    if (!planner_status_ || planner_status_->status != msg->status || planner_status_->error != msg->error)
    {
      std::cerr << test_scope_ << msg->header.stamp << " Status updated." << msg << std::endl;
    }
    planner_status_ = msg;
  }
  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    if (!path_ || path_->poses.size() != msg->poses.size())
    {
      if (msg->poses.size() == 0)
      {
        std::cerr << test_scope_ << msg->header.stamp << " Path updated. (empty)" << std::endl;
      }
      else
      {
        std::cerr
            << test_scope_ << msg->header.stamp << " Path updated." << std::endl
            << msg->poses.front().pose.position.x << ", " << msg->poses.front().pose.position.y << std::endl
            << msg->poses.back().pose.position.x << ", " << msg->poses.back().pose.position.y << std::endl;
      }
      path_ = msg;
    }
  }
  tf2::Stamped<tf2::Transform> lookupRobotTrans(const ros::Time& now)
  {
    geometry_msgs::TransformStamped trans_tmp =
        tfbuf_.lookupTransform("map", "base_link", now, ros::Duration(0.5));
    tf2::Stamped<tf2::Transform> trans;
    tf2::fromMsg(trans_tmp, trans);
    traj_.push_back(trans);
    return trans;
  }
  void dumpRobotTrajectory()
  {
    double x_prev(0), y_prev(0);
    tf2::Quaternion rot_prev(0, 0, 0, 1);

    std::cerr << test_scope_ << traj_.size() << " points recorded" << std::endl;

    for (const auto& t : traj_)
    {
      const double x = t.getOrigin().getX();
      const double y = t.getOrigin().getY();
      const tf2::Quaternion rot = t.getRotation();
      const double yaw_diff = rot.angleShortestPath(rot_prev);
      if (std::abs(x - x_prev) > 0.1 || std::abs(y - y_prev) > 0.1 || std::abs(yaw_diff) > 0.2)
      {
        x_prev = x;
        y_prev = y;
        rot_prev = rot;
        std::cerr << t.stamp_ << " " << x << " " << y << " " << tf2::getYaw(rot) << std::endl;
      }
    }
  }

  void waitForPlannerStatus(const std::string& name, const int expected_error)
  {
    ros::spinOnce();
    ros::Duration(0.2).sleep();

    ros::Rate wait(10);
    ros::Time deadline = ros::Time::now() + ros::Duration(10);
    while (ros::ok())
    {
      ros::spinOnce();
      wait.sleep();

      const ros::Time now = ros::Time::now();

      if (now > deadline)
      {
        dumpRobotTrajectory();
        FAIL()
            << test_scope_ << "/" << name << ": Navigation timeout." << std::endl
            << "now: " << now << std::endl
            << "status: " << planner_status_ << " (expected: " << expected_error << ")";
      }

      if (planner_status_->error == expected_error)
      {
        return;
      }
    }
  }
};

TEST_F(NavigateWithRememberUpdates, Navigate)
{
  ros::spinOnce();
  ros::Duration(0.2).sleep();

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.5;
  path.poses[0].pose.position.y = 5.6;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(10);
  const ros::Time deadline = ros::Time::now() + ros::Duration(120);
  while (ros::ok())
  {
    ros::spinOnce();
    wait.sleep();

    const ros::Time now = ros::Time::now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now << std::endl
          << "status: " << planner_status_;
      break;
    }

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      trans = lookupRobotTrans(now);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 &&
        planner_status_->status == planner_cspace_msgs::PlannerStatus::DONE)
    {
      std::cerr << test_scope_ << "Navagation success." << std::endl;
      ros::Duration(2.0).sleep();
      return;
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(NavigateWithRememberUpdates, CrowdEscape)
{
  if (!pnh_.param("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  ros::Publisher pub_trigger = nh_.advertise<std_msgs::Empty>("/planner_3d/temporary_escape", 1);

  ros::spinOnce();
  ros::Duration(0.2).sleep();

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.5;
  path.poses[0].pose.position.y = 5.6;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(2);
  const ros::Time deadline = ros::Time::now() + ros::Duration(120);
  while (ros::ok())
  {
    std_msgs::Empty msg;
    pub_trigger.publish(msg);

    ros::spinOnce();
    wait.sleep();

    const ros::Time now = ros::Time::now();

    if (now > deadline)
    {
      dumpRobotTrajectory();
      FAIL()
          << test_scope_ << "Navigation timeout." << std::endl
          << "now: " << now << std::endl
          << "status: " << planner_status_;
      break;
    }

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      trans = lookupRobotTrans(now);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << test_scope_ << e.what() << std::endl;
      continue;
    }

    const auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2 &&
        planner_status_->status == planner_cspace_msgs::PlannerStatus::DONE)
    {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      return;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate_remember");

  return RUN_ALL_TESTS();
}
