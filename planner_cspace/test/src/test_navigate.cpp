/*
 * Copyright (c) 2017, the neonavigation authors
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
#include <string>
#include <unistd.h>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>

#include <planner_cspace/planner_status.h>

#include <gtest/gtest.h>

class Navigate : public ::testing::Test
{
protected:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
  nav_msgs::OccupancyGrid::Ptr map_local_;
  planner_cspace_msgs::PlannerStatus::ConstPtr planner_status_;
  costmap_cspace_msgs::CSpace3D::ConstPtr costmap_;
  nav_msgs::Path::ConstPtr path_;
  trajectory_tracker_msgs::PathWithVelocity::ConstPtr path_vel_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_local_;
  ros::Subscriber sub_costmap_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_path_;
  ros::Subscriber sub_path_vel_;
  ros::ServiceClient srv_forget_;
  ros::Publisher pub_map_;
  ros::Publisher pub_map_local_;
  ros::Publisher pub_initial_pose_;
  ros::Publisher pub_patrol_nodes_;
  size_t local_map_apply_cnt_;
  std::vector<tf2::Stamped<tf2::Transform>> traj_;
  std::string test_scope_;

  Navigate()
    : pnh_("~")
    , tfl_(tfbuf_)
    , local_map_apply_cnt_(0)
  {
    sub_map_ = nh_.subscribe("map_global", 1, &Navigate::cbMap, this);
    sub_map_local_ = nh_.subscribe("map_local", 1, &Navigate::cbMapLocal, this);
    sub_costmap_ = nh_.subscribe("costmap", 1, &Navigate::cbCostmap, this);
    sub_status_ = nh_.subscribe(
        "/planner_3d/status", 10, &Navigate::cbStatus, this);
    sub_path_ = nh_.subscribe("path", 1, &Navigate::cbPath, this);
    sub_path_vel_ = nh_.subscribe("path_velocity", 1, &Navigate::cbPathVel, this);
    srv_forget_ =
        nh_.serviceClient<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
            "forget_planning_cost");
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    pub_map_local_ = nh_.advertise<nav_msgs::OccupancyGrid>("overlay", 1, true);
    pub_initial_pose_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    pub_patrol_nodes_ = nh_.advertise<nav_msgs::Path>("patrol_nodes", 1, true);
  }

  virtual void SetUp()
  {
    test_scope_ =
        "[" + std::to_string(getpid()) + "/" +
        ::testing::UnitTest::GetInstance()->current_test_info()->name() + "] ";

    ros::Rate rate(10.0);

    const ros::Time deadline = ros::Time::now() + ros::Duration(15.0);
    while (ros::ok())
    {
      rate.sleep();
      ASSERT_LT(ros::Time::now(), deadline)
          << test_scope_ << "Initialization timeout: "
          << "sub_map:" << sub_map_.getNumPublishers() << " "
          << "sub_map_local:" << sub_map_local_.getNumPublishers() << " "
          << "sub_costmap:" << sub_costmap_.getNumPublishers() << " "
          << "sub_status:" << sub_status_.getNumPublishers() << " "
          << "pub_map:" << pub_map_.getNumSubscribers() << " "
          << "pub_map_local:" << pub_map_local_.getNumSubscribers() << " "
          << "pub_initial_pose:" << pub_initial_pose_.getNumSubscribers() << " "
          << "pub_patrol_nodes:" << pub_patrol_nodes_.getNumSubscribers() << " ";
      if (sub_map_.getNumPublishers() > 0 &&
          sub_map_local_.getNumPublishers() > 0 &&
          sub_costmap_.getNumPublishers() > 0 &&
          sub_status_.getNumPublishers() > 0 &&
          pub_map_.getNumSubscribers() > 0 &&
          pub_map_local_.getNumSubscribers() > 0 &&
          pub_initial_pose_.getNumSubscribers() > 0 &&
          pub_patrol_nodes_.getNumSubscribers() > 0)
      {
        break;
      }
    }
    ASSERT_TRUE(srv_forget_.waitForExistence(ros::Duration(10.0)));

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 2.5;
    pose.pose.pose.position.y = 0.45;
    pose.pose.pose.orientation.z = 1.0;
    pub_initial_pose_.publish(pose);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      const ros::Time now = ros::Time::now();
      ASSERT_LT(now, deadline) << test_scope_ << "Initial transform timeout";
      if (tfbuf_.canTransform("map", "base_link", now, ros::Duration(0.5)))
      {
        break;
      }
    }

    while (ros::ok() && !map_)
    {
      ros::spinOnce();
      rate.sleep();
      ASSERT_LT(ros::Time::now(), deadline) << test_scope_ << "Initial map timeout";
    }
    pub_map_.publish(map_);
    std::cerr << test_scope_ << ros::Time::now() << " Map applied." << std::endl;

    while (ros::ok() && !map_local_)
    {
      ros::spinOnce();
      rate.sleep();
      ASSERT_LT(ros::Time::now(), deadline) << test_scope_ << "Initial local map timeout";
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
  void cbMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    map_ = msg;
    std::cerr << test_scope_ << msg->header.stamp << " Map received." << std::endl;
  }
  void cbMapLocal(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    if (map_local_)
    {
      return;
    }
    map_local_.reset(new nav_msgs::OccupancyGrid(*msg));
    std::cerr << test_scope_ << msg->header.stamp << " Local map received." << std::endl;
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
  void cbPathVel(const trajectory_tracker_msgs::PathWithVelocity::ConstPtr& msg)
  {
    if (!path_vel_ || path_vel_->poses.size() != msg->poses.size())
    {
      if (msg->poses.size() == 0)
      {
        std::cerr << test_scope_ << msg->header.stamp << " PathWithVelocity updated. (empty)" << std::endl;
      }
      else
      {
        std::cerr
            << test_scope_ << msg->header.stamp << " PathWithVelocity updated." << std::endl
            << msg->poses.front().pose.position.x << ", " << msg->poses.front().pose.position.y << std::endl
            << msg->poses.back().pose.position.x << ", " << msg->poses.back().pose.position.y << std::endl;
      }
      path_vel_ = msg;
    }
  }
  void pubMapLocal()
  {
    if (!map_local_)
    {
      return;
    }
    pub_map_local_.publish(map_local_);
    if ((local_map_apply_cnt_++) % 30 == 0)
    {
      int num_occupied = 0;
      for (const auto& c : map_local_->data)
      {
        if (c == 100)
        {
          num_occupied++;
        }
      }
      std::cerr << test_scope_ << " Local map applied. occupied:" << num_occupied << std::endl;
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
      if (std::abs(x - x_prev) >= 0 || std::abs(y - y_prev) >= 0 || std::abs(yaw_diff) >= 0)
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
    ASSERT_TRUE(static_cast<bool>(map_));
    ASSERT_TRUE(static_cast<bool>(map_local_));
    pubMapLocal();
    ros::Duration(0.2).sleep();

    ros::Rate wait(10);
    ros::Time deadline = ros::Time::now() + ros::Duration(10);
    while (ros::ok())
    {
      pubMapLocal();
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

TEST_F(Navigate, Navigate)
{
  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));

  nav_msgs::Path path;
  path.poses.resize(2);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  path.poses[1].header.frame_id = path.header.frame_id;
  path.poses[1].pose.position.x = 1.9;
  path.poses[1].pose.position.y = 2.8;
  path.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  pub_patrol_nodes_.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(10);
  const ros::Time deadline = ros::Time::now() + ros::Duration(60);
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
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2)
    {
      std::cerr << test_scope_ << "Navigation success." << std::endl;
      ros::Duration(2.0).sleep();
      return;
    }

    for (int x = -2; x <= 2; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        const tf2::Vector3 pos =
            trans * tf2::Vector3(x * map_->info.resolution, y * map_->info.resolution, 0);
        const int map_x = pos.x() / map_->info.resolution;
        const int map_y = pos.y() / map_->info.resolution;
        const size_t addr = map_x + map_y * map_->info.width;
        ASSERT_LT(addr, map_->data.size());
        ASSERT_LT(map_x, static_cast<int>(map_->info.width));
        ASSERT_LT(map_y, static_cast<int>(map_->info.height));
        ASSERT_GE(map_x, 0);
        ASSERT_GE(map_y, 0);
        ASSERT_NE(map_->data[addr], 100);
      }
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(Navigate, NavigateWithLocalMap)
{
  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));
  pubMapLocal();
  ros::Duration(0.2).sleep();

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  pub_patrol_nodes_.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(10);
  const ros::Time deadline = ros::Time::now() + ros::Duration(60);
  while (ros::ok())
  {
    pubMapLocal();
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
        std::abs(tf2::getYaw(goal_rel.getRotation())) < 0.2)
    {
      std::cerr << test_scope_ << "Navagation success." << std::endl;
      ros::Duration(2.0).sleep();
      return;
    }

    for (int x = -2; x <= 2; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        const tf2::Vector3 pos =
            trans * tf2::Vector3(x * map_->info.resolution, y * map_->info.resolution, 0);
        const int map_x = pos.x() / map_->info.resolution;
        const int map_y = pos.y() / map_->info.resolution;
        const size_t addr = map_x + map_y * map_->info.width;
        ASSERT_LT(addr, map_->data.size());
        ASSERT_LT(map_x, static_cast<int>(map_->info.width));
        ASSERT_LT(map_y, static_cast<int>(map_->info.height));
        ASSERT_GE(map_x, 0);
        ASSERT_GE(map_y, 0);
        ASSERT_NE(map_->data[addr], 100);
        ASSERT_NE(map_local_->data[addr], 100);
      }
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(Navigate, GlobalPlan)
{
  ros::ServiceClient srv_plan =
      nh_.serviceClient<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>(
          "/planner_3d/make_plan");

  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));

  nav_msgs::GetPlanRequest req;
  nav_msgs::GetPlanResponse res;

  req.tolerance = 0.0;
  req.start.header.frame_id = "map";
  req.start.pose.position.x = 1.95;
  req.start.pose.position.y = 0.45;
  req.start.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 3.14));

  req.goal.header.frame_id = "map";
  req.goal.pose.position.x = 1.25;
  req.goal.pose.position.y = 2.15;
  req.goal.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  // Planning failes as (12, 21, 0) is in rock.
  ASSERT_FALSE(srv_plan.call(req, res));

  // Goal grid is moved to (12, 22, 0).
  req.tolerance = 0.1;
  ASSERT_TRUE(srv_plan.call(req, res));
  EXPECT_NEAR(1.25, res.plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(2.25, res.plan.poses.back().pose.position.y, 1.0e-5);

  // Goal grid is moved to (12, 23, 0). This is because cost of (12, 22, 0) is larger than 50.
  req.tolerance = 0.2f;
  ASSERT_TRUE(srv_plan.call(req, res));
  EXPECT_NEAR(1.25, res.plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(2.35, res.plan.poses.back().pose.position.y, 1.0e-5);

  req.tolerance = 0.0;
  req.goal.header.frame_id = "map";
  req.goal.pose.position.x = 1.85;
  req.goal.pose.position.y = 2.75;
  req.goal.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  ASSERT_TRUE(srv_plan.call(req, res));

  EXPECT_NEAR(req.start.pose.position.x, res.plan.poses.front().pose.position.x, 1.0e-5);
  EXPECT_NEAR(req.start.pose.position.y, res.plan.poses.front().pose.position.y, 1.0e-5);
  EXPECT_NEAR(req.goal.pose.position.x, res.plan.poses.back().pose.position.x, 1.0e-5);
  EXPECT_NEAR(req.goal.pose.position.y, res.plan.poses.back().pose.position.y, 1.0e-5);

  for (const geometry_msgs::PoseStamped& p : res.plan.poses)
  {
    const int map_x = p.pose.position.x / map_->info.resolution;
    const int map_y = p.pose.position.y / map_->info.resolution;
    const size_t addr = map_x + map_y * map_->info.width;
    ASSERT_LT(addr, map_->data.size());
    ASSERT_LT(map_x, static_cast<int>(map_->info.width));
    ASSERT_LT(map_y, static_cast<int>(map_->info.height));
    ASSERT_GE(map_x, 0);
    ASSERT_GE(map_y, 0);
    ASSERT_NE(map_->data[addr], 100);
  }
}

TEST_F(Navigate, RobotIsInRockOnSetGoal)
{
  ros::Publisher pub_path = nh_.advertise<nav_msgs::Path>("patrol_nodes", 1, true);

  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));
  pubMapLocal();
  ros::Duration(0.2).sleep();

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.19;
  path.poses[0].pose.position.y = 1.90;
  path.poses[0].pose.orientation.w = 1.0;
  pub_path.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(10);
  const ros::Time deadline = ros::Time::now() + ros::Duration(10);
  while (ros::ok())
  {
    pubMapLocal();
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

    if (planner_status_->error == planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND)
    {
      return;
    }
  }
  ASSERT_TRUE(false);
}

TEST_F(Navigate, GoalIsInRockRecovered)
{
  if (pnh_.param("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is set";
  }

  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  for (int x = 10; x <= 16; ++x)
  {
    for (int y = 22; y <= 26; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 100;
    }
  }
  pubMapLocal();

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.25;
  path.poses[0].pose.position.y = 2.55;
  path.poses[0].pose.orientation.w = 1.0;
  pub_patrol_nodes_.publish(path);

  waitForPlannerStatus("Got stuck", planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND);
  ASSERT_FALSE(::testing::Test::HasFailure());

  for (int x = 10; x <= 16; ++x)
  {
    for (int y = 22; y <= 26; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 0;
    }
  }
  waitForPlannerStatus("Stuck recovered", planner_cspace_msgs::PlannerStatus::GOING_WELL);
}

TEST_F(Navigate, RobotIsInRockOnRecovered)
{
  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  for (int x = 21; x <= 28; ++x)
  {
    for (int y = 2; y <= 8; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 100;
    }
  }
  pubMapLocal();

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.25;
  path.poses[0].pose.position.y = 2.55;
  path.poses[0].pose.orientation.w = 1.0;
  pub_patrol_nodes_.publish(path);

  waitForPlannerStatus("Got stuck", planner_cspace_msgs::PlannerStatus::IN_ROCK);
  ASSERT_FALSE(::testing::Test::HasFailure());

  for (int x = 21; x <= 28; ++x)
  {
    for (int y = 2; y <= 8; ++y)
    {
      const int pos = x + y * map_local_->info.width;
      map_local_->data[pos] = 0;
    }
  }
  waitForPlannerStatus("Stuck recovered", planner_cspace_msgs::PlannerStatus::GOING_WELL);
}

TEST_F(Navigate, RobotIsInCrowd)
{
  if (!pnh_.param("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.6;
  path.poses[0].pose.position.y = 2.2;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(10);
  const ros::Time deadline = ros::Time::now() + ros::Duration(60);
  while (ros::ok())
  {
    pubMapLocal();
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
      ros::Duration(2.0).sleep();
      return;
    }

    const size_t rx = trans.getOrigin().x() / map_->info.resolution;
    const size_t ry = trans.getOrigin().y() / map_->info.resolution;
    const size_t data_size = map_local_->data.size();
    map_local_->data.clear();
    map_local_->data.resize(data_size, 0);
    const int bs = 6;
    for (int i = -bs; i <= bs; ++i)
    {
      map_local_->data[std::min((rx + i) + (ry - bs) * map_local_->info.width, data_size - 1)] = 100;
      map_local_->data[std::min((rx + i) + (ry + bs) * map_local_->info.width, data_size - 1)] = 100;
      map_local_->data[std::min((rx - bs) + (ry + i) * map_local_->info.width, data_size - 1)] = 100;
      map_local_->data[std::min((rx + bs) + (ry + i) * map_local_->info.width, data_size - 1)] = 100;
    }
  }
}

TEST_F(Navigate, ForceTemporaryEscape)
{
  if (!pnh_.param("enable_crowd_mode", false))
  {
    GTEST_SKIP() << "enable_crowd_mode is not set";
  }

  ros::Publisher pub_trigger = nh_.advertise<std_msgs::Empty>("/planner_3d/temporary_escape", 1);

  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map_));
  ASSERT_TRUE(static_cast<bool>(map_local_));

  nav_msgs::Path path;
  path.poses.resize(1);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.6;
  path.poses[0].pose.position.y = 2.2;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 1.57));
  pub_patrol_nodes_.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(2);
  const ros::Time deadline = ros::Time::now() + ros::Duration(60);
  while (ros::ok())
  {
    const size_t data_size = map_local_->data.size();
    map_local_->data.clear();
    map_local_->data.resize(data_size, 0);
    pubMapLocal();

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
      ros::Duration(2.0).sleep();
      return;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate");

  return RUN_ALL_TESTS();
}
