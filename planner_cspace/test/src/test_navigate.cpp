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

#include <gtest/gtest.h>

namespace planner_cspace_msgs
{
std::ostream& operator<<(std::ostream& os, const PlannerStatus::ConstPtr& msg)
{
  if (!msg)
  {
    os << "nullptr";
  }
  else
  {
    os << std::endl
       << "  header: " << msg->header.stamp << " " << msg->header.frame_id << std::endl
       << "  status: " << static_cast<int>(msg->status) << std::endl
       << "  error: " << static_cast<int>(msg->error);
  }
  return os;
}
}  // namespace planner_cspace_msgs

class Navigate : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  nav_msgs::OccupancyGrid::ConstPtr map_;
  nav_msgs::OccupancyGrid::ConstPtr map_local_;
  planner_cspace_msgs::PlannerStatus::ConstPtr planner_status_;
  costmap_cspace_msgs::CSpace3D::ConstPtr costmap_;
  nav_msgs::Path::ConstPtr path_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_local_;
  ros::Subscriber sub_costmap_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_path_;
  ros::ServiceClient srv_forget_;
  ros::Publisher pub_map_;
  ros::Publisher pub_map_local_;
  ros::Publisher pub_initial_pose_;
  size_t local_map_apply_cnt_;
  std::vector<tf2::Stamped<tf2::Transform>> traj_;
  std::string test_scope_;

  Navigate()
    : tfl_(tfbuf_)
    , local_map_apply_cnt_(0)
  {
    sub_map_ = nh_.subscribe("map_global", 1, &Navigate::cbMap, this);
    sub_map_local_ = nh_.subscribe("map_local", 1, &Navigate::cbMapLocal, this);
    sub_costmap_ = nh_.subscribe("costmap", 1, &Navigate::cbCostmap, this);
    sub_status_ = nh_.subscribe(
        "/planner_3d/status", 10, &Navigate::cbStatus, this);
    sub_path_ = nh_.subscribe("path", 1, &Navigate::cbPath, this);
    srv_forget_ =
        nh_.serviceClient<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
            "forget_planning_cost");
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    pub_map_local_ = nh_.advertise<nav_msgs::OccupancyGrid>("overlay", 1, true);
    pub_initial_pose_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
  }

  virtual void SetUp()
  {
    test_scope_ = "[" + std::to_string(getpid()) + "] ";

    srv_forget_.waitForExistence(ros::Duration(10.0));
    ros::Rate rate(10.0);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 2.5;
    pose.pose.pose.position.y = 0.45;
    pose.pose.pose.orientation.z = 1.0;
    pub_initial_pose_.publish(pose);

    const ros::Time deadline = ros::Time::now() + ros::Duration(15);

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

    while (ros::ok() && !map_)
    {
      ros::spinOnce();
      rate.sleep();
      const ros::Time now = ros::Time::now();
      if (now > deadline)
      {
        FAIL() << test_scope_ << now << " SetUp: map timeout" << std::endl;
      }
    }
    pub_map_.publish(map_);
    std::cerr << test_scope_ << ros::Time::now() << " Map applied." << std::endl;

    while (ros::ok() && !costmap_)
    {
      ros::spinOnce();
      rate.sleep();
      const ros::Time now = ros::Time::now();
      if (now > deadline)
      {
        FAIL() << test_scope_ << now << " SetUp: costmap timeout" << std::endl;
      }
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
    map_local_ = msg;
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
  void pubMapLocal()
  {
    if (map_local_)
    {
      pub_map_local_.publish(map_local_);
      if ((local_map_apply_cnt_++) % 30 == 0)
        std::cerr << test_scope_ << " Local map applied." << std::endl;
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
};

TEST_F(Navigate, Navigate)
{
  ros::Publisher pub_path = nh_.advertise<nav_msgs::Path>("patrol_nodes", 1, true);

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
  pub_path.publish(path);

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
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  pub_path.publish(path);

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate");

  return RUN_ALL_TESTS();
}
