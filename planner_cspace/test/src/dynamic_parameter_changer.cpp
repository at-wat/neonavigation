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
#include <string>

#include <gtest/gtest.h>

#include <dynamic_reconfigure/client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <planner_cspace/Planner3DConfig.h>
#include <ros/ros.h>

#include <planner_cspace/action_test_base.h>

class DynamicParameterChangeTest
{
public:
  DynamicParameterChangeTest()
    : tfl_(tfbuf_)
    , map_ready_(false)
  {
    move_base_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("/move_base");
    sub_status_ = node_.subscribe(
        "/planner_3d/status", 10, &DynamicParameterChangeTest::cbStatus, this);
    path_ = nullptr;
    planner_3d_client_.reset(
        new dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>("/planner_3d/"));
    sub_path_ = node_.subscribe("path", 1, &DynamicParameterChangeTest::cbPath, this);
    pub_map_overlay_ = node_.advertise<nav_msgs::OccupancyGrid>("map_overlay", 1, true);
  }

  bool setup()
  {
    if (!move_base_->waitForServer(ros::Duration(30.0)))
    {
      return false;
    }
    const ros::Time deadline = ros::Time::now() + ros::Duration(10.0);
    ros::ServiceClient srv_plan =
        node_.serviceClient<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>(
            "/planner_3d/make_plan");

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
      }
      ros::Duration(1).sleep();
      ros::spinOnce();
    }

    planner_cspace::Planner3DConfig config = planner_cspace::Planner3DConfig::__getDefault__();
    config.cost_in_place_turn = 3.0;
    config.min_curve_radius = 0.4;
    config.max_vel = 0.3;
    config.max_ang_vel = 1.0;
    config.keep_a_part_of_previous_path = true;
    return true;
  }

  void spin()
  {
    nav_msgs::OccupancyGrid map_overlay;
    map_overlay.header.frame_id = "map";
    map_overlay.info.resolution = 0.1;
    map_overlay.info.width = 32;
    map_overlay.info.height = 32;
    map_overlay.info.origin.position.x = 0.0;
    map_overlay.info.origin.position.y = 0.0;
    map_overlay.info.origin.position.z = 0.0;
    map_overlay.info.origin.orientation.x = 0.0;
    map_overlay.info.origin.orientation.y = 0.0;
    map_overlay.info.origin.orientation.z = 0.0;
    map_overlay.info.origin.orientation.w = 1.0;
    map_overlay.data.resize(map_overlay.info.width * map_overlay.info.height, 0);
    pub_map_overlay_.publish(map_overlay);
    // map_overlay.data[12 + 10 * map_overlay.info.width] = 100;

    sendGoalAndWaitForPath();
    /*
    planner_cspace::Planner3DConfig config;
    planner_3d_client_->getCurrentConfiguration(config, ros::Duration(0.1));
    config.keep_a_part_of_previous_path = false;
    planner_3d_client_->setConfiguration(config);
*/
    sendGoalAndWaitForPath();
    ros::Rate r(5);
    int i = 0;
    while (ros::ok())
    {
      pub_map_overlay_.publish(map_overlay);
      ros::spinOnce();
      r.sleep();
      ++i;

      if (i > 20)
      {
        ROS_WARN("ADDED");
        map_overlay.data[13 + 5 * map_overlay.info.width] = 100;
      }
      //      map_overlay.data[13 + (i / 10 + 5) * map_overlay.info.width] = 100;

      // ROS_WARN("i: %d", 12 + i / 15);
      // map_overlay.data[12 + i / 15 + 2 * map_overlay.info.width] = 100;
    }
  }

protected:
  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    path_ = msg;
    std::cerr << "Path received: " << path_->poses.size() << std::endl;
    for (size_t i = 0; i < path_->poses.size(); ++i)
    {
      const auto& pose = path_->poses[i];
      std::cerr << " #" << i << " " << pose.pose.position.x << ", "
                << pose.pose.position.y << "," << tf2::getYaw(pose.pose.orientation) << std::endl;
    }
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

  void sendGoalAndWaitForPath()
  {
    path_ = nullptr;
    const ros::Time start_time = ros::Time::now();
    ros::Time deadline = start_time + ros::Duration(1.0);
    move_base_->sendGoal(CreateGoalInFree());
    while (ros::ok())
    {
      if (path_ && (path_->header.stamp > start_time) && (path_->poses.size() > 0))
      {
        break;
      }
      ros::spinOnce();
    }
  }

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
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_;
  planner_cspace_msgs::PlannerStatus::ConstPtr planner_status_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  bool map_ready_;

  ros::Subscriber sub_path_;
  nav_msgs::Path::ConstPtr path_;
  std::unique_ptr<dynamic_reconfigure::Client<planner_cspace::Planner3DConfig>> planner_3d_client_;
  ros::Publisher pub_map_overlay_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dynamic_parameter_changer");

  DynamicParameterChangeTest node;
  if (!node.setup())
  {
    return -1;
  }
  node.spin();

  return 0;
}
