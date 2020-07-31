/*
 * Copyright (c) 2018-2020, the neonavigation authors
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

#ifndef TRAJECTORY_TRACKER_TEST_H
#define TRAJECTORY_TRACKER_TEST_H

#include <algorithm>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/TrajectoryTrackerStatus.h>

#include <gtest/gtest.h>

class TrajectoryTrackerTest : public ::testing::Test
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_status_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_vel_;
  tf2_ros::TransformBroadcaster tfb_;
  ros::Time cmd_vel_time_;
  ros::Time trans_stamp_last_;

  ros::Time initial_cmd_vel_time_;
  int cmd_vel_count_;

protected:
  std_msgs::Header last_path_header_;

private:
  void cbStatus(const trajectory_tracker_msgs::TrajectoryTrackerStatus::ConstPtr& msg)
  {
    status_ = msg;
  }
  void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
  {
    const ros::Time now = ros::Time::now();
    if (cmd_vel_time_ == ros::Time(0))
      cmd_vel_time_ = now;
    const float dt = std::min((now - cmd_vel_time_).toSec(), 0.1);

    yaw_ += msg->angular.z * dt;
    pos_ += Eigen::Vector2d(std::cos(yaw_), std::sin(yaw_)) * msg->linear.x * dt;
    cmd_vel_time_ = now;
    cmd_vel_ = msg;
    ++cmd_vel_count_;
  }

public:
  Eigen::Vector2d pos_;
  double yaw_;
  trajectory_tracker_msgs::TrajectoryTrackerStatus::ConstPtr status_;
  geometry_msgs::Twist::ConstPtr cmd_vel_;

  TrajectoryTrackerTest()
    : nh_("")
  {
    sub_cmd_vel_ = nh_.subscribe(
        "cmd_vel", 1, &TrajectoryTrackerTest::cbCmdVel, this);
    sub_status_ = nh_.subscribe(
        "trajectory_tracker/status", 1, &TrajectoryTrackerTest::cbStatus, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
    pub_path_vel_ = nh_.advertise<trajectory_tracker_msgs::PathWithVelocity>("path_velocity", 1, true);
  }
  void initState(const Eigen::Vector2d& pos, const float yaw)
  {
    // Wait trajectory_tracker node
    ros::Rate rate(10);
    const auto start = ros::WallTime::now();
    while (ros::ok())
    {
      nav_msgs::Path path;
      path.header.frame_id = "odom";
      path.header.stamp = ros::Time::now();
      pub_path_.publish(path);

      yaw_ = yaw;
      pos_ = pos;
      publishTransform();

      rate.sleep();
      ros::spinOnce();
      if (status_ &&
          status_->status != trajectory_tracker_msgs::TrajectoryTrackerStatus::FOLLOWING)
        break;
      ASSERT_LT(ros::WallTime::now(), start + ros::WallDuration(10.0))
          << "trajectory_tracker status timeout, status: "
          << (status_ ? std::to_string(static_cast<int>(status_->status)) : "none");
    }
  }
  void waitUntilStart(const std::function<void()> func = nullptr)
  {
    ros::Rate rate(50);
    const auto start = ros::WallTime::now();
    initial_cmd_vel_time_ = ros::Time::now();
    cmd_vel_count_ = 0;
    while (ros::ok())
    {
      if (func)
        func();

      publishTransform();
      rate.sleep();
      ros::spinOnce();
      if (status_ &&
          status_->status == trajectory_tracker_msgs::TrajectoryTrackerStatus::FOLLOWING)
        break;
      ASSERT_LT(ros::WallTime::now(), start + ros::WallDuration(10.0))
          << "trajectory_tracker status timeout, status: "
          << (status_ ? std::to_string(static_cast<int>(status_->status)) : "none");
    }
  }
  void publishPath(const std::vector<Eigen::Vector3d>& poses)
  {
    nav_msgs::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    for (const Eigen::Vector3d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      path.poses.push_back(pose);
    }
    pub_path_.publish(path);
    last_path_header_ = path.header;
  }
  void publishPathVelocity(const std::vector<Eigen::Vector4d>& poses)
  {
    trajectory_tracker_msgs::PathWithVelocity path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    for (const Eigen::Vector4d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      trajectory_tracker_msgs::PoseStampedWithVelocity pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      pose.linear_velocity.x = p[3];

      path.poses.push_back(pose);
    }
    // needs sleep to prevent that the empty path from initState arrives later.
    ros::Duration(0.5).sleep();
    pub_path_vel_.publish(path);
    last_path_header_ = path.header;
  }
  void publishTransform()
  {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_, Eigen::Vector3d(0, 0, 1)));
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.header.stamp = ros::Time::now() + ros::Duration(0.1);
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = pos_[0];
    trans.transform.translation.y = pos_[1];
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();

    if (trans.header.stamp != trans_stamp_last_)
    {
      tfb_.sendTransform(trans);
    }
    trans_stamp_last_ = trans.header.stamp;
  }
  double getCmdVelFrameRate() const
  {
    return cmd_vel_count_ / (cmd_vel_time_ - initial_cmd_vel_time_).toSec();
  }
};

#endif  // TRAJECTORY_TRACKER_TEST_H
