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
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <trajectory_tracker_msgs/msg/path_with_velocity.hpp>
#include <trajectory_tracker_msgs/msg/trajectory_tracker_status.hpp>

#include <gtest/gtest.h>

class TrajectoryTrackerTest : public ::testing::Test, public rclcpp::Node
{
private:
  rclcpp::SubscriptionBase::SharedPtr sub_cmd_vel_;
  rclcpp::SubscriptionBase::SharedPtr sub_status_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<trajectory_tracker_msgs::msg::PathWithVelocity>::SharedPtr pub_path_vel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr sc_set_parameters_;

  tf2_ros::TransformBroadcaster tfb_;
  rclcpp::Time cmd_vel_time_;
  rclcpp::Time trans_stamp_last_;

  rclcpp::Time initial_cmd_vel_time_;
  int cmd_vel_count_;

  std::list<nav_msgs::msg::Odometry> odom_buffer_;

  std::shared_ptr<rclcpp::Clock> steady_clock_;

protected:
  std_msgs::msg::Header last_path_header_;
  double error_lin_;
  double error_large_lin_;
  double error_ang_;

private:
  void cbStatus(const trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::ConstSharedPtr& msg)
  {
    status_ = msg;
  }
  void cbCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
  {
    const auto now = this->now();

    if (cmd_vel_time_.seconds() == 0.0)
      cmd_vel_time_ = now;
    const float dt = std::min((now - cmd_vel_time_).seconds(), 0.1);

    yaw_ += msg->angular.z * dt;
    pos_ += Eigen::Vector2d(std::cos(yaw_), std::sin(yaw_)) * msg->linear.x * dt;
    cmd_vel_time_ = now;
    cmd_vel_ = msg;
    ++cmd_vel_count_;
  }

public:
  Eigen::Vector2d pos_;
  double yaw_;
  trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::ConstSharedPtr status_;
  geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_;
  rclcpp::Duration delay_;

  TrajectoryTrackerTest()
    : Node("trajectory_tracker_test")
    , tfb_(this)
    , steady_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
    , delay_(0, 0)
  {
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&TrajectoryTrackerTest::cbCmdVel, this, std::placeholders::_1));
    sub_status_ = create_subscription<trajectory_tracker_msgs::msg::TrajectoryTrackerStatus>(
        "trajectory_tracker/status", 1, std::bind(&TrajectoryTrackerTest::cbStatus, this, std::placeholders::_1));
    pub_path_ = create_publisher<nav_msgs::msg::Path>("path", 1);
    pub_path_vel_ = create_publisher<trajectory_tracker_msgs::msg::PathWithVelocity>("path_velocity", 1);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    sc_set_parameters_ = create_client<rcl_interfaces::srv::SetParameters>("/trajectory_tracker/set_parameters");

    declare_parameter("odom_delay", 0.0);
    declare_parameter("error_lin", 0.01);
    declare_parameter("error_large_lin", 0.1);
    declare_parameter("error_ang", 0.01);

    delay_ = rclcpp::Duration::from_seconds(get_parameter("odom_delay").as_double());
    get_parameter("error_lin", error_lin_);
    get_parameter("error_large_lin", error_large_lin_);
    get_parameter("error_ang", error_ang_);
    rclcpp::Rate wait(10);
    for (size_t i = 0; i < 100; ++i)
    {
      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (pub_path_->get_subscription_count() > 0)
        break;
    }
  }

  void initState(const Eigen::Vector2d& pos, const float yaw)
  {
    // Wait trajectory_tracker node
    rclcpp::Rate rate(10);
    const auto start = steady_clock_->now();
    while (rclcpp::ok())
    {
      nav_msgs::msg::Path path;
      path.header.frame_id = "odom";
      path.header.stamp = now();
      pub_path_->publish(path);

      yaw_ = yaw;
      pos_ = pos;
      publishTransform();

      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (status_ && status_->status != trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING)
        break;
      ASSERT_LT(steady_clock_->now(), start + rclcpp::Duration::from_seconds(10.0))
          << "trajectory_tracker status timeout, status: "
          << (status_ ? std::to_string(static_cast<int>(status_->status)) : "none");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  void waitUntilStart(const std::function<void()> func = nullptr)
  {
    rclcpp::Rate rate(50);
    const auto start = steady_clock_->now();
    while (rclcpp::ok())
    {
      if (func)
        func();

      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (status_ && status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::FOLLOWING)
        break;
      ASSERT_LT(steady_clock_->now(), start + rclcpp::Duration::from_seconds(10.0))
          << "trajectory_tracker status timeout, status: "
          << (status_ ? std::to_string(static_cast<int>(status_->status)) : "none");
    }
    initial_cmd_vel_time_ = now();
    cmd_vel_count_ = 0;
  }
  void publishPath(const std::vector<Eigen::Vector3d>& poses)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = now();

    for (const Eigen::Vector3d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      path.poses.push_back(pose);
    }
    pub_path_->publish(path);
    last_path_header_ = path.header;
  }
  void publishPathVelocity(const std::vector<Eigen::Vector4d>& poses)
  {
    trajectory_tracker_msgs::msg::PathWithVelocity path;
    path.header.frame_id = "odom";
    path.header.stamp = now();

    for (const Eigen::Vector4d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      trajectory_tracker_msgs::msg::PoseStampedWithVelocity pose;
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
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    pub_path_vel_->publish(path);
    last_path_header_ = path.header;
  }

  void publishTransform()
  {
    const auto now = this->now();
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_, Eigen::Vector3d(0, 0, 1)));

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = now;
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pos_[0];
    odom.pose.pose.position.y = pos_[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    if (cmd_vel_)
    {
      odom.twist.twist.linear = cmd_vel_->linear;
      odom.twist.twist.angular = cmd_vel_->angular;
    }
    publishTransform(odom);
  }

  void publishTransform(const nav_msgs::msg::Odometry& odom)
  {
    odom_buffer_.push_back(odom);
    const rclcpp::Time pub_time = rclcpp::Time(odom.header.stamp) - delay_;
    while (odom_buffer_.size() > 0)
    {
      nav_msgs::msg::Odometry odom = odom_buffer_.front();
      if (rclcpp::Time(odom.header.stamp) > pub_time)
        break;

      odom_buffer_.pop_front();

      if (odom.header.stamp != trans_stamp_last_)
      {
        geometry_msgs::msg::TransformStamped trans;
        trans.header.frame_id = odom.header.frame_id;
        trans.header.stamp = rclcpp::Time(odom.header.stamp) + rclcpp::Duration::from_seconds(0.1);
        trans.child_frame_id = odom.child_frame_id;
        trans.transform.translation.x = odom.pose.pose.position.x;
        trans.transform.translation.y = odom.pose.pose.position.y;
        trans.transform.rotation.x = odom.pose.pose.orientation.x;
        trans.transform.rotation.y = odom.pose.pose.orientation.y;
        trans.transform.rotation.z = odom.pose.pose.orientation.z;
        trans.transform.rotation.w = odom.pose.pose.orientation.w;

        tfb_.sendTransform(trans);
        pub_odom_->publish(odom);
      }
      trans_stamp_last_ = odom.header.stamp;
    }
  }

  double getCmdVelFrameRate() const
  {
    return cmd_vel_count_ / (cmd_vel_time_ - initial_cmd_vel_time_).seconds();
  }

  bool setConfig(const std::vector<rclcpp::Parameter>& params)
  {
    if (!sc_set_parameters_->wait_for_service(std::chrono::seconds(5)))
    {
      return false;
    }

    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto& param : params)
    {
      req->parameters.push_back(param.to_parameter_msg());
    }
    auto result = sc_set_parameters_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Successfully set parameter.");
      return true;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to set parameter.");
      return false;
    }
  }

  /*
    bool getConfig(ParamType& config) const
    {
      const ros::WallTime time_limit = ros::WallTime::now() + ros::WallDuration(10.0);
      while (time_limit > ros::WallTime::now())
      {
        if (dynamic_reconfigure_client_->getCurrentConfiguration(config, rclcpp::Duration::from_seconds(0.1)))
        {
          return true;
        }
        rclcpp::spin_some(node);
      }
      return false;
    }

    bool setConfig(const ParamType& config)
    {
      // Wait until parameter server becomes ready
      ParamType dummy;
      if (!getConfig(dummy))
      {
        return false;
      }
      return dynamic_reconfigure_client_->setConfiguration(config);
    }
    */
};

#endif  // TRAJECTORY_TRACKER_TEST_H

/*
namespace trajectory_tracker_msgs
{
std::ostream& operator<<(std::ostream& os, const TrajectoryTrackerStatus::ConstSharedPtr& msg)
{
  if (!msg)
  {
    os << "nullptr";
  }
  else
  {
    os << "  header: " << msg->header.stamp << " " << msg->header.frame_id << std::endl
       << "  distance_remains: " << msg->distance_remains << std::endl
       << "  angle_remains: " << msg->angle_remains << std::endl
       << "  status: " << msg->status << std::endl
       << "  path_header: " << msg->path_header.stamp << " " << msg->header.frame_id;
  }
  return os;
}
}  // namespace trajectory_tracker_msgs
*/