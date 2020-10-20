/*
 * Copyright (c) 2018-2019, the neonavigation authors
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

#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtest/gtest.h>

class TrackOdometryTest : public ::testing::TestWithParam<const char*>
{
protected:
  std::vector<nav_msgs::Odometry> odom_msg_buffer_;

public:
  void initializeNode(const std::string& ns)
  {
    ros::NodeHandle nh(ns);
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("odom_raw", 10);
    pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    sub_odom_ = nh.subscribe("odom", 10, &TrackOdometryTest::cbOdom, this);
  }
  bool initializeTrackOdometry(
      nav_msgs::Odometry& odom_raw,
      sensor_msgs::Imu& imu)
  {
    ros::Duration(0.1).sleep();
    ros::Rate rate(100);
    odom_ = nullptr;
    for (int i = 0; i < 1000 && ros::ok(); ++i)
    {
      odom_raw.header.stamp = ros::Time::now();
      imu.header.stamp = odom_raw.header.stamp + ros::Duration(0.0001);
      pub_odom_.publish(odom_raw);
      pub_imu_.publish(imu);
      rate.sleep();
      odom_.reset();
      ros::spinOnce();
      if (odom_ && i > 50)
        break;
    }
    return static_cast<bool>(odom_);
  }
  bool run(
      nav_msgs::Odometry& odom_raw,
      sensor_msgs::Imu& imu,
      const double dt, const int steps)
  {
    ros::Rate rate(1.0 / dt);
    int cnt(0);

    while (ros::ok())
    {
      tf2::Quaternion quat_odom;
      tf2::fromMsg(odom_raw.pose.pose.orientation, quat_odom);
      odom_raw.pose.pose.orientation =
          tf2::toMsg(
              quat_odom *
              tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), dt * odom_raw.twist.twist.angular.z));
      tf2::Quaternion quat_imu;
      tf2::fromMsg(imu.orientation, quat_imu);
      imu.orientation =
          tf2::toMsg(
              quat_imu *
              tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), dt * imu.angular_velocity.z));

      const double yaw = tf2::getYaw(odom_raw.pose.pose.orientation);
      odom_raw.pose.pose.position.x += cos(yaw) * dt * odom_raw.twist.twist.linear.x;
      odom_raw.pose.pose.position.y += sin(yaw) * dt * odom_raw.twist.twist.linear.x;

      stepAndPublish(odom_raw, imu, dt);

      rate.sleep();
      ros::spinOnce();
      if (++cnt >= steps)
        break;
    }
    flushOdomMsgs();
    return ros::ok();
  }
  void stepAndPublish(
      nav_msgs::Odometry& odom_raw,
      sensor_msgs::Imu& imu,
      const double dt)
  {
    odom_raw.header.stamp += ros::Duration(dt);
    imu.header.stamp += ros::Duration(dt);
    pub_imu_.publish(imu);

    // Buffer odom message to add delay and jitter.
    // Send odometry in half rate of IMU.
    ++odom_cnt_;
    if (odom_cnt_ % 2 == 0)
      odom_msg_buffer_.push_back(odom_raw);
    if (odom_msg_buffer_.size() > 10)
    {
      flushOdomMsgs();
    }
  }
  void flushOdomMsgs()
  {
    for (nav_msgs::Odometry& o : odom_msg_buffer_)
    {
      pub_odom_.publish(o);
    }
    odom_msg_buffer_.clear();
  }
  void waitAndSpinOnce()
  {
    nav_msgs::Odometry::ConstPtr odom_prev = odom_;
    while (true)
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      if (odom_prev == odom_)
      {
        // no more new messages
        return;
      }
      odom_prev = odom_;
    }
  }

protected:
  ros::Publisher pub_odom_;
  ros::Publisher pub_imu_;
  ros::Subscriber sub_odom_;
  nav_msgs::Odometry::ConstPtr odom_;
  size_t odom_cnt_;

  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
  {
    odom_ = msg;
  };
};

TEST_F(TrackOdometryTest, OdomImuFusion)
{
  initializeNode("");

  const double dt = 0.01;
  const int steps = 200;

  nav_msgs::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.w = 1;
  imu.linear_acceleration.z = 9.8;

  ASSERT_TRUE(initializeTrackOdometry(odom_raw, imu));

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 1.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.y, 0.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.z, 0.0, 1e-3);
  ASSERT_NEAR(tf2::getYaw(odom_->pose.pose.orientation), 0.0, 1e-3);

  // Turn 90 degrees with 10% of odometry errors
  imu.angular_velocity.z = M_PI * 0.25;
  odom_raw.twist.twist.angular.z = imu.angular_velocity.z * 1.1;  // Odometry with error
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  imu.angular_velocity.z = 0;
  odom_raw.twist.twist.angular.z = 0;
  imu.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI / 2));
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 1.0, 1e-2);
  ASSERT_NEAR(odom_->pose.pose.position.y, 0.0, 1e-2);
  ASSERT_NEAR(odom_->pose.pose.position.z, 0.0, 1e-2);
  ASSERT_NEAR(tf2::getYaw(odom_->pose.pose.orientation), M_PI / 2, 1e-2);

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 1.0, 5e-2);
  ASSERT_NEAR(odom_->pose.pose.position.y, 1.0, 5e-2);
  ASSERT_NEAR(odom_->pose.pose.position.z, 0.0, 5e-2);
  ASSERT_NEAR(tf2::getYaw(odom_->pose.pose.orientation), M_PI / 2, 1e-2);
}

TEST_P(TrackOdometryTest, ZFilterOff)
{
  const std::string ns_postfix(GetParam());
  initializeNode("no_z_filter" + ns_postfix);

  const double dt = 0.01;
  const int steps = 200;

  nav_msgs::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.y = sin(-M_PI / 4);
  imu.orientation.w = cos(-M_PI / 4);
  imu.linear_acceleration.x = -9.8;

  ASSERT_TRUE(initializeTrackOdometry(odom_raw, imu));

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.x, 0.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.y, 0.0, 1e-3);
  ASSERT_NEAR(odom_->pose.pose.position.z, 1.0, 1e-3);
}

TEST_P(TrackOdometryTest, ZFilterOn)
{
  const std::string ns_postfix(GetParam());
  initializeNode("z_filter" + ns_postfix);

  const double dt = 0.01;
  const int steps = 200;

  nav_msgs::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.y = sin(-M_PI / 4);
  imu.orientation.w = cos(-M_PI / 4);
  imu.linear_acceleration.x = -9.8;

  ASSERT_TRUE(initializeTrackOdometry(odom_raw, imu));

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  ASSERT_TRUE(run(odom_raw, imu, dt, steps));

  odom_raw.twist.twist.linear.x = 0.0;
  stepAndPublish(odom_raw, imu, dt);
  flushOdomMsgs();

  waitAndSpinOnce();
  ASSERT_NEAR(odom_->pose.pose.position.z, 1.0 - 1.0 / M_E, 5e-2);
}

INSTANTIATE_TEST_CASE_P(
    TrackOdometryTestInstance, TrackOdometryTest,
    ::testing::Values("", "_old_param"));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_track_odometry");

  return RUN_ALL_TESTS();
}
