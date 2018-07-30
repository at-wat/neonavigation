/*
 * Copyright (c) 2018, the neonavigation authors
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <gtest/gtest.h>

TEST(TrackOdometry, OdomImuFusion)
{
  nav_msgs::Odometry::ConstPtr odom;

  const boost::function<void(const nav_msgs::Odometry::ConstPtr &)> cb_odom =
      [&odom](const nav_msgs::Odometry::ConstPtr &msg) -> void
  {
    odom = msg;
  };

  ros::NodeHandle nh("");
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom_raw", 10);
  ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
  ros::Subscriber sub_odom = nh.subscribe("odom", 2, cb_odom);

  nav_msgs::Odometry odom_raw;
  odom_raw.header.frame_id = "odom";
  odom_raw.pose.pose.orientation.w = 1;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  imu.orientation.w = 1;
  imu.linear_acceleration.z = 9.8;

  const float dt = 0.02;
  ros::Rate rate(1.0 / dt);
  size_t cnt = 0;

  // Go forward for 1m
  ros::Duration(1).sleep();
  odom_raw.twist.twist.linear.x = 0.5;
  while (ros::ok())
  {
    const double yaw = tf::getYaw(odom_raw.pose.pose.orientation);
    odom_raw.pose.pose.position.x += cos(yaw) * dt * odom_raw.twist.twist.linear.x;
    odom_raw.pose.pose.position.y += sin(yaw) * dt * odom_raw.twist.twist.linear.x;
    imu.header.stamp = odom_raw.header.stamp = ros::Time::now();
    pub_odom.publish(odom_raw);
    pub_imu.publish(imu);

    rate.sleep();
    ros::spinOnce();
    if (++cnt >= 100)
      break;
  }
  ASSERT_TRUE(ros::ok());

  odom_raw.twist.twist.linear.x = 0.0;
  pub_odom.publish(odom_raw);
  pub_imu.publish(imu);

  rate.sleep();
  ros::spinOnce();
  ASSERT_NEAR(odom->pose.pose.position.x, 1.0, 1e-3);
  ASSERT_NEAR(odom->pose.pose.position.y, 0.0, 1e-3);
  ASSERT_NEAR(odom->pose.pose.position.z, 0.0, 1e-3);
  ASSERT_NEAR(tf::getYaw(odom->pose.pose.orientation), 0.0, 1e-3);

  // Turn 90 degrees with 10% of odometry errors
  ros::Duration(1).sleep();
  imu.angular_velocity.z = M_PI * 0.25;
  odom_raw.twist.twist.angular.z = imu.angular_velocity.z * 0.9;  // Odometry with error
  cnt = 0;
  rate.sleep();
  while (ros::ok())
  {
    odom_raw.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(
            tf::getYaw(odom_raw.pose.pose.orientation) + dt * odom_raw.twist.twist.angular.z);
    imu.orientation =
        tf::createQuaternionMsgFromYaw(
            tf::getYaw(imu.orientation) + dt * imu.angular_velocity.z);
    imu.header.stamp = odom_raw.header.stamp = ros::Time::now();
    pub_odom.publish(odom_raw);
    pub_imu.publish(imu);

    rate.sleep();
    ros::spinOnce();
    if (++cnt >= 100)
      break;
  }
  ASSERT_TRUE(ros::ok());

  imu.angular_velocity.z = 0;
  odom_raw.twist.twist.angular.z = 0;
  imu.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
  pub_odom.publish(odom_raw);
  pub_imu.publish(imu);

  rate.sleep();
  ros::spinOnce();

  ASSERT_NEAR(odom->pose.pose.position.x, 1.0, 1e-2);
  ASSERT_NEAR(odom->pose.pose.position.y, 0.0, 1e-2);
  ASSERT_NEAR(odom->pose.pose.position.z, 0.0, 1e-2);
  ASSERT_NEAR(tf::getYaw(odom->pose.pose.orientation), M_PI / 2, 1e-2);

  // Go forward for 1m
  odom_raw.twist.twist.linear.x = 0.5;
  cnt = 0;
  rate.sleep();
  while (ros::ok())
  {
    const double yaw = tf::getYaw(odom_raw.pose.pose.orientation);
    odom_raw.pose.pose.position.x += cos(yaw) * dt * odom_raw.twist.twist.linear.x;
    odom_raw.pose.pose.position.y += sin(yaw) * dt * odom_raw.twist.twist.linear.x;
    imu.header.stamp = odom_raw.header.stamp = ros::Time::now();
    pub_odom.publish(odom_raw);
    pub_imu.publish(imu);

    rate.sleep();
    ros::spinOnce();
    if (++cnt >= 100)
      break;
  }
  ASSERT_TRUE(ros::ok());

  odom_raw.twist.twist.linear.x = 0.0;
  pub_odom.publish(odom_raw);
  pub_imu.publish(imu);

  rate.sleep();
  ros::spinOnce();

  ASSERT_NEAR(odom->pose.pose.position.x, 1.0, 5e-2);
  ASSERT_NEAR(odom->pose.pose.position.y, 1.0, 5e-2);
  ASSERT_NEAR(odom->pose.pose.position.z, 0.0, 5e-2);
  ASSERT_NEAR(tf::getYaw(odom->pose.pose.orientation), M_PI / 2, 1e-2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_track_odometry");

  return RUN_ALL_TESTS();
}
