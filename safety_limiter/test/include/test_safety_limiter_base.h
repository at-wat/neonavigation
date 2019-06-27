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

#ifndef TEST_SAFETY_LIMITER_BASE_H
#define TEST_SAFETY_LIMITER_BASE_H

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>

#include <gtest/gtest.h>

namespace
{
inline void GenerateSinglePointPointcloud2(
    sensor_msgs::PointCloud2& cloud,
    const float x,
    const float y,
    const float z)
{
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  modifier.resize(1);
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
}
}  // namespace

class SafetyLimiterTest : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_watchdog_;

public:
  inline SafetyLimiterTest()
    : nh_()
  {
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_in", 1);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    pub_watchdog_ = nh_.advertise<std_msgs::Empty>("watchdog_reset", 1);
  }
  inline void publishWatchdogReset()
  {
    std_msgs::Empty watchdog_reset;
    pub_watchdog_.publish(watchdog_reset);
  }
  inline void publishSinglePointPointcloud2(
      const float x,
      const float y,
      const float z,
      const std::string frame_id,
      const ros::Time stamp)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
    GenerateSinglePointPointcloud2(cloud, x, y, z);
    pub_cloud_.publish(cloud);
  }
  inline void publishTwist(
      const float lin,
      const float ang)
  {
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = lin;
    cmd_vel_out.angular.z = ang;
    pub_cmd_vel_.publish(cmd_vel_out);
  }
};

#endif  // TEST_SAFETY_LIMITER_BASE_H
