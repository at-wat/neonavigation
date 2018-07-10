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
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>

#include <gtest/gtest.h>

void GenerateSinglePointPointcloud2(
    sensor_msgs::PointCloud2 &cloud,
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

TEST(SafetyLimiter, testSafetyTimeouts)
{
  ros::NodeHandle nh("");

  geometry_msgs::Twist::ConstPtr cmd_vel;
  const boost::function<void(const geometry_msgs::Twist::ConstPtr &)> cb_cmd_vel =
      [&cmd_vel](const geometry_msgs::Twist::ConstPtr &msg) -> void
  {
    cmd_vel = msg;
  };
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel_in", 1);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Publisher pub_watchdog = nh.advertise<std_msgs::Empty>("watchdog_reset", 1);
  ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, cb_cmd_vel);

  ros::Rate wait(10.0);

  for (size_t with_cloud = 0; with_cloud < 3; ++with_cloud)
  {
    for (size_t with_watchdog_reset = 0; with_watchdog_reset < 2; ++with_watchdog_reset)
    {
      ros::Duration(0.3).sleep();

      cmd_vel.reset();
      for (size_t i = 0; i < 20; ++i)
      {
        ASSERT_TRUE(ros::ok());

        if (with_watchdog_reset > 0)
        {
          std_msgs::Empty watchdog_reset;
          pub_watchdog.publish(watchdog_reset);
        }

        if (with_cloud > 0)
        {
          sensor_msgs::PointCloud2 cloud;
          cloud.header.frame_id = "base_link";
          if (with_cloud > 1)
          {
            // cloud must have timestamp, otherwise the robot stops
            cloud.header.stamp = ros::Time::now();
          }
          GenerateSinglePointPointcloud2(cloud, 1000.0, 1000., 1000.);
          pub_cloud.publish(cloud);
        }

        geometry_msgs::Twist cmd_vel_out;
        cmd_vel_out.linear.x = 1.0;
        cmd_vel_out.angular.z = 2.0;
        pub_cmd_vel.publish(cmd_vel_out);

        wait.sleep();
        ros::spinOnce();

        if (i > 5 && cmd_vel)
        {
          if (with_watchdog_reset > 0 && with_cloud > 1)
          {
            ASSERT_EQ(cmd_vel->linear.x, cmd_vel_out.linear.x);
            ASSERT_EQ(cmd_vel->linear.y, cmd_vel_out.linear.y);
            ASSERT_EQ(cmd_vel->linear.z, cmd_vel_out.linear.z);
            ASSERT_EQ(cmd_vel->angular.x, cmd_vel_out.angular.x);
            ASSERT_EQ(cmd_vel->angular.y, cmd_vel_out.angular.y);
            ASSERT_EQ(cmd_vel->angular.z, cmd_vel_out.angular.z);
          }
          else
          {
            ASSERT_EQ(cmd_vel->linear.x, 0.0);
            ASSERT_EQ(cmd_vel->linear.y, 0.0);
            ASSERT_EQ(cmd_vel->linear.z, 0.0);
            ASSERT_EQ(cmd_vel->angular.x, 0.0);
            ASSERT_EQ(cmd_vel->angular.y, 0.0);
            ASSERT_EQ(cmd_vel->angular.z, 0.0);
          }
        }
      }
    }
  }
}

TEST(SafetyLimiter, testCloudBuffering)
{
  return;
  ros::NodeHandle nh("");
  ros::Rate wait(35.0);

  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel_in", 1);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  ros::Publisher pub_watchdog = nh.advertise<std_msgs::Empty>("watchdog_reset", 1);

  // Skip initial state
  for (size_t i = 0; i < 30 && ros::ok(); ++i)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "base_link";
    GenerateSinglePointPointcloud2(cloud, 0.5, 0, 0);
    pub_cloud.publish(cloud);

    wait.sleep();
    ros::spinOnce();
  }

  // 1.0 m/ss, obstacle at 0.5 m: limited to 1.0 m/s (t_margin: 0)
  const boost::function<void(const geometry_msgs::Twist::ConstPtr &)> cb_cmd_vel =
      [](const geometry_msgs::Twist::ConstPtr &msg) -> void
  {
    ASSERT_NEAR(msg->linear.x, 0.5, 1e-3);
  };
  ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, cb_cmd_vel);

  for (size_t i = 0; i < 35 * 6 && ros::ok(); ++i)
  {
    // safety_limiter: 10 hz, cloud publish: 31 hz
    geometry_msgs::Twist cmd_vel_out;
    if ((i % 3) == 0)
      cmd_vel_out.linear.x = 2.0;
    pub_cmd_vel.publish(cmd_vel_out);
    // 1/3 of pointclouds have collision point
    // safety_limiter must check 3 buffered clouds

    wait.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter");

  return RUN_ALL_TESTS();
}
