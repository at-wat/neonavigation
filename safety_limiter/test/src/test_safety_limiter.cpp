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
#include <std_msgs/Empty.h>

#include <gtest/gtest.h>

TEST(SafetyLimiter, testSafetyTimeouts)
{
  ros::NodeHandle nh("");

  geometry_msgs::Twist::ConstPtr cmd_vel;
  const boost::function<void(const geometry_msgs::Twist::ConstPtr &)> cb_cmd_vel =
      [&cmd_vel](const geometry_msgs::Twist::ConstPtr &msg) -> void
  {
    cmd_vel = msg;
  };
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/safety_limiter/cmd_vel_in", 1);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/safety_limiter/cloud", 1);
  ros::Publisher pub_watchdog = nh.advertise<std_msgs::Empty>("/safety_limiter/watchdog_reset", 1);
  ros::Subscriber sub_cmd_vel = nh.subscribe("/safety_limiter/cmd_vel_out", 1, cb_cmd_vel);

  ros::Rate wait(10.0);

  for (size_t with_cloud = 0; with_cloud < 3; ++with_cloud)
  {
    for (size_t with_watchdog_reset = 0; with_watchdog_reset < 2; ++with_watchdog_reset)
    {
      ros::Duration(0.3).sleep();

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
          cloud.fields.resize(3);
          cloud.fields[0].name = "x";
          cloud.fields[1].name = "y";
          cloud.fields[2].name = "z";
          if (with_cloud > 1)
          {
            // cloud must have timestamp, otherwise the robot stops
            cloud.header.stamp = ros::Time::now();
            cloud.header.frame_id = "base_link";
          }
          pub_cloud.publish(cloud);
        }

        geometry_msgs::Twist cmd_vel_out;
        cmd_vel_out.linear.x = 1.0;
        cmd_vel_out.angular.z = 2.0;
        pub_cmd_vel.publish(cmd_vel_out);

        wait.sleep();
        ros::spinOnce();

        if (i > 5)
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter");

  return RUN_ALL_TESTS();
}
