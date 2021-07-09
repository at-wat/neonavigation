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

#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#include <test_safety_limiter_base.h>

#include <gtest/gtest.h>

TEST_F(SafetyLimiterTest, SafetyLimitLinearSimpleSimulationWithMargin)
{
  const float dt = 0.02;
  ros::Rate wait(1.0 / dt);

  const float velocities[] = {-0.8, -0.4, 0.4, 0.8};
  for (const float vel : velocities)
  {
    float x = 0;
    bool stop = false;
    const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
        [dt, &x, &stop](const geometry_msgs::Twist::ConstPtr& msg) -> void
    {
      if (std::abs(msg->linear.x) < 1e-4 && x > 0.5)
        stop = true;

      x += dt * msg->linear.x;
    };
    ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

    for (int i = 0; i < std::lround(10.0 / dt) && ros::ok() && !stop; ++i)
    {
      if (vel > 0)
        publishSinglePointPointcloud2(1.0 - x, 0, 0, "base_link", ros::Time::now());
      else
        publishSinglePointPointcloud2(-3.0 - x, 0, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist(vel, 0.0);
      broadcastTF("odom", "base_link", x, 0.0);

      wait.sleep();
      ros::spinOnce();
    }
    // margin is set to 0.2
    if (vel > 0)
    {
      EXPECT_GT(0.81, x);
      EXPECT_LT(0.75, x);
    }
    else
    {
      EXPECT_LT(-0.81, x);
      EXPECT_GT(-0.75, x);
    }
    sub_cmd_vel.shutdown();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter");

  return RUN_ALL_TESTS();
}
