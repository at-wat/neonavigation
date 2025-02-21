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

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <test_safety_limiter_base.h>

#include <gtest/gtest.h>

TEST_F(SafetyLimiterTest, SafetyLimitLinearSimpleSimulationWithMargin)
{
  const float dt = 0.02;
  const double ax = 2.0;  // [m/ss]
  double v = 0.0;         // [m/s]
  rclcpp::Rate wait(1.0 / dt);

  const float velocities[] = {-0.8, -0.4, 0.4, 0.8};
  for (const float vel : velocities)
  {
    float x = 0;
    bool stopped = false;
    const auto cb_cmd_vel = [dt, ax, &x, &v, &stopped](const geometry_msgs::msg::Twist& msg) -> void
    {
      if (msg.linear.x >= v)
      {
        v = std::min(v + ax * dt, msg.linear.x);
      }
      else
      {
        v = std::max(v - ax * dt, msg.linear.x);
      }
      x += dt * v;

      if (std::abs(v) < 1e-4 && std::abs(x) > 0.5)
      {
        stopped = true;
      }
    };
    auto sub_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, cb_cmd_vel);

    int count_after_stop = 10;
    for (float t = 0; t < 10.0 && rclcpp::ok() && count_after_stop > 0; t += dt)
    {
      if (vel > 0)
        publishSinglePointPointcloud2(1.5 - x, 0, 0, "base_link", now());
      else
        publishSinglePointPointcloud2(-3.5 - x, 0, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(vel, 0.0);
      broadcastTF("odom", "base_link", x, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (stopped)
      {
        count_after_stop--;
      }
    }
    // margin is set to 0.2
    if (vel > 0)
    {
      EXPECT_GT(1.4, x) << "vel: " << vel;  // Collision point - margin
      EXPECT_LT(1.3, x) << "vel: " << vel;  // Collision point - margin * 2
    }
    else
    {
      EXPECT_LT(-1.4, x) << "vel: " << vel;  // Collision point + margin
      EXPECT_GT(-1.3, x) << "vel: " << vel;  // Collision point + margin * 2
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
