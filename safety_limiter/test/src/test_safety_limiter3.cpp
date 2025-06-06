/*
 * Copyright (c) 2018-2025, the neonavigation authors
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

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#include <test_safety_limiter_base.h>

#include <gtest/gtest.h>

TEST_F(SafetyLimiterTest, SafetyLimitCentrifugalAccel)
{
  ros::Rate wait(20.0);

  bool received = false;
  const double vel = 0.8;
  const double ang_vel = 0.4;

  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    publishEmptyPointPointcloud2("base_link", ros::Time::now());
    publishWatchdogReset();
    publishTwist(vel, ang_vel);
    broadcastTF("odom", "base_link", 0.0, 0.0);
    wait.sleep();
    ros::spinOnce();

    if (cmd_vel_)
    {
      received = true;
      const double current_speed = std::hypot(cmd_vel_->linear.x, cmd_vel_->linear.y);
      ASSERT_NEAR(cmd_vel_->linear.x, std::sqrt(0.1), 1e-2)
          << " Raw: " << vel << " i: " << i
          << " Limited: " << cmd_vel_->linear.x;
      ASSERT_NEAR(current_speed, std::sqrt(0.4), 1e-2)
          << " Raw: " << ang_vel << " i: " << i
          << " Limited: " << cmd_vel_->angular.z;
    }
  }

  ASSERT_TRUE(hasDiag());
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, diag_->status[0].level)
      << "message: " << diag_->status[0].message;

  ASSERT_TRUE(hasStatus());
  EXPECT_TRUE(status_->is_cloud_available);
  EXPECT_FALSE(status_->has_watchdog_timed_out);
  EXPECT_EQ(status_->stuck_started_since, ros::Time(0));

  ASSERT_TRUE(received);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter");

  return RUN_ALL_TESTS();
}
