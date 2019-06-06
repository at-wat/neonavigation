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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include <test_safety_limiter_base.h>

TEST_F(SafetyLimiterTest, Timeouts)
{
  geometry_msgs::Twist::ConstPtr cmd_vel;
  const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
      [&cmd_vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
  {
    cmd_vel = msg;
  };
  ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

  ros::Rate wait(10.0);

  for (int with_cloud = 0; with_cloud < 3; ++with_cloud)
  {
    for (int with_watchdog_reset = 0; with_watchdog_reset < 2; ++with_watchdog_reset)
    {
      ros::Duration(0.3).sleep();

      cmd_vel.reset();
      for (int i = 0; i < 20; ++i)
      {
        ASSERT_TRUE(ros::ok());

        if (with_watchdog_reset > 0)
          publishWatchdogReset();

        if (with_cloud > 0)
        {
          // cloud must have timestamp, otherwise the robot stops
          if (with_cloud > 1)
            publishSinglePointPointcloud2(1000, 1000, 0, "base_link", ros::Time::now());
          else
            publishSinglePointPointcloud2(1000, 1000, 0, "base_link", ros::Time());
        }

        const float vel = 0.1;
        const float ang_vel = 0.2;
        publishTwist(vel, ang_vel);

        wait.sleep();
        ros::spinOnce();

        if (i > 5 && cmd_vel)
        {
          if (with_watchdog_reset > 0 && with_cloud > 1)
          {
            ASSERT_EQ(cmd_vel->linear.x, vel);
            ASSERT_EQ(cmd_vel->linear.y, 0.0);
            ASSERT_EQ(cmd_vel->linear.z, 0.0);
            ASSERT_EQ(cmd_vel->angular.x, 0.0);
            ASSERT_EQ(cmd_vel->angular.y, 0.0);
            ASSERT_EQ(cmd_vel->angular.z, ang_vel);
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

TEST_F(SafetyLimiterTest, CloudBuffering)
{
  ros::Rate wait(60.0);

  // Skip initial state
  for (int i = 0; i < 30 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(0.5, 0, 0, "base_link", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  bool received = false;
  bool failed = false;
  bool en = false;
  // 1.0 m/ss, obstacle at 0.5 m: limited to 1.0 m/s (t_margin: 0)
  const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
      [&received, &failed, &en](const geometry_msgs::Twist::ConstPtr& msg) -> void
  {
    if (!en)
      return;
    received = true;
    failed = true;
    ASSERT_NEAR(msg->linear.x, 1.0, 1e-1);
    failed = false;
  };
  ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

  for (int i = 0; i < 60 * 6 && ros::ok() && !failed; ++i)
  {
    // enable check after two cycles of safety_limiter
    if (i > 8)
      en = true;
    // safety_limiter: 15 hz, cloud publish: 60 hz
    // safety_limiter must check 4 buffered clouds
    // 1/3 of pointclouds have collision point
    if ((i % 3) == 0)
      publishSinglePointPointcloud2(0.5, 0, 0, "base_link", ros::Time::now());
    else
      publishSinglePointPointcloud2(10, 0, 0, "base_link", ros::Time::now());

    publishWatchdogReset();
    publishTwist(2.0, 0);

    wait.sleep();
    ros::spinOnce();
  }
  ASSERT_TRUE(received);
}

TEST_F(SafetyLimiterTest, SafetyLimitLinear)
{
  ros::Rate wait(20.0);

  // Skip initial state
  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(0.5, 0, 0, "base_link", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  for (float vel = 0.0; vel < 2.0; vel += 0.4)
  {
    // 1.0 m/ss, obstacle at 0.5 m: limited to 1.0 m/s
    bool received = false;
    bool failed = false;
    bool en = false;
    const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
        [&received, &failed, &en, vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
    {
      if (!en)
        return;
      const float expected_vel = std::min<float>(vel, 1.0);
      received = true;
      failed = true;
      ASSERT_NEAR(msg->linear.x, expected_vel, 1e-1);
      failed = false;
    };
    ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

    for (int i = 0; i < 10 && ros::ok() && !failed; ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(0.5, 0, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist(vel, ((i % 5) - 2.0) * 0.01);

      wait.sleep();
      ros::spinOnce();
    }
    ASSERT_TRUE(received);
    sub_cmd_vel.shutdown();
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitLinearBackward)
{
  ros::Rate wait(20.0);

  // Skip initial state
  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(-2.5, 0, 0, "base_link", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  for (float vel = 0.0; vel > -2.0; vel -= 0.4)
  {
    // 1.0 m/ss, obstacle at -2.5 m: limited to -1.0 m/s
    bool received = false;
    bool failed = false;
    bool en = false;
    const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
        [&received, &failed, &en, vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
    {
      if (!en)
        return;
      const float expected_vel = std::max<float>(vel, -1.0);
      received = true;
      failed = true;
      ASSERT_NEAR(msg->linear.x, expected_vel, 1e-1);
      failed = false;
    };
    ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

    for (int i = 0; i < 10 && ros::ok() && !failed; ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-2.5, 0, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist(vel, ((i % 5) - 2.0) * 0.01);

      wait.sleep();
      ros::spinOnce();
    }
    ASSERT_TRUE(received);
    sub_cmd_vel.shutdown();
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitLinearEscape)
{
  ros::Rate wait(20.0);

  // Skip initial state
  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(-0.05, 0, 0, "base_link", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  const float vel_ref[] = { -0.2, -0.4, 0.2, 0.4 };
  for (const float vel : vel_ref)
  {
    // 1.0 m/ss, obstacle at -0.05 m (already in collision): escape motion must be allowed
    bool received = false;
    bool failed = false;
    bool en = false;
    const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
        [&received, &failed, &en, vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
    {
      if (!en)
        return;
      received = true;
      failed = true;
      if (vel < 0)
        // escaping from collision must be allowed
        ASSERT_NEAR(msg->linear.x, vel, 1e-1);
      else
        // colliding motion must be limited
        ASSERT_NEAR(msg->linear.x, 0.0, 1e-1);
      failed = false;
    };
    ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

    for (int i = 0; i < 10 && ros::ok() && !failed; ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-0.05, 0, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist(vel, 0.0);

      wait.sleep();
      ros::spinOnce();
    }
    ASSERT_TRUE(received);
    sub_cmd_vel.shutdown();
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitAngular)
{
  ros::Rate wait(20.0);

  // Skip initial state
  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(-1, -1, 0, "base_link", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  for (float vel = 0.0; vel < M_PI; vel += M_PI / 10)
  {
    // pi/2 rad/ss, obstacle at pi/4 rad: limited to pi/2 rad/s
    bool received = false;
    bool failed = false;
    bool en = false;
    const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
        [&received, &failed, &en, vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
    {
      if (!en)
        return;
      const float expected_vel = std::min<float>(vel, M_PI / 2);
      received = true;
      failed = true;
      ASSERT_NEAR(msg->angular.z, expected_vel, M_PI / 20);
      failed = false;
    };
    ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

    for (int i = 0; i < 10 && ros::ok() && !failed; ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-1, -1.1, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist((i % 3) * 0.01, vel);

      wait.sleep();
      ros::spinOnce();
    }
    ASSERT_TRUE(received);
    sub_cmd_vel.shutdown();
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitAngularEscape)
{
  ros::Rate wait(20.0);

  // Skip initial state
  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(-1, -0.09, 0, "base_link", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  const float vel_ref[] = { -0.2, -0.4, 0.2, 0.4 };
  for (const float vel : vel_ref)
  {
    // already colliding at rear-right side: only positive rotation must be allowed
    bool received = false;
    bool failed = false;
    bool en = false;
    const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
        [&received, &failed, &en, vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
    {
      if (!en)
        return;
      received = true;
      failed = true;
      if (vel < 0)
        // escaping from collision must be allowed
        ASSERT_NEAR(msg->angular.z, vel, 1e-1);
      else
        // colliding motion must be limited
        ASSERT_NEAR(msg->angular.z, 0.0, 1e-1);
      failed = false;
    };
    ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

    for (int i = 0; i < 10 && ros::ok() && !failed; ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-1, -0.09, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist(0.0, vel);

      wait.sleep();
      ros::spinOnce();
    }
    ASSERT_TRUE(received);
    sub_cmd_vel.shutdown();
  }
}

TEST_F(SafetyLimiterTest, NoCollision)
{
  ros::Rate wait(20.0);

  for (float vel = 0.0; vel < 1.0; vel += 0.2)
  {
    for (float ang_vel = 0.0; ang_vel < 1.0; ang_vel += 0.2)
    {
      bool received = false;
      bool failed = false;
      bool en = false;
      const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_cmd_vel =
          [&received, &failed, &en, vel, ang_vel](const geometry_msgs::Twist::ConstPtr& msg) -> void
      {
        failed = true;
        received = true;
        ASSERT_NEAR(msg->linear.x, vel, 1e-3);
        ASSERT_NEAR(msg->angular.z, ang_vel, 1e-3);
        failed = false;
      };
      ros::Subscriber sub_cmd_vel = nh_.subscribe("cmd_vel", 1, cb_cmd_vel);

      for (int i = 0; i < 10 && ros::ok() && !failed; ++i)
      {
        if (i > 5)
          en = true;
        publishSinglePointPointcloud2(1000, 1000, 0, "base_link", ros::Time::now());
        publishWatchdogReset();
        publishTwist(vel, ang_vel);

        wait.sleep();
        ros::spinOnce();
      }
      ASSERT_TRUE(received);
      sub_cmd_vel.shutdown();
    }
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitLinearSimpleSimulation)
{
  const float dt = 0.05;
  ros::Rate wait(1.0 / dt);

  for (float vel = 0.5; vel < 1.0; vel += 0.2)
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

    for (int i = 0; i < lround(10.0 / dt) && ros::ok() && !stop; ++i)
    {
      publishSinglePointPointcloud2(1.0 - x, 0, 0, "base_link", ros::Time::now());
      publishWatchdogReset();
      publishTwist(vel, 0.0);

      wait.sleep();
      ros::spinOnce();
    }
    EXPECT_GT(1.01, x);
    EXPECT_LT(0.95, x);
    sub_cmd_vel.shutdown();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter");

  return RUN_ALL_TESTS();
}
