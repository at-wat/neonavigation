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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <test_safety_limiter_base.h>

#include <gtest/gtest.h>

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

class RosRate : public rclcpp::RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RosRate)

  explicit RosRate(double rate, rclcpp::Node& node)
    : RosRate(duration_cast<nanoseconds>(duration<double>(1.0 / rate)), node)
  {
  }
  explicit RosRate(std::chrono::nanoseconds period, rclcpp::Node& node)
    : node_(node.get_node_base_interface())
    , clock_(node.get_clock())
    , period_(period)
  {
    last_interval_ = clock_->now();
  }

  virtual bool sleep()
  {
    // Time coming into sleep
    auto now = clock_->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_)
    {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (time_to_sleep <= std::chrono::seconds(0))
    {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_)
      {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    while (clock_->now() < next_interval)
    {
      rclcpp::spin_some(node_);
      clock_->sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  virtual bool is_steady() const
  {
    return false;
  }

  virtual void reset()
  {
    last_interval_ = clock_->now();
  }

  std::chrono::nanoseconds period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

private:
  RCLCPP_DISABLE_COPY(RosRate)

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
};

TEST_F(SafetyLimiterTest, Timeouts)
{
  resetMessages();
  RosRate wait(10.0, *this);

  for (int with_cloud = 0; with_cloud < 3; ++with_cloud)
  {
    for (int with_watchdog_reset = 0; with_watchdog_reset < 2; ++with_watchdog_reset)
    {
      const std::string test_condition =
          "with_watchdog_reset: " + std::to_string(with_watchdog_reset) + ", with_cloud: " + std::to_string(with_cloud);
      this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.3));

      for (int i = 0; i < 20; ++i)
      {
        ASSERT_TRUE(rclcpp::ok());

        if (with_watchdog_reset > 0)
          publishWatchdogReset();

        if (with_cloud > 0)
        {
          // cloud must have timestamp, otherwise the robot stops
          if (with_cloud > 1)
            publishSinglePointPointcloud2(1000, 1000, 0, "base_link", now());
          else
            publishSinglePointPointcloud2(1000, 1000, 0, "base_link", rclcpp::Time());
        }

        const float vel = 0.1;
        const float ang_vel = 0.2;
        publishTwist(vel, ang_vel);
        broadcastTF("odom", "base_link", 0.0, 0.0);

        wait.sleep();
        rclcpp::spin_some(get_node_base_interface());

        if (i > 5 && cmd_vel_received_)
        {
          if (with_watchdog_reset > 0 && with_cloud > 1)
          {
            ASSERT_EQ(cmd_vel_.linear.x, vel) << test_condition;
            ASSERT_EQ(cmd_vel_.linear.y, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.linear.z, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.angular.x, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.angular.y, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.angular.z, ang_vel) << test_condition;
          }
          else
          {
            ASSERT_EQ(cmd_vel_.linear.x, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.linear.y, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.linear.z, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.angular.x, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.angular.y, 0.0) << test_condition;
            ASSERT_EQ(cmd_vel_.angular.z, 0.0) << test_condition;
          }
        }
      }

      ASSERT_TRUE(hasStatus()) << test_condition;
      EXPECT_EQ(with_cloud > 1, status_.is_cloud_available) << test_condition;
      EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0)) << test_condition;

      if (with_watchdog_reset > 0 && with_cloud > 1)
      {
        ASSERT_TRUE(hasDiag()) << test_condition;
        EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
            << test_condition << ", "
            << "message: " << diag_.status[0].message;

        EXPECT_FALSE(status_.has_watchdog_timed_out) << test_condition;
      }
      else
      {
        ASSERT_TRUE(hasDiag()) << test_condition;
        EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::ERROR, diag_.status[0].level)
            << test_condition << ", "
            << "message: " << diag_.status[0].message;
        if (with_watchdog_reset == 0)
        {
          EXPECT_TRUE(status_.has_watchdog_timed_out) << test_condition;
        }
      }
    }
  }
}

TEST_F(SafetyLimiterTest, CloudBuffering)
{
  resetMessages();
  RosRate wait(60.0, *this);

  // Skip initial state
  for (int i = 0; i < 30 && rclcpp::ok(); ++i)
  {
    publishSinglePointPointcloud2(0.5, 0, 0, "base_link", now());
    publishWatchdogReset();

    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
  }

  bool received = false;
  bool en = false;
  // 1.0 m/ss, obstacle at 0.5 m: limited to 1.0 m/s (t_margin: 0)

  for (int i = 0; i < 60 * 6 && rclcpp::ok(); ++i)
  {
    // enable check after two cycles of safety_limiter
    if (i > 8)
      en = true;
    // safety_limiter: 15 hz, cloud publish: 60 hz
    // safety_limiter must check 4 buffered clouds
    // 3/4 of pointclouds have collision point
    if ((i % 4) == 0)
      publishSinglePointPointcloud2(10, 0, 0, "base_link", now());
    else
      publishSinglePointPointcloud2(0.5, 0, 0, "base_link", now());

    publishWatchdogReset();
    publishTwist(2.0, 0);
    broadcastTF("odom", "base_link", 0.0, 0.0);
    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
    if (en && cmd_vel_received_)
    {
      received = true;
      ASSERT_NEAR(cmd_vel_.linear.x, 1.0, 1e-1);
    }
  }
  ASSERT_TRUE(received);
}

TEST_F(SafetyLimiterTest, SafetyLimitLinear)
{
  resetMessages();
  RosRate wait(20.0, *this);

  // Skip initial state
  for (int i = 0; i < 10 && rclcpp::ok(); ++i)
  {
    publishSinglePointPointcloud2(0.5, 0, 0, "base_link", now());
    publishWatchdogReset();

    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
  }

  for (float vel = 0.0; vel < 2.0; vel += 0.4)
  {
    // 1.0 m/ss, obstacle at 0.5 m: limited to 1.0 m/s
    bool received = false;
    bool en = false;

    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(0.5, 0, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(vel, ((i % 5) - 2.0) * 0.01);
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (en && cmd_vel_received_)
      {
        received = true;
        const float expected_vel = std::min<float>(vel, 1.0);
        ASSERT_NEAR(cmd_vel_.linear.x, expected_vel, 1e-1);
      }
    }
    ASSERT_TRUE(hasDiag());
    EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
        << "message: " << diag_.status[0].message;

    ASSERT_TRUE(hasStatus());
    EXPECT_TRUE(status_.is_cloud_available);
    EXPECT_FALSE(status_.has_watchdog_timed_out);
    EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0));

    ASSERT_TRUE(received);
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitLinearBackward)
{
  resetMessages();
  RosRate wait(20.0, *this);

  // Skip initial state
  for (int i = 0; i < 10 && rclcpp::ok(); ++i)
  {
    publishSinglePointPointcloud2(-2.5, 0, 0, "base_link", now());
    publishWatchdogReset();

    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
  }

  for (float vel = 0.0; vel > -2.0; vel -= 0.4)
  {
    // 1.0 m/ss, obstacle at -2.5 m: limited to -1.0 m/s
    bool received = false;
    bool en = false;

    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-2.5, 0, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(vel, ((i % 5) - 2.0) * 0.01);
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (en && cmd_vel_received_)
      {
        received = true;
        const float expected_vel = std::max<float>(vel, -1.0);
        ASSERT_NEAR(cmd_vel_.linear.x, expected_vel, 1e-1);
      }
    }
    ASSERT_TRUE(hasDiag());
    EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
        << "message: " << diag_.status[0].message;

    ASSERT_TRUE(hasStatus());
    EXPECT_TRUE(status_.is_cloud_available);
    EXPECT_FALSE(status_.has_watchdog_timed_out);
    EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0));

    ASSERT_TRUE(received);
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitLinearEscape)
{
  resetMessages();
  RosRate wait(20.0, *this);

  // Skip initial state
  for (int i = 0; i < 10 && rclcpp::ok(); ++i)
  {
    publishSinglePointPointcloud2(-0.05, 0, 0, "base_link", now());
    publishWatchdogReset();

    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
  }

  const float vel_ref[] = {-0.2, -0.4, 0.2, 0.4};
  for (const float vel : vel_ref)
  {
    // 1.0 m/ss, obstacle at -0.05 m (already in collision): escape motion must be allowed
    bool received = false;
    bool en = false;

    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-0.05, 0, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(vel, 0.0);
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (en && cmd_vel_received_)
      {
        received = true;
        if (vel < 0)
        {
          // escaping from collision must be allowed
          ASSERT_NEAR(cmd_vel_.linear.x, vel, 1e-1);
        }
        else
        {
          // colliding motion must be limited
          ASSERT_NEAR(cmd_vel_.linear.x, 0.0, 1e-1);
        }
      }
    }
    if (vel < 0)
    {
      ASSERT_TRUE(hasDiag());
      EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
          << "message: " << diag_.status[0].message;
    }
    else
    {
      ASSERT_TRUE(hasDiag());
      EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, diag_.status[0].level)
          << "message: " << diag_.status[0].message;
    }

    ASSERT_TRUE(hasStatus());
    EXPECT_TRUE(status_.is_cloud_available);
    EXPECT_FALSE(status_.has_watchdog_timed_out);
    EXPECT_NE(status_.stuck_started_since, rclcpp::Time(0));

    ASSERT_TRUE(received);
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitAngular)
{
  resetMessages();
  RosRate wait(20.0, *this);

  // Skip initial state
  for (int i = 0; i < 10 && rclcpp::ok(); ++i)
  {
    publishSinglePointPointcloud2(-1, -1, 0, "base_link", now());
    publishWatchdogReset();

    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
  }

  for (float vel = 0.0; vel < M_PI; vel += M_PI / 10)
  {
    // pi/2 rad/ss, obstacle at pi/4 rad: limited to pi/2 rad/s
    bool received = false;
    bool en = false;

    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-1, -1.1, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist((i % 3) * 0.01, vel);
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (en && cmd_vel_received_)
      {
        received = true;
        const float expected_vel = std::min<float>(vel, M_PI / 2);
        ASSERT_NEAR(cmd_vel_.angular.z, expected_vel, M_PI / 20);
      }
    }
    ASSERT_TRUE(hasDiag());
    EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
        << "message: " << diag_.status[0].message;

    ASSERT_TRUE(hasStatus());
    EXPECT_TRUE(status_.is_cloud_available);
    EXPECT_FALSE(status_.has_watchdog_timed_out);
    EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0));

    ASSERT_TRUE(received);
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitAngularEscape)
{
  resetMessages();
  RosRate wait(20.0, *this);

  // Skip initial state
  for (int i = 0; i < 10 && rclcpp::ok(); ++i)
  {
    publishSinglePointPointcloud2(-1, -0.09, 0, "base_link", now());
    publishWatchdogReset();

    wait.sleep();
    rclcpp::spin_some(get_node_base_interface());
  }

  const float vel_ref[] = {-0.2, -0.4, 0.2, 0.4};
  for (const float vel : vel_ref)
  {
    // already colliding at rear-right side: only positive rotation must be allowed
    bool received = false;
    bool en = false;

    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-1, -0.09, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(0.0, vel);
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (en && cmd_vel_received_)
      {
        received = true;
        if (vel < 0)
        {
          // escaping from collision must be allowed
          ASSERT_NEAR(cmd_vel_.angular.z, vel, 1e-1);
        }
        else
        {
          // colliding motion must be limited
          ASSERT_NEAR(cmd_vel_.angular.z, 0.0, 1e-1);
        }
      }
    }
    if (vel < 0)
    {
      ASSERT_TRUE(hasDiag());
      EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
          << "message: " << diag_.status[0].message;
    }
    else
    {
      ASSERT_TRUE(hasDiag());
      EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::WARN, diag_.status[0].level)
          << "message: " << diag_.status[0].message;
    }

    ASSERT_TRUE(hasStatus());
    EXPECT_TRUE(status_.is_cloud_available);
    EXPECT_FALSE(status_.has_watchdog_timed_out);
    EXPECT_NE(status_.stuck_started_since, rclcpp::Time(0));

    ASSERT_TRUE(received);
  }
}

TEST_F(SafetyLimiterTest, NoCollision)
{
  resetMessages();
  RosRate wait(20.0, *this);

  for (float vel = 0.0; vel < 1.0; vel += 0.2)
  {
    for (float ang_vel = 0.0; ang_vel < 1.0; ang_vel += 0.2)
    {
      bool received = false;
      bool en = false;

      for (int i = 0; i < 10 && rclcpp::ok(); ++i)
      {
        if (i > 5)
          en = true;
        publishSinglePointPointcloud2(1000, 1000, 0, "base_link", now());
        publishWatchdogReset();
        publishTwist(vel, ang_vel);
        broadcastTF("odom", "base_link", 0.0, 0.0);

        wait.sleep();
        rclcpp::spin_some(get_node_base_interface());
        if (en && cmd_vel_received_)
        {
          received = true;
          ASSERT_NEAR(cmd_vel_.linear.x, vel, 1e-3);
          ASSERT_NEAR(cmd_vel_.angular.z, ang_vel, 1e-3);
        }
      }
      ASSERT_TRUE(received);

      ASSERT_TRUE(hasDiag());
      EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
          << "message: " << diag_.status[0].message;

      ASSERT_TRUE(hasStatus());
      EXPECT_EQ(1.0, status_.limit_ratio);
      EXPECT_TRUE(status_.is_cloud_available);
      EXPECT_FALSE(status_.has_watchdog_timed_out);
      EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0));
    }
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitLinearSimpleSimulation)
{
  resetMessages();
  const float dt = 0.02;
  RosRate wait(1.0 / dt, *this);

  const float velocities[] = {-0.8, -0.5, 0.5, 0.8};
  for (const float vel : velocities)
  {
    float x = 0;
    bool stopped = false;

    int count_after_stop = 10;
    for (float t = 0; t < 10.0 && rclcpp::ok() && count_after_stop > 0; t += dt)
    {
      if (vel > 0)
        publishSinglePointPointcloud2(1.0 - x, 0, 0, "base_link", now());
      else
        publishSinglePointPointcloud2(-3.0 - x, 0, 0, "base_link", now());

      publishWatchdogReset();
      publishTwist(vel, 0.0);
      broadcastTF("odom", "base_link", x, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (cmd_vel_received_)
      {
        if (std::abs(cmd_vel_.linear.x) < 1e-4 && x > 0.5)
        {
          stopped = true;
        }
        x += dt * cmd_vel_.linear.x;
      }
      if (stopped)
      {
        count_after_stop--;
      }
    }
    if (vel > 0)
    {
      EXPECT_GT(1.02, x);
      EXPECT_LT(0.95, x);
    }
    else
    {
      EXPECT_LT(-1.02, x);
      EXPECT_GT(-0.95, x);
    }
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitMaxVelocitiesValues)
{
  resetMessages();
  RosRate wait(20, *this);

  const float linear_velocities[] = {-1.7, -1.5, 0.0, 1.5, 1.7};
  const float angular_velocities[] = {-2.7, -2.5, 0.0, 2.5, 2.7};
  const float expected_linear_velocities[] = {-1.5, -1.5, 0.0, 1.5, 1.5};
  const float expected_angular_velocities[] = {-2.5, -2.5, 0.0, 2.5, 2.5};

  for (int linear_index = 0; linear_index < 5; linear_index++)
  {
    for (int angular_index = 0; angular_index < 5; angular_index++)
    {
      bool received = false;
      bool en = false;

      for (int i = 0; i < 10 && rclcpp::ok(); ++i)
      {
        if (i > 5)
          en = true;
        publishSinglePointPointcloud2(1000, 1000, 0, "base_link", now());
        publishWatchdogReset();
        publishTwist(linear_velocities[linear_index], angular_velocities[angular_index]);
        broadcastTF("odom", "base_link", 0.0, 0.0);

        wait.sleep();
        rclcpp::spin_some(get_node_base_interface());
        if (en && cmd_vel_received_)
        {
          received = true;
          ASSERT_NEAR(expected_linear_velocities[linear_index], cmd_vel_.linear.x, 1e-3);
          ASSERT_NEAR(expected_angular_velocities[angular_index], cmd_vel_.angular.z, 1e-3);
        }
      }
      ASSERT_TRUE(received);

      ASSERT_TRUE(hasDiag());
      EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
          << "message: " << diag_.status[0].message;

      ASSERT_TRUE(hasStatus());
      EXPECT_EQ(1.0, status_.limit_ratio);
      EXPECT_TRUE(status_.is_cloud_available);
      EXPECT_FALSE(status_.has_watchdog_timed_out);
      EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0));
    }
  }
}

TEST_F(SafetyLimiterTest, SafetyLimitOmniDirectional)
{
  resetMessages();
  RosRate wait(20, *this);
  const double vel = 1.5;
  const double threshold_angle = std::atan2(1.0, 0.1);
  for (double angle = -M_PI; angle < M_PI; angle += M_PI / 8)
  {
    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      publishSinglePointPointcloud2(2.0, 0, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(0, 0);
      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    double obstacle_x;
    double obstacle_y;
    if (std::abs(angle) < threshold_angle)
    {
      // Moving forward
      obstacle_x = 0.5 * std::cos(angle);
      obstacle_y = 0.5 * std::sin(angle);
    }
    else if ((M_PI - threshold_angle) < std::abs(angle))
    {
      // Moving backward
      obstacle_x = 0.5 * std::cos(angle) - 2.0;
      obstacle_y = 0.5 * std::sin(angle);
    }
    else if (angle > 0)
    {
      // Moving left
      obstacle_x = 0.5 * std::cos(angle) - 1.0;
      obstacle_y = 0.5 * std::sin(angle) + 0.1;
    }
    else
    {
      // Moving right
      obstacle_x = 0.5 * std::cos(angle) - 1.0;
      obstacle_y = 0.5 * std::sin(angle) - 0.1;
    }

    // 1.0 m/ss, obstacle at 0.5 m: limited to 1.0 m/s
    bool received = false;
    bool en = false;
    for (int i = 0; i < 10 && rclcpp::ok(); ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(obstacle_x, obstacle_y, 0, "base_link", now());
      publishWatchdogReset();
      publishTwist(vel * std::cos(angle), 0, vel * std::sin(angle));
      broadcastTF("odom", "base_link", 0.0, 0.0);

      wait.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (en && cmd_vel_received_)
      {
        received = true;
        const double current_speed = std::hypot(cmd_vel_.linear.x, cmd_vel_.linear.y);
        ASSERT_NEAR(current_speed, 1.0, 1e-1)
            << " Angle: " << angle << " i: " << i << " vel: (" << cmd_vel_.linear.x << "," << cmd_vel_.linear.y << ")";
        ASSERT_FLOAT_EQ(std::atan2(cmd_vel_.linear.y, cmd_vel_.linear.x), angle)
            << " Angle: " << angle << " i: " << i << " vel: (" << cmd_vel_.linear.x << "," << cmd_vel_.linear.y << ")";
      }
    }
    ASSERT_TRUE(hasDiag());
    EXPECT_EQ(diagnostic_msgs::msg::DiagnosticStatus::OK, diag_.status[0].level)
        << "message: " << diag_.status[0].message;

    ASSERT_TRUE(hasStatus());
    EXPECT_TRUE(status_.is_cloud_available);
    EXPECT_FALSE(status_.has_watchdog_timed_out);
    EXPECT_EQ(status_.stuck_started_since, rclcpp::Time(0));

    ASSERT_TRUE(received);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
