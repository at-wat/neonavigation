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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>

#include <algorithm>
#include <string>

#include <gtest/gtest.h>

namespace
{
void GenerateSinglePointPointcloud2(
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
  SafetyLimiterTest()
    : nh_()
  {
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_in", 1);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    pub_watchdog_ = nh_.advertise<std_msgs::Empty>("watchdog_reset", 1);
  }
  void publishWatchdogReset()
  {
    std_msgs::Empty watchdog_reset;
    pub_watchdog_.publish(watchdog_reset);
  }
  void publishSinglePointPointcloud2(
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
  void publishTwist(
      const float lin,
      const float ang)
  {
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = lin;
    cmd_vel_out.angular.z = ang;
    pub_cmd_vel_.publish(cmd_vel_out);
  }
};

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
  ros::Rate wait(40.0);

  // Skip initial state
  for (size_t i = 0; i < 30 && ros::ok(); ++i)
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

  for (size_t i = 0; i < 40 * 6 && ros::ok() && !failed; ++i)
  {
    // enable check after two cycles of safety_limiter
    if (i > 8)
      en = true;
    // safety_limiter: 10 hz, cloud publish: 40 hz
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
  for (size_t i = 0; i < 10 && ros::ok(); ++i)
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

    for (size_t i = 0; i < 10 && ros::ok() && !failed; ++i)
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
  for (size_t i = 0; i < 10 && ros::ok(); ++i)
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

    for (size_t i = 0; i < 10 && ros::ok() && !failed; ++i)
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
  for (size_t i = 0; i < 10 && ros::ok(); ++i)
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

    for (size_t i = 0; i < 10 && ros::ok() && !failed; ++i)
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
  for (size_t i = 0; i < 10 && ros::ok(); ++i)
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

    for (size_t i = 0; i < 10 && ros::ok() && !failed; ++i)
    {
      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(-1, -1, 0, "base_link", ros::Time::now());
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
  for (size_t i = 0; i < 10 && ros::ok(); ++i)
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

    for (size_t i = 0; i < 10 && ros::ok() && !failed; ++i)
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

      for (size_t i = 0; i < 10 && ros::ok() && !failed; ++i)
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter");

  return RUN_ALL_TESTS();
}
