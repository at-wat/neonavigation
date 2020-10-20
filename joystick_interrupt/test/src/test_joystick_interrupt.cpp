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
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>

#include <gtest/gtest.h>

class JoystickInterruptTest : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_joy_;
  ros::Subscriber sub_cmd_vel_;

  geometry_msgs::Twist::ConstPtr cmd_vel_;

  void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
  {
    cmd_vel_ = msg;
  }

public:
  JoystickInterruptTest()
    : nh_()
  {
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_input", 1);
    pub_joy_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
    sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &JoystickInterruptTest::cbCmdVel, this);

    ros::Rate wait(10);
    for (size_t i = 0; i < 100; ++i)
    {
      wait.sleep();
      ros::spinOnce();
      if (i > 5 && pub_cmd_vel_.getNumSubscribers() > 0)
        break;
    }
  }
  void publishCmdVel(
      const float lin,
      const float ang)
  {
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = lin;
    cmd_vel_out.angular.z = ang;
    pub_cmd_vel_.publish(cmd_vel_out);
  }
  void publishJoy(
      const int button,
      const int high_speed,
      const float lin0,
      const float ang0,
      const float lin1,
      const float ang1)
  {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.buttons.resize(2);
    joy.buttons[0] = button;
    joy.buttons[1] = high_speed;
    joy.axes.resize(4);
    joy.axes[0] = lin0;
    joy.axes[1] = ang0;
    joy.axes[2] = lin1;
    joy.axes[3] = ang1;
    pub_joy_.publish(joy);
  }
};

TEST_F(JoystickInterruptTest, NoInterrupt)
{
  ros::Duration(1.0).sleep();
  ros::Rate rate(20);
  for (size_t i = 0; i < 25; ++i)
  {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(0, 0, 1, 1, 0, 0);
    else if (i < 15)
      publishJoy(0, 0, 0, 0, 1, 1);
    else if (i < 20)
      publishJoy(0, 1, 1, 1, 0, 0);
    else
      publishJoy(0, 1, 0, 0, 1, 1);

    rate.sleep();
    ros::spinOnce();
    if (i < 3)
      continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
    ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
  }
}

TEST_F(JoystickInterruptTest, Interrupt)
{
  ros::Duration(1.0).sleep();
  ros::Rate rate(20);
  for (size_t i = 0; i < 25; ++i)
  {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 0, 1, 0, 0, 0);
    else if (i < 15)
      publishJoy(1, 0, 0, 0, 1, 0);
    else if (i < 20)
      publishJoy(1, 0, 0, 1, 0, 0);
    else
      publishJoy(1, 0, 0, 0, 0, 1);

    rate.sleep();
    ros::spinOnce();
    if (i < 3)
      continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
    }
    else if (i < 15)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 1.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    }
    else
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 1.0, 1e-3);
    }
  }
}

TEST_F(JoystickInterruptTest, InterruptNoTwistInput)
{
  ros::Duration(1.0).sleep();
  // make sure the internal state of the joystick interrupt node
  // (i.e. last_input_twist_) is set back to a zero twist.
  publishCmdVel(0, 0);
  ros::Rate rate(20);
  for (size_t i = 0; i < 20; ++i)
  {
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 0, 1, 0.5, 0, 0);
    else if (i < 15)
      publishJoy(0, 0, 1, 0, 0, 0);
    else
      publishJoy(0, 0, 0, 0.5, 0, 0);

    rate.sleep();
    ros::spinOnce();
    if (i < 3)
      continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0, 1e-3);
    }
    else if (i < 10)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 1.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.5, 1e-3);
    }
    else
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0, 1e-3);
    }
  }
}

TEST_F(JoystickInterruptTest, InterruptHighSpeed)
{
  ros::Duration(1.0).sleep();
  ros::Rate rate(20);
  for (size_t i = 0; i < 25; ++i)
  {
    publishCmdVel(0.1, 0.2);
    if (i < 5)
      publishJoy(0, 0, 0, 0, 0, 0);
    else if (i < 10)
      publishJoy(1, 1, 1, 0, 0, 0);
    else if (i < 15)
      publishJoy(1, 1, 0, 0, 1, 0);
    else if (i < 20)
      publishJoy(1, 1, 0, 1, 0, 0);
    else
      publishJoy(1, 1, 0, 0, 0, 1);

    rate.sleep();
    ros::spinOnce();
    if (i < 3)
      continue;
    ASSERT_TRUE(static_cast<bool>(cmd_vel_));
    if (i < 5)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.1, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.2, 1e-3);
    }
    else if (i < 15)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 2.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 0.0, 1e-3);
    }
    else
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.0, 1e-3);
      ASSERT_NEAR(cmd_vel_->angular.z, 2.0, 1e-3);
    }
  }
}

class JoystickMuxTest : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub_joy_;
  ros::Subscriber sub_;

  std_msgs::Int32::ConstPtr msg_;

  void cbMsg(const std_msgs::Int32::ConstPtr& msg)
  {
    msg_ = msg;
  }

public:
  JoystickMuxTest()
  {
    pub1_ = nh_.advertise<std_msgs::Int32>("mux_input0", 1);
    pub2_ = nh_.advertise<std_msgs::Int32>("mux_input1", 1);
    pub_joy_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
    sub_ = nh_.subscribe("mux_output", 1, &JoystickMuxTest::cbMsg, this);

    ros::Rate wait(10);
    for (size_t i = 0; i < 100; ++i)
    {
      wait.sleep();
      ros::spinOnce();
      if (i > 5 && pub1_.getNumSubscribers() > 0)
        break;
    }
  }
  void waitPublisher()
  {
    ros::Rate wait(10);
    for (size_t i = 0; i < 100; ++i)
    {
      wait.sleep();
      ros::spinOnce();
      if (i > 5 && sub_.getNumPublishers() > 0)
        break;
    }
  }
  void publish1(const int32_t v)
  {
    std_msgs::Int32 msg_out;
    msg_out.data = v;
    pub1_.publish(msg_out);
  }
  void publish2(const int32_t v)
  {
    std_msgs::Int32 msg_out;
    msg_out.data = v;
    pub2_.publish(msg_out);
  }
  void publishJoy(const int button)
  {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.buttons.resize(1);
    joy.buttons[0] = button;
    pub_joy_.publish(joy);
  }
  void publishEmptyJoy()
  {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    pub_joy_.publish(joy);
  }
};

TEST_F(JoystickMuxTest, Interrupt)
{
  publish1(0);
  publish2(0);
  waitPublisher();
  for (int btn = 0; btn < 2; ++btn)
  {
    publishJoy(btn);
    ros::Duration(1.0).sleep();
    ros::Rate rate(20);
    for (int i = 0; i < 15; ++i)
    {
      publishJoy(btn);
      publish1(i);
      publish2(-i);

      rate.sleep();
      ros::spinOnce();

      if (i < 5)
        continue;

      ASSERT_TRUE(static_cast<bool>(msg_)) << "button: " << btn;
      if (btn)
      {
        ASSERT_NEAR(-i, msg_->data, 2) << "button: " << btn;
      }
      else
      {
        ASSERT_NEAR(i, msg_->data, 2) << "button:" << btn;
      }
    }
  }
}
/*
TEST_F(JoystickMuxTest, Timeout)
{
  publish1(0);
  publish2(0);
  waitPublisher();
  ros::Rate rate(20);
  publishJoy(1);
  for (int i = 0; i < 20; ++i)
  {
    publish1(i);
    publish2(-i);

    rate.sleep();
    ros::spinOnce();

    if (i < 5)
      continue;

    ASSERT_TRUE(static_cast<bool>(msg_));
    if (i < 10)
    {
      ASSERT_NEAR(-i, msg_->data, 2);
    }
    else if (i > 13)
    {
      // after timeout
      ASSERT_NEAR(i, msg_->data, 2);
    }
  }
}

TEST_F(JoystickMuxTest, ButtonNumberInsufficient)
{
  publish1(0);
  publish2(0);
  waitPublisher();
  ros::Rate rate(20);
  for (int i = 0; i < 20; ++i)
  {
    publishEmptyJoy();
    publish1(i);
    publish2(-i);

    rate.sleep();
    ros::spinOnce();

    if (i < 3)
      continue;

    ASSERT_TRUE(static_cast<bool>(msg_));
    ASSERT_NEAR(i, msg_->data, 2);
  }
}*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joystick_interrupt");

  return RUN_ALL_TESTS();
}
