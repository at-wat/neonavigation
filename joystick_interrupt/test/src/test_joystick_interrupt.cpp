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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joystick_interrupt");

  return RUN_ALL_TESTS();
}
