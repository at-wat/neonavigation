/*
 * Copyright (c) 2015-2018, the neonavigation authors
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

#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <neonavigation_common/compatibility.h>

class JoystickInterrupt
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_joy_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_int_;
  double linear_vel_;
  double angular_vel_;
  double timeout_;
  double linear_high_speed_ratio_;
  double angular_high_speed_ratio_;
  int linear_axis_;
  int angular_axis_;
  int linear_axis2_;
  int angular_axis2_;
  int interrupt_button_;
  int high_speed_button_;
  ros::Time last_joy_msg_;
  geometry_msgs::Twist last_input_twist_;

  void cbJoy(const sensor_msgs::Joy::Ptr msg)
  {
    if (static_cast<size_t>(interrupt_button_) >= msg->buttons.size())
    {
      ROS_ERROR("Out of range: number of buttons (%lu) must be greater than interrupt_button (%d).",
                msg->buttons.size(), interrupt_button_);
      last_joy_msg_ = ros::Time(0);
      return;
    }
    if (!msg->buttons[interrupt_button_])
    {
      if (last_joy_msg_ != ros::Time(0))
      {
        pub_twist_.publish(last_input_twist_);
      }
      last_joy_msg_ = ros::Time(0);
      return;
    }

    last_joy_msg_ = ros::Time::now();

    float lin(0.0f);
    float ang(0.0f);
    if (static_cast<size_t>(linear_axis_) < msg->axes.size())
      lin = msg->axes[linear_axis_];
    else
      ROS_ERROR("Out of range: number of axis (%lu) must be greater than linear_axis (%d).",
                msg->axes.size(), linear_axis_);
    if (static_cast<size_t>(angular_axis_) < msg->axes.size())
      ang = msg->axes[angular_axis_];
    else
      ROS_ERROR("Out of range: number of axis (%lu) must be greater than angular_axis (%d).",
                msg->axes.size(), angular_axis_);

    if (linear_axis2_ >= 0)
    {
      if (static_cast<size_t>(linear_axis2_) < msg->axes.size())
      {
        if (std::abs(msg->axes[linear_axis2_]) > std::abs(lin))
          lin = msg->axes[linear_axis2_];
      }
      else
        ROS_ERROR("Out of range: number of axis (%lu) must be greater than linear_axis2 (%d).",
                  msg->axes.size(), linear_axis2_);
    }
    if (angular_axis2_ >= 0)
    {
      if (static_cast<size_t>(angular_axis2_) < msg->axes.size())
      {
        if (std::abs(msg->axes[angular_axis2_]) > std::abs(ang))
          ang = msg->axes[angular_axis2_];
      }
      else
        ROS_ERROR("Out of range: number of axis (%lu) must be greater than angular_axis2 (%d).",
                  msg->axes.size(), angular_axis2_);
    }

    if (high_speed_button_ >= 0)
    {
      if (static_cast<size_t>(high_speed_button_) < msg->buttons.size())
      {
        if (msg->buttons[high_speed_button_])
        {
          lin *= linear_high_speed_ratio_;
          ang *= angular_high_speed_ratio_;
        }
      }
      else
        ROS_ERROR("Out of range: number of buttons (%lu) must be greater than high_speed_button (%d).",
                  msg->buttons.size(), high_speed_button_);
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = lin * linear_vel_;
    cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
    cmd_vel.angular.z = ang * angular_vel_;
    cmd_vel.angular.x = cmd_vel.angular.y = 0.0;
    pub_twist_.publish(cmd_vel);
  };
  void cbTwist(const geometry_msgs::Twist::Ptr msg)
  {
    last_input_twist_ = *msg;
    std_msgs::Bool status;
    if (ros::Time::now() - last_joy_msg_ > ros::Duration(timeout_) ||
        (ros::Time::isSimTime() && last_joy_msg_ == ros::Time(0)))
    {
      pub_twist_.publish(last_input_twist_);
      status.data = true;
    }
    else
    {
      status.data = false;
    }
    pub_int_.publish(status);
  };

public:
  JoystickInterrupt()
    : nh_("")
    , pnh_("~")
  {
    neonavigation_common::compat::checkCompatMode();
    sub_joy_ = nh_.subscribe("joy", 1, &JoystickInterrupt::cbJoy, this);
    sub_twist_ = neonavigation_common::compat::subscribe(
        nh_, "cmd_vel_input",
        pnh_, "cmd_vel_input", 1, &JoystickInterrupt::cbTwist, this);
    pub_twist_ = neonavigation_common::compat::advertise<geometry_msgs::Twist>(
        nh_, "cmd_vel",
        pnh_, "cmd_vel", 2);
    pub_int_ = pnh_.advertise<std_msgs::Bool>("interrupt_status", 2);

    pnh_.param("linear_vel", linear_vel_, 0.5);
    pnh_.param("angular_vel", angular_vel_, 0.8);
    pnh_.param("linear_axis", linear_axis_, 1);
    pnh_.param("angular_axis", angular_axis_, 0);
    pnh_.param("linear_axis2", linear_axis2_, -1);
    pnh_.param("angular_axis2", angular_axis2_, -1);
    pnh_.param("interrupt_button", interrupt_button_, 6);
    pnh_.param("high_speed_button", high_speed_button_, -1);
    pnh_.param("linear_high_speed_ratio", linear_high_speed_ratio_, 1.3);
    pnh_.param("angular_high_speed_ratio", angular_high_speed_ratio_, 1.1);
    pnh_.param("timeout", timeout_, 0.5);
    last_joy_msg_ = ros::Time(0);

    if (interrupt_button_ < 0)
    {
      ROS_ERROR("interrupt_button must be grater than -1.");
      ros::shutdown();
      return;
    }
    if (linear_axis_ < 0)
    {
      ROS_ERROR("linear_axis must be grater than -1.");
      ros::shutdown();
      return;
    }
    if (angular_axis_ < 0)
    {
      ROS_ERROR("angular_axis must be grater than -1.");
      ros::shutdown();
      return;
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joystick_interrupt");

  JoystickInterrupt jy;
  ros::spin();

  return 0;
}
