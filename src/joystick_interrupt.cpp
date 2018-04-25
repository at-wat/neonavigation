/*
 * Copyright (c) 2015-2018, the joystick_interrupt authors
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
#include <std_msgs/Bool.h>

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

  void cbJoy(const sensor_msgs::Joy::Ptr msg)
  {
    if (msg->buttons[interrupt_button_])
    {
      last_joy_msg_ = ros::Time::now();

      float lin, ang;
      lin = msg->axes[linear_axis_];
      ang = msg->axes[angular_axis_];
      if (linear_axis2_ >= 0)
      {
        if (fabs(msg->axes[linear_axis2_]) > fabs(lin))
        {
          lin = msg->axes[linear_axis2_];
        }
      }
      if (angular_axis2_ >= 0)
      {
        if (fabs(msg->axes[angular_axis2_]) > fabs(lin))
        {
          ang = msg->axes[angular_axis2_];
        }
      }

      if (high_speed_button_ >= 0)
      {
        if (msg->buttons[high_speed_button_])
        {
          lin *= linear_high_speed_ratio_;
          ang *= angular_high_speed_ratio_;
        }
      }

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = lin * linear_vel_;
      cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
      cmd_vel.angular.z = ang * angular_vel_;
      cmd_vel.angular.x = cmd_vel.angular.y = 0.0;
      pub_twist_.publish(cmd_vel);
    }
    else
    {
      last_joy_msg_ = ros::Time(0);
    }
  };
  void cbTwist(const geometry_msgs::Twist::Ptr msg)
  {
    std_msgs::Bool status;
    if (ros::Time::now() - last_joy_msg_ > ros::Duration(timeout_))
    {
      pub_twist_.publish(*msg);
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
    sub_joy_ = nh_.subscribe("joy", 1, &JoystickInterrupt::cbJoy, this);
    sub_twist_ = pnh_.subscribe("cmd_vel_input", 1, &JoystickInterrupt::cbTwist, this);
    pub_twist_ = pnh_.advertise<geometry_msgs::Twist>("cmd_vel", 2);
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
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joystick_interrupt");

  JoystickInterrupt jy;
  ros::spin();

  return 0;
}
