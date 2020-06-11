/*
 * Copyright (c) 2015-2020, the neonavigation authors
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

#include <topic_tools/shape_shifter.h>

#include <neonavigation_common/compatibility.h>

class JoystickMux
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_topics_[2];
  ros::Subscriber sub_joy_;
  ros::Publisher pub_topic_;
  ros::Timer timer_;
  double timeout_;
  int interrupt_button_;
  ros::Time last_joy_msg_;
  bool advertised_;
  int selected_;

  void cbJoy(const sensor_msgs::Joy::Ptr msg)
  {
    if (static_cast<size_t>(interrupt_button_) >= msg->buttons.size())
    {
      ROS_ERROR(
          "Out of range: number of buttons (%lu) must be greater than interrupt_button (%d).",
          msg->buttons.size(), interrupt_button_);
      return;
    }

    last_joy_msg_ = ros::Time::now();
    if (msg->buttons[interrupt_button_])
    {
      selected_ = 1;
    }
    else
    {
      selected_ = 0;
    }
  };
  void cbTopic(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg, int id)
  {
    if (selected_ == id)
    {
      if (!advertised_)
      {
        advertised_ = true;
        if (neonavigation_common::compat::getCompat() !=
            neonavigation_common::compat::current_level)
        {
          ROS_ERROR(
              "Use %s (%s%s) topic instead of %s (%s%s)",
              nh_.resolveName("mux_output", false).c_str(),
              neonavigation_common::compat::getSimplifiedNamespace(nh_).c_str(),
              "mux_output",
              pnh_.resolveName("output", false).c_str(),
              neonavigation_common::compat::getSimplifiedNamespace(pnh_).c_str(),
              "output");
          pub_topic_ = msg->advertise(pnh_, "output", 1, false);
        }
        else
        {
          pub_topic_ = msg->advertise(nh_, "mux_output", 1, false);
        }
      }
      pub_topic_.publish(*msg);
    }
  };
  void cbTimer(const ros::TimerEvent& e)
  {
    if (ros::Time::now() - last_joy_msg_ > ros::Duration(timeout_))
    {
      selected_ = 0;
    }
  }

public:
  JoystickMux()
    : nh_("")
    , pnh_("~")
  {
    neonavigation_common::compat::checkCompatMode();
    sub_joy_ = nh_.subscribe("joy", 1, &JoystickMux::cbJoy, this);
    sub_topics_[0] = neonavigation_common::compat::subscribe<topic_tools::ShapeShifter>(
        nh_, "mux_input0",
        pnh_, "input0", 1, boost::bind(&JoystickMux::cbTopic, this, _1, 0));
    sub_topics_[1] = neonavigation_common::compat::subscribe<topic_tools::ShapeShifter>(
        nh_, "mux_input1",
        pnh_, "input1", 1, boost::bind(&JoystickMux::cbTopic, this, _1, 1));

    pnh_.param("interrupt_button", interrupt_button_, 5);
    pnh_.param("timeout", timeout_, 0.5);
    last_joy_msg_ = ros::Time::now();

    timer_ = nh_.createTimer(ros::Duration(0.1), &JoystickMux::cbTimer, this);

    advertised_ = false;
    selected_ = 0;
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joystick_mux");

  JoystickMux jy;
  ros::spin();

  return 0;
}
