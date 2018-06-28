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
#include <sensor_msgs/Joy.h>

#include <topic_tools/shape_shifter.h>

class JoystickMux
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_topics_[2];
  ros::Subscriber sub_joy_;
  ros::Publisher pub_topic_;
  double timeout_;
  int interrupt_button_;
  ros::Time last_joy_msg_;
  bool advertised_;
  int selected_;

  void cbJoy(const sensor_msgs::Joy::Ptr msg)
  {
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
  void cbTopic(const boost::shared_ptr<topic_tools::ShapeShifter const> &msg, int id)
  {
    if (selected_ == id)
    {
      if (!advertised_)
      {
        advertised_ = true;
        pub_topic_ = msg->advertise(pnh_, "output", 1, false);
      }
      pub_topic_.publish(*msg);
    }
  };

public:
  JoystickMux()
    : nh_("")
    , pnh_("~")
  {
    sub_joy_ = nh_.subscribe("joy", 1, &JoystickMux::cbJoy, this);
    sub_topics_[0] = pnh_.subscribe<topic_tools::ShapeShifter>(
        "input0", 1, boost::bind(&JoystickMux::cbTopic, this, _1, 0));
    sub_topics_[1] = pnh_.subscribe<topic_tools::ShapeShifter>(
        "input1", 1, boost::bind(&JoystickMux::cbTopic, this, _1, 1));

    pnh_.param("interrupt_button", interrupt_button_, 5);
    pnh_.param("timeout", timeout_, 0.5);
    last_joy_msg_ = ros::Time::now();

    advertised_ = false;
    selected_ = 0;
  }
  void spin()
  {
    ros::Rate wait(10);
    while (ros::ok())
    {
      wait.sleep();
      ros::spinOnce();
      if (ros::Time::now() - last_joy_msg_ > ros::Duration(timeout_))
      {
        selected_ = 0;
      }
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joystick_mux");

  JoystickMux jy;
  jy.spin();

  return 0;
}
