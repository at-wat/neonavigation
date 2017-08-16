/*
 * Copyright (c) 2014-2017, the neonavigation authors
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

class DummyRobot
{
protected:
  float x_;
  float y_;
  float yaw_;
  float v_;
  float w_;

  ros::Subscriber sub_twist_;
  ros::Subscriber sub_init_;
  tf::TransformBroadcaster tfb_;

  void cbTwist(const geometry_msgs::Twist::ConstPtr &msg)
  {
    v_ = msg->linear.x;
    w_ = msg->angular.z;
  }
  void cbInit(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
  }

public:
  DummyRobot()
  {
    ros::NodeHandle nh("");

    sub_twist_ = nh.subscribe("cmd_vel", 1, &DummyRobot::cbTwist, this);
    sub_init_ = nh.subscribe("initialpose", 1, &DummyRobot::cbInit, this);
  }
  void spin()
  {
    const float dt = 0.01;
    ros::Rate rate(1.0 / dt);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();

      yaw_ += w_ * dt;
      x_ += cosf(yaw_) * v_ * dt;
      y_ += sinf(yaw_) * v_ * dt;

      tf::StampedTransform trans;
      trans.stamp_ = ros::Time::now();
      trans.frame_id_ = "odom";
      trans.child_frame_id_ = "base_link";
      trans.setOrigin(tf::Vector3(x_, y_, 0.0));
      trans.setRotation(tf::createQuaternionFromYaw(yaw_));
      tfb_.sendTransform(trans);
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dummy_robot");

  DummyRobot robot;
  robot.spin();

  return 0;
}
