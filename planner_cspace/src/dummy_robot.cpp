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
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <neonavigation_common/compatibility.h>

class DummyRobotNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  double x_;
  double y_;
  double yaw_;
  float v_;
  float w_;

  ros::Publisher pub_odom_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_init_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformBroadcaster tfb_;
  tf2_ros::TransformListener tfl_;

  void cbTwist(const geometry_msgs::Twist::ConstPtr& msg)
  {
    v_ = msg->linear.x;
    w_ = msg->angular.z;
  }
  void cbInit(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;
    try
    {
      geometry_msgs::TransformStamped trans =
          tfbuf_.lookupTransform("odom", pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(1.0));
      tf2::doTransform(pose_in, pose_out, trans);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }

    x_ = pose_out.pose.position.x;
    y_ = pose_out.pose.position.y;
    yaw_ = tf2::getYaw(pose_out.pose.orientation);
  }

public:
  DummyRobotNode()
    : nh_()
    , pnh_("~")
    , tfl_(tfbuf_)
  {
    neonavigation_common::compat::checkCompatMode();
    pnh_.param("initial_x", x_, 0.0);
    pnh_.param("initial_y", y_, 0.0);
    pnh_.param("initial_yaw", yaw_, 0.0);
    v_ = 0.0;
    w_ = 0.0;

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
    sub_twist_ = nh_.subscribe("cmd_vel", 1, &DummyRobotNode::cbTwist, this);
    sub_init_ = nh_.subscribe("initialpose", 1, &DummyRobotNode::cbInit, this);
  }
  void spin()
  {
    const float dt = 0.01;
    ros::Rate rate(1.0 / dt);

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      const ros::Time current_time = ros::Time::now();

      yaw_ += w_ * dt;
      x_ += cosf(yaw_) * v_ * dt;
      y_ += sinf(yaw_) * v_ * dt;

      geometry_msgs::TransformStamped trans;
      trans.header.stamp = current_time;
      trans.header.frame_id = "odom";
      trans.child_frame_id = "base_link";
      trans.transform.translation = tf2::toMsg(tf2::Vector3(x_, y_, 0.0));
      trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
      tfb_.sendTransform(trans);

      nav_msgs::Odometry odom;
      odom.header.frame_id = "odom";
      odom.header.stamp = current_time;
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
      odom.twist.twist.linear.x = v_;
      odom.twist.twist.angular.z = w_;
      pub_odom_.publish(odom);
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dummy_robot");

  DummyRobotNode robot;
  robot.spin();

  return 0;
}
