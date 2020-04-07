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

#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <neonavigation_common/compatibility.h>

class PoseTransformNode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  std::string to_;

  ros::Publisher pub_pose_;
  ros::Subscriber sub_pose_;

  void cbPose(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
  {
    try
    {
      geometry_msgs::PoseStamped in;
      geometry_msgs::PoseStamped out;
      geometry_msgs::PoseWithCovarianceStamped out_msg;
      in.header = msg->header;
      in.header.stamp = ros::Time(0);
      in.pose = msg->pose.pose;
      geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform(
          to_, msg->header.frame_id, in.header.stamp, ros::Duration(0.5));
      tf2::doTransform(in, out, trans);
      out_msg = *msg;
      out_msg.header = out.header;
      out_msg.pose.pose = out.pose;
      pub_pose_.publish(out_msg);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("pose_transform: %s", e.what());
    }
  }

public:
  PoseTransformNode()
    : pnh_("~")
    , tfl_(tfbuf_)
  {
    neonavigation_common::compat::checkCompatMode();
    sub_pose_ = neonavigation_common::compat::subscribe(
        nh_, "pose_in",
        pnh_, "pose_in", 1, &PoseTransformNode::cbPose, this);
    pub_pose_ = neonavigation_common::compat::advertise<geometry_msgs::PoseWithCovarianceStamped>(
        nh_, "pose_out",
        pnh_, "pose_out", 1, false);
    pnh_.param("to_frame", to_, std::string("map"));
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_transform");

  PoseTransformNode ptn();
  ros::spin();

  return 0;
}
