/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal
   Affairs and Communications entitled, 'Novel and innovative R&D making use
   of brain structures'

   This software was implemented to accomplish the above research.
 */

#include <cmath>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>

#include <neonavigation_common/compatibility.h>

class RecorderNode
{
public:
  RecorderNode();
  ~RecorderNode();
  void spin();

private:
  bool clearPath(std_srvs::Empty::Request& req,
                 std_srvs::Empty::Response& res);

  std::string topic_path_;
  std::string frame_robot_;
  std::string frame_global_;
  double dist_interval_;
  double ang_interval_;
  bool store_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_path_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  ros::ServiceServer srs_clear_path_;

  nav_msgs::Path path_;
};

RecorderNode::RecorderNode()
  : nh_()
  , pnh_("~")
  , tfl_(tfbuf_)
{
  neonavigation_common::compat::checkCompatMode();
  pnh_.param("frame_robot", frame_robot_, std::string("base_link"));
  pnh_.param("frame_global", frame_global_, std::string("map"));
  neonavigation_common::compat::deprecatedParam(pnh_, "path", topic_path_, std::string("recpath"));
  pnh_.param("dist_interval", dist_interval_, 0.3);
  pnh_.param("ang_interval", ang_interval_, 1.0);
  pnh_.param("store_time", store_time_, false);

  pub_path_ = neonavigation_common::compat::advertise<nav_msgs::Path>(
      nh_, "path",
      pnh_, topic_path_, 10, true);
  srs_clear_path_ = pnh_.advertiseService("clear_path", &RecorderNode::clearPath, this);
}

RecorderNode::~RecorderNode()
{
}

float dist2d(geometry_msgs::Point& a, geometry_msgs::Point& b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool RecorderNode::clearPath(std_srvs::Empty::Request& /* req */,
                             std_srvs::Empty::Response& /* res */)
{
  path_.poses.clear();
  return true;
}

void RecorderNode::spin()
{
  ros::Rate loop_rate(50);
  path_.header.frame_id = frame_global_;
  path_.header.seq = 0;

  while (ros::ok())
  {
    ros::Time now = ros::Time(0);
    if (store_time_)
      now = ros::Time::now();
    tf2::Stamped<tf2::Transform> transform;
    try
    {
      tf2::fromMsg(
          tfbuf_.lookupTransform(frame_global_, frame_robot_, now, ros::Duration(0.2)), transform);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("TF exception: %s", e.what());
      continue;
    }
    geometry_msgs::PoseStamped pose;
    tf2::Quaternion q;
    transform.getBasis().getRotation(q);
    pose.pose.orientation = tf2::toMsg(q);
    tf2::Vector3 origin = transform.getOrigin();
    pose.pose.position.x = origin.x();
    pose.pose.position.y = origin.y();
    pose.pose.position.z = origin.z();
    pose.header.frame_id = frame_global_;
    pose.header.stamp = now;
    pose.header.seq = path_.poses.size();

    path_.header.seq++;
    path_.header.stamp = now;

    if (path_.poses.size() == 0)
    {
      path_.poses.push_back(pose);
      pub_path_.publish(path_);
    }
    else if (dist2d(path_.poses.back().pose.position, pose.pose.position) > dist_interval_)
    {
      path_.poses.push_back(pose);
      pub_path_.publish(path_);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_recorder");

  RecorderNode rec;
  rec.spin();

  return 0;
}
