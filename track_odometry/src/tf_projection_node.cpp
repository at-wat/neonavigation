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

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include <track_odometry/tf_projection.h>

class TfProjectionNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  double rate_;
  double tf_tolerance_;
  bool flat_;
  bool project_posture_;
  bool align_all_posture_to_source_;

  std::string source_frame_;
  std::string projection_surface_frame_;
  std::string parent_frame_;
  std::string projected_frame_;

public:
  TfProjectionNode()
    : nh_()
    , pnh_("~")
    , tf_listener_(tf_buffer_)
  {
    if (pnh_.hasParam("base_link_frame") ||
        pnh_.hasParam("projection_frame") ||
        pnh_.hasParam("target_frame") ||
        pnh_.hasParam("frame"))
    {
      ROS_ERROR(
          "tf_projection parameters \"base_link_frame\", \"projection_frame\", \"target_frame\", and \"frame\" "
          "are replaced by \"source_frame\", \"projection_surface_frame\", \"parent_frame\", and \"projected_frame\"");

      pnh_.param("base_link_frame", source_frame_, std::string("base_link"));
      pnh_.param("projection_frame", projection_surface_frame_, std::string("map"));
      pnh_.param("target_frame", parent_frame_, std::string("map"));
      pnh_.param("frame", projected_frame_, std::string("base_link_projected"));
    }
    else
    {
      pnh_.param("source_frame", source_frame_, std::string("base_link"));
      pnh_.param("projection_surface_frame", projection_surface_frame_, std::string("map"));
      pnh_.param("parent_frame", parent_frame_, std::string("map"));
      pnh_.param("projected_frame", projected_frame_, std::string("base_link_projected"));
    }

    pnh_.param("hz", rate_, 10.0);
    pnh_.param("tf_tolerance", tf_tolerance_, 0.1);
    pnh_.param("flat", flat_, false);

    pnh_.param("project_posture", project_posture_, false);
    pnh_.param("align_all_posture_to_source", align_all_posture_to_source_, false);
  }
  void process()
  {
    tf2::Stamped<tf2::Transform> trans;
    tf2::Stamped<tf2::Transform> trans_target;
    try
    {
      tf2::fromMsg(
          tf_buffer_.lookupTransform(projection_surface_frame_, source_frame_, ros::Time(0), ros::Duration(0.1)),
          trans);
      tf2::fromMsg(
          tf_buffer_.lookupTransform(parent_frame_, projection_surface_frame_, trans.stamp_, ros::Duration(0.1)),
          trans_target);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(1.0, "%s", e.what());
      return;
    }

    if (!trans.stamp_.isZero())
      trans.stamp_ += ros::Duration(tf_tolerance_);

    if (project_posture_)
    {
      if (align_all_posture_to_source_)
      {
        const tf2::Quaternion rot(trans.getRotation());
        const tf2::Quaternion rot_yaw(tf2::Vector3(0.0, 0.0, 1.0), tf2::getYaw(rot));
        const tf2::Transform rot_inv(rot_yaw * rot.inverse());
        trans.setData(rot_inv * trans);
      }
      else
      {
        const float yaw = tf2::getYaw(trans.getRotation());
        trans.setRotation(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
      }
    }

    const tf2::Stamped<tf2::Transform> result(
        track_odometry::projectTranslation(trans, trans_target),
        trans.stamp_,
        parent_frame_);

    geometry_msgs::TransformStamped trans_out = tf2::toMsg(result);
    if (flat_)
    {
      const double yaw = tf2::getYaw(trans_out.transform.rotation);
      trans_out.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    }
    trans_out.child_frame_id = projected_frame_;

    if (trans.stamp_.isZero())
    {
      tf_static_broadcaster_.sendTransform(trans_out);
    }
    else
    {
      tf_broadcaster_.sendTransform(trans_out);
    }
  }
  void cbTimer(const ros::TimerEvent& event)
  {
    process();
  }
  void spin()
  {
    ros::Timer timer = nh_.createTimer(
        ros::Duration(1.0 / rate_), &TfProjectionNode::cbTimer, this);
    ros::spin();
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tf_projection");

  TfProjectionNode proj;
  proj.spin();

  return 0;
}
