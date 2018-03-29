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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <string>

#include <tf_projection.h>

class TfProjectionNode : public TfProjection
{
private:
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  double rate_;
  double tf_tolerance_;
  bool flat_;

public:
  TfProjectionNode()
    : nh_("~")
  {
    nh_.param("base_link_frame", frames_["base"], std::string("base_link"));
    nh_.param("projection_frame", frames_["projection"], std::string("map"));
    nh_.param("target_frame", frames_["target"], std::string("map"));
    nh_.param("frame", frames_["frame"], std::string("base_link_projected"));

    nh_.param("hz", rate_, 10.0);
    nh_.param("tf_tolerance", tf_tolerance_, 0.1);
    nh_.param("flat", flat_, false);
  }
  void process()
  {
    try
    {
      tf::StampedTransform trans;
      tf_listener_.waitForTransform(frames_["projection"], frames_["base"],
                                    ros::Time(0), ros::Duration(0.1));
      tf_listener_.lookupTransform(frames_["projection"], frames_["base"],
                                   ros::Time(0), trans);

      tf::StampedTransform trans_target;
      tf_listener_.waitForTransform(frames_["target"], frames_["projection"],
                                    trans.stamp_, ros::Duration(0.1));
      tf_listener_.lookupTransform(frames_["target"], frames_["projection"],
                                   trans.stamp_, trans_target);

      const auto result = project(trans, trans_target);

      geometry_msgs::TransformStamped trans_out;
      tf::transformStampedTFToMsg(result, trans_out);
      if (flat_)
      {
        const float yaw = tf::getYaw(trans_out.transform.rotation);
        trans_out.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
      }
      trans_out.header.stamp = trans.stamp_ + ros::Duration(tf_tolerance_);

      tf_broadcaster_.sendTransform(trans_out);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN_ONCE("%s", e.what());
    }
  }
  void cbTimer(const ros::TimerEvent &event)
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_projection");

  TfProjectionNode proj;
  proj.spin();

  return 0;
}
