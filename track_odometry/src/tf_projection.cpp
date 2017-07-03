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

class tf_projection_node : public tf_projection
{
private:
  ros::NodeHandle nh;
  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;

  double rate;
  double tf_tolerance;
  bool flat;

public:
  tf_projection_node()
    : nh("~")
  {
    nh.param("base_link_frame", frames["base"], std::string("base_link"));
    nh.param("projection_frame", frames["projection"], std::string("map"));
    nh.param("target_frame", frames["target"], std::string("map"));
    nh.param("frame", frames["frame"], std::string("base_link_projected"));

    nh.param("hz", rate, 10.0);
    nh.param("tf_tolerance", tf_tolerance, 0.1);
    nh.param("flat", flat, false);
  }
  void process()
  {
    try
    {
      tf::StampedTransform trans;
      tf_listener.waitForTransform(frames["projection"], frames["base"],
                                   ros::Time(0), ros::Duration(0.1));
      tf_listener.lookupTransform(frames["projection"], frames["base"],
                                  ros::Time(0), trans);

      tf::StampedTransform trans_target;
      tf_listener.waitForTransform(frames["target"], frames["projection"],
                                   trans.stamp_, ros::Duration(0.1));
      tf_listener.lookupTransform(frames["target"], frames["projection"],
                                  trans.stamp_, trans_target);

      const auto result = project(trans, trans_target);

      geometry_msgs::TransformStamped trans_out;
      tf::transformStampedTFToMsg(result, trans_out);
      if (flat)
      {
        const float yaw = tf::getYaw(trans_out.transform.rotation);
        trans_out.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
      }
      trans_out.header.stamp = trans.stamp_ + ros::Duration(tf_tolerance);

      tf_broadcaster.sendTransform(trans_out);
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN_ONCE("%s", e.what());
    }
  }
  void spin()
  {
    ros::Rate r(rate);
    while (ros::ok())
    {
      ros::spinOnce();
      process();
      r.sleep();
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_projection");

  tf_projection_node proj;
  proj.spin();

  return 0;
}
