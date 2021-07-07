/*
 * Copyright (c) 2018, the neonavigation authors
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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <string>

#include <gtest/gtest.h>

TEST(TrajectoryRecorder, TfToPath)
{
  ros::NodeHandle nh("");

  nav_msgs::Path::ConstPtr path;
  int received_count = 0;
  const boost::function<void(const nav_msgs::Path::ConstPtr&)> cb_path =
      [&path, &received_count](const nav_msgs::Path::ConstPtr& msg) -> void
  {
    ++received_count;
    path = msg;
  };
  ros::Subscriber sub_path = nh.subscribe("path", 1, cb_path);
  tf2_ros::TransformBroadcaster tfb;

  const tf2::Transform points[] =
      {
          tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0)),
          tf2::Transform(tf2::Quaternion(0, 0, 1, 0), tf2::Vector3(2, 0, 0)),
          tf2::Transform(tf2::Quaternion(0, 0, 0, -1), tf2::Vector3(3, 5, 0)),
          tf2::Transform(tf2::Quaternion(0, 0, -1, 0), tf2::Vector3(-1, 5, 1)),
      };
  const size_t len = sizeof(points) / sizeof(tf2::Transform);

  ros::Duration(1.0).sleep();
  for (auto& p : points)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      geometry_msgs::TransformStamped trans =
          tf2::toMsg(tf2::Stamped<tf2::Transform>(
              p, ros::Time::now() + ros::Duration(0.1), "map"));
      trans.child_frame_id = "base_link";
      tfb.sendTransform(trans);
      ros::Duration(0.1).sleep();
    }
  }
  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(path));
  ASSERT_EQ(received_count, 1);

  ASSERT_EQ(path->poses.size(), len);
  for (size_t i = 0; i < len; ++i)
  {
    ASSERT_EQ(path->poses[i].pose.position.x, points[i].getOrigin().x());
    ASSERT_EQ(path->poses[i].pose.position.y, points[i].getOrigin().y());
    ASSERT_EQ(path->poses[i].pose.position.z, points[i].getOrigin().z());
    ASSERT_EQ(path->poses[i].pose.orientation.x, points[i].getRotation().x());
    ASSERT_EQ(path->poses[i].pose.orientation.y, points[i].getRotation().y());
    ASSERT_EQ(path->poses[i].pose.orientation.z, points[i].getRotation().z());
    ASSERT_EQ(path->poses[i].pose.orientation.w, points[i].getRotation().w());
  }

  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/trajectory_recorder/clear_path");
  std_srvs::Empty empty;
  ASSERT_TRUE(client.call(empty));

  while (received_count != 2)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ASSERT_EQ(static_cast<int>(path->poses.size()), 1);
  ASSERT_EQ(path->poses.back().pose.position.x, points[len - 1].getOrigin().x());
  ASSERT_EQ(path->poses.back().pose.position.y, points[len - 1].getOrigin().y());
  ASSERT_EQ(path->poses.back().pose.position.z, points[len - 1].getOrigin().z());
  ASSERT_EQ(path->poses.back().pose.orientation.x, points[len - 1].getRotation().x());
  ASSERT_EQ(path->poses.back().pose.orientation.y, points[len - 1].getRotation().y());
  ASSERT_EQ(path->poses.back().pose.orientation.z, points[len - 1].getRotation().z());
  ASSERT_EQ(path->poses.back().pose.orientation.w, points[len - 1].getRotation().w());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_trajectory_recorder");

  return RUN_ALL_TESTS();
}
