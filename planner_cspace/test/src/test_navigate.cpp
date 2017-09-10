/*
 * Copyright (c) 2017, the neonavigation authors
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
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstddef>

#include <gtest/gtest.h>

TEST(Navigate, testNavigate)
{
  nav_msgs::OccupancyGrid map;

  const boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr &)> cb_map =
      [&map](const nav_msgs::OccupancyGrid::ConstPtr &msg) -> void
  {
    map = *msg;
    std::cerr << "Map received." << std::endl;
  };

  ros::NodeHandle nh("~");
  tf::TransformListener tfl;
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/patrol/path", 1, true);
  ros::Subscriber sub_map = nh.subscribe("/map", 1, cb_map);

  ros::Duration(2.0).sleep();
  nav_msgs::Path path;
  path.poses.resize(2);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.9;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.9;
  path.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(-3.14);
  pub_path.publish(path);

  tf::Pose goal;
  tf::poseMsgToTF(path.poses.back().pose, goal);

  ros::Rate wait(10);
  while (ros::ok())
  {
    ros::spinOnce();
    wait.sleep();

    tf::StampedTransform trans;
    try
    {
      const ros::Time now = ros::Time::now();
      tfl.waitForTransform("map", "base_link", now, ros::Duration(0.5));
      tfl.lookupTransform("map", "base_link", now, trans);
    }
    catch (tf::TransformException &e)
    {
      std::cerr << e.what() << std::endl;
      continue;
    }

    auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        fabs(tf::getYaw(goal_rel.getRotation())) < 0.2)
    {
      std::cerr << "Navagation success." << std::endl;
      ros::Duration(2.0).sleep();
      return;
    }

    if (map.data.size() == 0)
    {
      std::cerr << "Waiting map." << std::endl;
      continue;
    }

    for (int x = -2; x <= 2; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        const tf::Vector3 pos = trans * tf::Vector3(x * map.info.resolution, y * map.info.resolution, 0);
        const int map_x = lroundf(pos.x() / map.info.resolution);
        const int map_y = lroundf(pos.y() / map.info.resolution);
        const size_t addr = map_x + map_y * map.info.width;
        ASSERT_LT(addr, map.data.size());
        ASSERT_LT(map_x, static_cast<int>(map.info.width));
        ASSERT_LT(map_y, static_cast<int>(map.info.height));
        ASSERT_GT(map_x, 0);
        ASSERT_GT(map_y, 0);
        ASSERT_NE(map.data[addr], 100);
      }
    }
  }
  ASSERT_TRUE(false);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate");

  return RUN_ALL_TESTS();
}
