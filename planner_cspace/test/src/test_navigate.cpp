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
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstddef>

#include <gtest/gtest.h>

TEST(Navigate, Navigate)
{
  nav_msgs::OccupancyGrid::ConstPtr map;

  const boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)> cb_map =
      [&map](const nav_msgs::OccupancyGrid::ConstPtr& msg) -> void
  {
    map = msg;
    std::cerr << "Map received." << std::endl;
  };

  ros::NodeHandle nh("");
  tf2_ros::Buffer tfbuf;
  tf2_ros::TransformListener tfl(tfbuf);
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("patrol_nodes", 1, true);
  ros::Subscriber sub_map = nh.subscribe("map", 1, cb_map);

  ros::Duration(2.0).sleep();
  nav_msgs::Path path;
  path.poses.resize(2);
  path.header.frame_id = "map";
  path.poses[0].header.frame_id = path.header.frame_id;
  path.poses[0].pose.position.x = 1.7;
  path.poses[0].pose.position.y = 2.8;
  path.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  path.poses[1].header.frame_id = path.header.frame_id;
  path.poses[1].pose.position.x = 1.9;
  path.poses[1].pose.position.y = 2.8;
  path.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  pub_path.publish(path);

  tf2::Transform goal;
  tf2::fromMsg(path.poses.back().pose, goal);

  ros::Rate wait(10);
  while (ros::ok())
  {
    ros::spinOnce();
    wait.sleep();

    tf2::Stamped<tf2::Transform> trans;
    try
    {
      const ros::Time now = ros::Time::now();
      geometry_msgs::TransformStamped trans_tmp =
          tfbuf.lookupTransform("map", "base_link", now, ros::Duration(0.5));
      tf2::fromMsg(trans_tmp, trans);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << e.what() << std::endl;
      continue;
    }

    auto goal_rel = trans.inverse() * goal;
    if (goal_rel.getOrigin().length() < 0.2 &&
        fabs(tf2::getYaw(goal_rel.getRotation())) < 0.2)
    {
      std::cerr << "Navagation success." << std::endl;
      ros::Duration(2.0).sleep();
      return;
    }

    if (!map)
    {
      std::cerr << "Waiting map." << std::endl;
      continue;
    }

    for (int x = -2; x <= 2; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        const tf2::Vector3 pos =
            trans * tf2::Vector3(x * map->info.resolution, y * map->info.resolution, 0);
        const int map_x = pos.x() / map->info.resolution;
        const int map_y = pos.y() / map->info.resolution;
        const size_t addr = map_x + map_y * map->info.width;
        ASSERT_LT(addr, map->data.size());
        ASSERT_LT(map_x, static_cast<int>(map->info.width));
        ASSERT_LT(map_y, static_cast<int>(map->info.height));
        ASSERT_GE(map_x, 0);
        ASSERT_GE(map_y, 0);
        ASSERT_NE(map->data[addr], 100);
      }
    }
  }
  ASSERT_TRUE(false);
}

TEST(Navigate, GlobalPlan)
{
  nav_msgs::OccupancyGrid::ConstPtr map;

  const boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)> cb_map =
      [&map](const nav_msgs::OccupancyGrid::ConstPtr& msg) -> void
  {
    map = msg;
    std::cerr << "Map received." << std::endl;
  };

  ros::NodeHandle nh("");
  tf2_ros::Buffer tfbuf;
  tf2_ros::TransformListener tfl(tfbuf);
  ros::ServiceClient srv_plan =
      nh.serviceClient<nav_msgs::GetPlanRequest, nav_msgs::GetPlanResponse>(
          "/planner_3d/make_plan");
  ros::Subscriber sub_map = nh.subscribe("map", 1, cb_map);

  ros::Duration(2.0).sleep();
  ros::spinOnce();
  ASSERT_TRUE(static_cast<bool>(map));

  nav_msgs::GetPlanRequest req;
  nav_msgs::GetPlanResponse res;

  req.start.header.frame_id = "map";
  req.start.pose.position.x = 2.0;
  req.start.pose.position.y = 0.45;
  req.start.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 3.14));

  req.goal.header.frame_id = "map";
  req.goal.pose.position.x = 1.2;
  req.goal.pose.position.y = 1.9;
  req.goal.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -3.14));
  ASSERT_FALSE(srv_plan.call(req, res));

  req.goal.header.frame_id = "map";
  req.goal.pose.position.x = 1.9;
  req.goal.pose.position.y = 2.8;
  req.goal.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -1.57));
  ASSERT_TRUE(srv_plan.call(req, res));

  ASSERT_NEAR(req.start.pose.position.x, res.plan.poses.front().pose.position.x, 0.1);
  ASSERT_NEAR(req.start.pose.position.y, res.plan.poses.front().pose.position.y, 0.1);
  ASSERT_NEAR(req.goal.pose.position.x, res.plan.poses.back().pose.position.x, 0.1);
  ASSERT_NEAR(req.goal.pose.position.y, res.plan.poses.back().pose.position.y, 0.1);

  for (const geometry_msgs::PoseStamped& p : res.plan.poses)
  {
    const int map_x = p.pose.position.x / map->info.resolution;
    const int map_y = p.pose.position.y / map->info.resolution;
    const size_t addr = map_x + map_y * map->info.width;
    ASSERT_LT(addr, map->data.size());
    ASSERT_LT(map_x, static_cast<int>(map->info.width));
    ASSERT_LT(map_y, static_cast<int>(map->info.height));
    ASSERT_GE(map_x, 0);
    ASSERT_GE(map_y, 0);
    ASSERT_NE(map->data[addr], 100);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate");

  return RUN_ALL_TESTS();
}
