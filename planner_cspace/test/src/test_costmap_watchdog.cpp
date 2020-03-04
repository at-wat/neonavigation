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

#include <string>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <planner_cspace_msgs/PlannerStatus.h>

#include <gtest/gtest.h>

TEST(Planner3D, CostmapWatchdog)
{
  int cnt = 0;
  planner_cspace_msgs::PlannerStatus::ConstPtr status;
  nav_msgs::Path::ConstPtr path;
  diagnostic_msgs::DiagnosticArray::ConstPtr diag;

  const boost::function<void(const planner_cspace_msgs::PlannerStatus::ConstPtr&)> cb_status =
      [&status, &cnt](const planner_cspace_msgs::PlannerStatus::ConstPtr& msg) -> void
  {
    status = msg;
    cnt++;
  };
  const boost::function<void(const nav_msgs::Path::ConstPtr&)> cb_path =
      [&path](const nav_msgs::Path::ConstPtr& msg) -> void
  {
    path = msg;
  };
  const boost::function<void(const diagnostic_msgs::DiagnosticArray::ConstPtr&)> cb_diag =
      [&diag](const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) -> void
  {
    diag = msg;
  };

  ros::NodeHandle nh("");
  ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("goal", 1, true);
  ros::Publisher pub_cost_update = nh.advertise<costmap_cspace_msgs::CSpace3DUpdate>("costmap_update", 1);
  ros::Subscriber sub_status = nh.subscribe("planner_3d/status", 1, cb_status);
  ros::Subscriber sub_path = nh.subscribe("path", 1, cb_path);
  ros::Subscriber sub_diag = nh.subscribe("diagnostics", 1, cb_diag);

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 1.9;
  goal.pose.position.y = 2.8;
  goal.pose.orientation.w = 1.0;
  // Assure that goal is received after map in planner_3d.
  ros::Duration(0.5).sleep();
  pub_goal.publish(goal);

  ros::Rate rate(10);
  while (ros::ok())
  {
    // cnt increments in 5 Hz at maximum
    if (cnt == 0 || cnt > 8)
    {
      costmap_cspace_msgs::CSpace3DUpdate update;
      update.header.stamp = ros::Time::now();
      update.header.frame_id = "map";
      update.width = update.height = update.angle = 0;
      pub_cost_update.publish(update);
    }

    ros::spinOnce();
    rate.sleep();

    if (!status)
      continue;

    if (5 < cnt && cnt < 8)
    {
      ASSERT_EQ(status->error, planner_cspace_msgs::PlannerStatus::DATA_MISSING);

      ASSERT_TRUE(static_cast<bool>(path));
      ASSERT_EQ(path->poses.size(), 0u);

      ASSERT_TRUE(static_cast<bool>(diag));
      ASSERT_EQ(diag->status.size(), 1u);
      ASSERT_EQ(diag->status[0].level, diagnostic_msgs::DiagnosticStatus::ERROR);
      ASSERT_NE(diag->status[0].message.find("missing"), std::string::npos);
    }
    else if (10 < cnt && cnt < 13)
    {
      ASSERT_EQ(status->status, planner_cspace_msgs::PlannerStatus::DOING);
      ASSERT_EQ(status->error, planner_cspace_msgs::PlannerStatus::GOING_WELL);

      ASSERT_EQ(diag->status.size(), 1u);
      ASSERT_EQ(diag->status[0].level, diagnostic_msgs::DiagnosticStatus::OK);
      ASSERT_NE(diag->status[0].message.find("well"), std::string::npos);
    }
    else if (cnt >= 13)
    {
      return;
    }
  }
  ASSERT_TRUE(ros::ok());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_navigate");

  return RUN_ALL_TESTS();
}
