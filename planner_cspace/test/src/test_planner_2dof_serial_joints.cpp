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
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <gtest/gtest.h>

TEST(Planner2DOFSerialJoints, Plan)
{
  ros::NodeHandle nh;
  ros::Publisher pub_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
  ros::Publisher pub_cmd = nh.advertise<trajectory_msgs::JointTrajectory>("trajectory_in", 1, true);

  trajectory_msgs::JointTrajectory::ConstPtr planned;
  const auto cb_plan = [&planned](const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    planned = msg;
  };
  ros::Subscriber sub_plan = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, cb_plan);

  sensor_msgs::JointState s;
  s.name.push_back("front");
  s.position.push_back(-1.57);
  s.name.push_back("rear");
  s.position.push_back(0.0);

  trajectory_msgs::JointTrajectory cmd;
  cmd.joint_names.push_back("front");
  cmd.joint_names.push_back("rear");
  trajectory_msgs::JointTrajectoryPoint p;
  p.positions.push_back(-4.71);
  p.positions.push_back(0.0);
  cmd.points.push_back(p);

  pub_state.publish(s);
  ros::Duration(1.0).sleep();

  ros::Rate rate(1);
  while (ros::ok())
  {
    pub_state.publish(s);
    pub_cmd.publish(cmd);

    ros::spinOnce();
    rate.sleep();
    if (planned)
      break;
  }
  ASSERT_TRUE(ros::ok());

  ASSERT_EQ(2u, planned->joint_names.size());
  ASSERT_EQ("front", planned->joint_names[0]);
  ASSERT_EQ("rear", planned->joint_names[1]);

  // collision at front=-3.14, rear=0.0 must be avoided.
  const float fc = -3.14;
  const float rc = 0.0;
  for (int i = 1; i < static_cast<int>(planned->points.size()); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint& p0 = planned->points[i - 1];
    const trajectory_msgs::JointTrajectoryPoint& p1 = planned->points[i];
    ASSERT_EQ(2u, p0.positions.size());
    ASSERT_EQ(2u, p1.positions.size());

    const float f0 = p0.positions[0];
    const float r0 = p0.positions[1];
    const float f1 = p1.positions[0];
    const float r1 = p1.positions[1];

    ASSERT_LT(-6.28, f0);
    ASSERT_GT(0.0, f0);
    ASSERT_LT(-3.14, r0);
    ASSERT_GT(3.14, r0);
    ASSERT_LT(-6.28, f1);
    ASSERT_GT(0.0, f1);
    ASSERT_LT(-3.14, r1);
    ASSERT_GT(3.14, r1);

    const float front_diff = f1 - f0;
    const float rear_diff = r1 - r0;

    const float d =
        std::abs(
            rear_diff * fc -
            front_diff * rc +
            f1 * r0 -
            r1 * f0) /
        std::hypot(front_diff, rear_diff);
    // std::cerr << d << std::endl;
    ASSERT_GT(d, 0.15);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_planner_2dof_serial_joints");

  return RUN_ALL_TESTS();
}
