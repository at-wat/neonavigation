/*
 * Copyright (c) 2021, the neonavigation authors
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

#include <vector>

#include <trajectory_tracker_test.h>
#include <trajectory_tracker/TrajectoryTrackerConfig.h>

#include <dynamic_reconfigure/client.h>

class TrajectoryTrackerOvershootTest : public TrajectoryTrackerTest
{
protected:
  void runTest(const double goal_tolerance_lin_vel, const double goal_tolerance_ang_vel,
               const double linear_vel, const double rotation_vel, const int32_t expected_status)
  {
    initState(Eigen::Vector2d(0, 0), 0);

    std::vector<Eigen::Vector3d> poses;
    for (double x = 0.0; x < 0.5; x += 0.01)
      poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
    poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
    waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

    ParamType param;
    ASSERT_TRUE(getConfig(param));
    param.goal_tolerance_lin_vel = goal_tolerance_lin_vel;
    param.goal_tolerance_ang_vel = goal_tolerance_ang_vel;
    ASSERT_TRUE(setConfig(param));

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 0.5;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = rotation_vel;

    ros::Rate rate(50);
    const ros::Time initial_time = ros::Time::now();
    const ros::Time time_limit = initial_time + ros::Duration(5.0);
    while (ros::ok() && time_limit > ros::Time::now())
    {
      odom.header.stamp = ros::Time::now();
      publishTransform(odom);
      rate.sleep();
      ros::spinOnce();
      if ((status_->header.stamp > initial_time + ros::Duration(0.5)) && (status_->status == expected_status))
      {
        return;
      }
    }
    FAIL() << "Expected status: " << expected_status << " Actual status: " << status_->status;
  }
};

TEST_F(TrajectoryTrackerOvershootTest, NoVelocityToleranceWithRemainingLinearVel)
{
  SCOPED_TRACE("NoVelocityToleranceWithRemainingLinearVel");
  runTest(0.0, 0.0, 0.1, 0.0, trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL);
}

TEST_F(TrajectoryTrackerOvershootTest, NoVelocityToleranceWithRemainingAngularVel)
{
  SCOPED_TRACE("NoVelocityToleranceWithRemainingAngularVel");
  runTest(0.0, 0.0, 0.0, 0.1, trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL);
}

TEST_F(TrajectoryTrackerOvershootTest, LinearVelocityToleranceWithRemainingLinearVel)
{
  SCOPED_TRACE("LinearVelocityToleranceWithRemainingLinearVel");
  runTest(0.05, 0.0, 0.1, 0.0, trajectory_tracker_msgs::TrajectoryTrackerStatus::FOLLOWING);
}

TEST_F(TrajectoryTrackerOvershootTest, LinearVelocityToleranceWithRemainingAngularVel)
{
  SCOPED_TRACE("LinearVelocityToleranceWithRemainingAngularVel");
  runTest(0.05, 0.0, 0.0, 0.1, trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL);
}

TEST_F(TrajectoryTrackerOvershootTest, AngularrVelocityToleranceWithRemainingLinearVel)
{
  SCOPED_TRACE("AngularrVelocityToleranceWithRemainingLinearVel");
  runTest(0.0, 0.05, 0.1, 0.0, trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL);
}

TEST_F(TrajectoryTrackerOvershootTest, AngularrVelocityToleranceWithRemainingAngularVel)
{
  SCOPED_TRACE("AngularrVelocityToleranceWithRemainingAngularVel");
  runTest(0.0, 0.05, 0.0, 0.1, trajectory_tracker_msgs::TrajectoryTrackerStatus::FOLLOWING);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_trajectory_tracker_overshoot");

  return RUN_ALL_TESTS();
}
