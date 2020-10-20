/*
 * Copyright (c) 2020, the neonavigation authors
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

TEST_F(TrajectoryTrackerTest, FrameRate)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01)
    poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  ros::Rate rate(50);
  const ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ASSERT_LT(ros::Time::now() - start, ros::Duration(10.0));

    publishTransform();
    rate.sleep();
    ros::spinOnce();
    if (status_->status == trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL)
      break;
  }
  const double frame_rate = getCmdVelFrameRate();
  for (int i = 0; i < 25; ++i)
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
  }
  ASSERT_NEAR(yaw_, 0.0, 1e-2);
  ASSERT_NEAR(pos_[0], 0.5, 1e-2);
  ASSERT_NEAR(pos_[1], 0.0, 1e-2);
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
  // Parameter "hz" is set to 30.0, but the actual frame rate is around 50.0 as it is syncronized with the odometry.
  ASSERT_NEAR(50.0, frame_rate, 10.0);
}

TEST_F(TrajectoryTrackerTest, Timeout)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 2.0; x += 0.01)
    poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  poses.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  ros::Rate rate(50);
  for (int i = 0; i < 50; ++i)
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
  }
  // Wait until odometry timeout
  ros::Duration(0.2).sleep();
  ros::spinOnce();

  ASSERT_FLOAT_EQ(cmd_vel_->linear.x, 0.0);
  ASSERT_FLOAT_EQ(cmd_vel_->angular.z, 0.0);
  ASSERT_GT(pos_[0], 0.0);
  ASSERT_LT(pos_[0], 2.0);
  ASSERT_EQ(status_->status, trajectory_tracker_msgs::TrajectoryTrackerStatus::NO_PATH);

  while (ros::ok())
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
    if (status_->status == trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL)
      break;
  }
  for (int i = 0; i < 25; ++i)
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
  }
  ASSERT_NEAR(yaw_, 0.0, 1e-2);
  ASSERT_NEAR(pos_[0], 2.0, 1e-2);
  ASSERT_NEAR(pos_[1], 0.0, 1e-2);
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_trajectory_tracker_frame_rate");

  return RUN_ALL_TESTS();
}
