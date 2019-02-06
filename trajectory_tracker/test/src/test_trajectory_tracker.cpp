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
#include <vector>

#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <trajectory_tracker_msgs/TrajectoryTrackerStatus.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>

#include <gtest/gtest.h>

class TrajectoryTrackerTest : public ::testing::Test
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_status_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_vel_;
  tf2_ros::TransformBroadcaster tfb_;

  ros::Time cmd_vel_time_;

  void cbStatus(const trajectory_tracker_msgs::TrajectoryTrackerStatus::ConstPtr& msg)
  {
    status_ = msg;
  }
  void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
  {
    const ros::Time now = ros::Time::now();
    if (cmd_vel_time_ == ros::Time(0))
      cmd_vel_time_ = now;
    const float dt = (now - cmd_vel_time_).toSec();

    yaw_ += msg->angular.z * dt;
    pos_ += Eigen::Vector2d(std::cos(yaw_), std::sin(yaw_)) * msg->linear.x * dt;
    cmd_vel_time_ = now;
    cmd_vel_ = msg;
  }

public:
  Eigen::Vector2d pos_;
  double yaw_;
  trajectory_tracker_msgs::TrajectoryTrackerStatus::ConstPtr status_;
  geometry_msgs::Twist::ConstPtr cmd_vel_;

  TrajectoryTrackerTest()
    : nh_("")
  {
    sub_cmd_vel_ = nh_.subscribe(
        "cmd_vel", 1, &TrajectoryTrackerTest::cbCmdVel, this);
    sub_status_ = nh_.subscribe(
        "trajectory_tracker/status", 1, &TrajectoryTrackerTest::cbStatus, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
    pub_path_vel_ = nh_.advertise<trajectory_tracker_msgs::PathWithVelocity>("path_velocity", 1, true);
  }
  void initState(const Eigen::Vector2d& pos, const float yaw)
  {
    nav_msgs::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();
    pub_path_.publish(path);

    // Wait until trajectory_tracker node
    ros::Rate rate(10);
    while (ros::ok())
    {
      yaw_ = yaw;
      pos_ = pos;
      publishTransform();

      rate.sleep();
      ros::spinOnce();
      if (status_)
        break;
    }
  }
  void waitUntilStart()
  {
    ros::Rate rate(50);
    while (ros::ok())
    {
      publishTransform();
      rate.sleep();
      ros::spinOnce();
      if (status_->status == trajectory_tracker_msgs::TrajectoryTrackerStatus::FOLLOWING)
        break;
    }
  }
  void publishPath(const std::vector<Eigen::Vector3d>& poses)
  {
    nav_msgs::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    for (const Eigen::Vector3d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      path.poses.push_back(pose);
    }
    pub_path_.publish(path);
  }
  void publishPathVelocity(const std::vector<Eigen::Vector4d>& poses)
  {
    trajectory_tracker_msgs::PathWithVelocity path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    for (const Eigen::Vector4d& p : poses)
    {
      const Eigen::Quaterniond q(Eigen::AngleAxisd(p[2], Eigen::Vector3d(0, 0, 1)));

      trajectory_tracker_msgs::PoseStampedWithVelocity pose;
      pose.header.frame_id = path.header.frame_id;
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      pose.linear_velocity.x = p[3];

      path.poses.push_back(pose);
    }
    // needs sleep to prevent that the empty path from initState arrives later.
    ros::Duration(0.5).sleep();
    pub_path_vel_.publish(path);
  }
  void publishTransform()
  {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_, Eigen::Vector3d(0, 0, 1)));
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.header.stamp = ros::Time::now() + ros::Duration(0.1);
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = pos_[0];
    trans.transform.translation.y = pos_[1];
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
    tfb_.sendTransform(trans);
  }
};

TEST_F(TrajectoryTrackerTest, StraightStop)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01)
    poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  publishPath(poses);

  waitUntilStart();

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
  for (int i = 0; i < 25; ++i)
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
  }

  ASSERT_NEAR(yaw_, 0.0, 1e-2);
  ASSERT_NEAR(pos_[0], 0.5, 1e-2);
  ASSERT_NEAR(pos_[1], 0.0, 1e-2);
}

TEST_F(TrajectoryTrackerTest, StraightStopConvergence)
{
  const double vels[] =
      {
        0.02, 0.05, 0.1, 0.2, 0.5, 1.0
      };
  const double path_length = 2.0;
  for (const double vel : vels)
  {
    const std::string info_message = "linear vel: " + std::to_string(vel);

    initState(Eigen::Vector2d(0, 0.01), 0);

    std::vector<Eigen::Vector4d> poses;
    for (double x = 0.0; x < path_length; x += 0.01)
      poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, vel));
    poses.push_back(Eigen::Vector4d(path_length, 0.0, 0.0, vel));
    publishPathVelocity(poses);

    waitUntilStart();

    ros::Rate rate(50);
    const ros::Time start = ros::Time::now();
    while (ros::ok())
    {
      ASSERT_LT(ros::Time::now() - start, ros::Duration(5.0 + path_length / vel)) << info_message;

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

    EXPECT_NEAR(yaw_, 0.0, 1e-2) << info_message;
    EXPECT_NEAR(pos_[0], path_length, 1e-2) << info_message;
    EXPECT_NEAR(pos_[1], 0.0, 1e-2) << info_message;
  }
}

TEST_F(TrajectoryTrackerTest, StraightVelocityChange)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector4d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01)
    poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, 0.3));
  for (double x = 0.5; x < 1.5; x += 0.01)
    poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, 0.5));
  poses.push_back(Eigen::Vector4d(1.5, 0.0, 0.0, 0.5));
  publishPathVelocity(poses);

  waitUntilStart();

  ros::Rate rate(50);
  const ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ASSERT_LT(ros::Time::now() - start, ros::Duration(10.0));

    publishTransform();
    rate.sleep();
    ros::spinOnce();
    if (0.3 < pos_[0] && pos_[0] < 0.4)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.3, 1e-2);
    }
    else if (0.9 < pos_[0] && pos_[0] < 1.0)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.5, 1e-2);
    }

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
  ASSERT_NEAR(pos_[0], 1.5, 1e-2);
  ASSERT_NEAR(pos_[1], 0.0, 1e-2);
}

TEST_F(TrajectoryTrackerTest, CurveFollow)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  for (double t = 0.0; t < 1.0; t += 0.01)
  {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.005);
    poses.push_back(p);
  }
  for (double t = 0.0; t < 1.0; t += 0.01)
  {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.0);
    poses.push_back(p);
  }
  publishPath(poses);

  waitUntilStart();

  ros::Rate rate(50);
  const ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ASSERT_LT(ros::Time::now() - start, ros::Duration(20.0));

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

  ASSERT_NEAR(yaw_, p[2], 1e-2);
  ASSERT_NEAR(pos_[0], p[0], 1e-1);
  ASSERT_NEAR(pos_[1], p[1], 1e-1);
}

TEST_F(TrajectoryTrackerTest, InPlaceTurn)
{
  const float init_yaw_array[] =
      {
        0.0,
        3.0
      };
  for (const float init_yaw : init_yaw_array)
  {
    initState(Eigen::Vector2d(0, 0), init_yaw);

    const float target_angle_array[] =
        {
          0.5,
          -0.5
        };
    for (const float ang : target_angle_array)
    {
      std::vector<Eigen::Vector3d> poses;
      poses.push_back(Eigen::Vector3d(0.0, 0.0, init_yaw + ang));
      publishPath(poses);

      waitUntilStart();

      ros::Rate rate(50);
      const ros::Time start = ros::Time::now();
      while (ros::ok())
      {
        ASSERT_LT(ros::Time::now() - start, ros::Duration(10.0));

        publishTransform();
        rate.sleep();
        ros::spinOnce();

        if (cmd_vel_)
        {
          ASSERT_GT(cmd_vel_->angular.z * ang, -1e-2);
        }
        if (status_)
        {
          ASSERT_LT(status_->angle_remains * ang, 1e-2);
        }

        if (status_->status == trajectory_tracker_msgs::TrajectoryTrackerStatus::GOAL)
          break;
      }
      ASSERT_TRUE(static_cast<bool>(cmd_vel_));
      for (int i = 0; i < 25; ++i)
      {
        publishTransform();
        rate.sleep();
        ros::spinOnce();
      }

      ASSERT_NEAR(yaw_, init_yaw + ang, 1e-2);
    }
  }
}

TEST_F(TrajectoryTrackerTest, SwitchBack)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  for (double t = 0.0; t < 0.5; t += 0.01)
  {
    p -= Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, -0.01);
    poses.push_back(p);
  }
  for (double t = 0.0; t < 0.5; t += 0.01)
  {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.01);
    poses.push_back(p);
  }
  publishPath(poses);

  waitUntilStart();

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
  for (int i = 0; i < 25; ++i)
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
  }

  ASSERT_NEAR(yaw_, p[2], 1e-2);
  ASSERT_NEAR(pos_[0], p[0], 1e-1);
  ASSERT_NEAR(pos_[1], p[1], 1e-1);
}

TEST_F(TrajectoryTrackerTest, SwitchBackWithPathUpdate)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  std::vector<Eigen::Vector3d> poses_second_half;
  Eigen::Vector3d p(0.0, 0.0, 0.0);
  for (double t = 0.0; t < 0.5; t += 0.01)
  {
    p -= Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, -0.01);
    poses.push_back(p);
  }
  const Eigen::Vector2d pos_local_goal = p.head<2>();
  for (double t = 0.0; t < 1.0; t += 0.01)
  {
    p += Eigen::Vector3d(std::cos(p[2]) * 0.05, std::sin(p[2]) * 0.05, 0.01);
    poses.push_back(p);
    poses_second_half.push_back(p);
  }
  publishPath(poses);

  waitUntilStart();

  int cnt_arrive_local_goal(0);
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

    if ((pos_local_goal - pos_).norm() < 0.1)
      cnt_arrive_local_goal++;

    if (cnt_arrive_local_goal > 25)
      publishPath(poses_second_half);
    else
      publishPath(poses);
  }
  for (int i = 0; i < 25; ++i)
  {
    publishTransform();
    rate.sleep();
    ros::spinOnce();
  }

  ASSERT_NEAR(yaw_, p[2], 1e-2);
  ASSERT_NEAR(pos_[0], p[0], 1e-1);
  ASSERT_NEAR(pos_[1], p[1], 1e-1);
}

void timeSource()
{
  ros::NodeHandle nh("/");
  bool use_sim_time;
  nh.param("/use_sim_time", use_sim_time, false);
  if (!use_sim_time)
    return;

  ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);

  ros::WallRate rate(500.0);  // 500% speed
  ros::WallTime time = ros::WallTime::now();
  while (ros::ok())
  {
    rosgraph_msgs::Clock clock;
    clock.clock.fromNSec(time.toNSec());
    pub.publish(clock);
    rate.sleep();
    time += ros::WallDuration(0.01);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_trajectory_tracker");

  boost::thread time_thread(timeSource);

  return RUN_ALL_TESTS();
}
