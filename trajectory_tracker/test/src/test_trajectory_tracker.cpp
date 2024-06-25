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

#include <algorithm>
#include <string>
#include <vector>

#include "trajectory_tracker_test.h"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

class RosRate : public rclcpp::RateBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RosRate)

  explicit RosRate(double rate, rclcpp::Node& node)
    : RosRate(duration_cast<nanoseconds>(duration<double>(1.0 / rate)), node)
  {
  }
  explicit RosRate(std::chrono::nanoseconds period, rclcpp::Node& node)
    : node_(node.get_node_base_interface())
    , clock_(node.get_clock())
    , period_(period)
  {
    last_interval_ = clock_->now();
  }

  virtual bool sleep()
  {
    // Time coming into sleep
    auto now = clock_->now();
    // Time of next interval
    auto next_interval = last_interval_ + period_;
    // Detect backwards time flow
    if (now < last_interval_)
    {
      // Best thing to do is to set the next_interval to now + period
      next_interval = now + period_;
    }
    // Calculate the time to sleep
    auto time_to_sleep = next_interval - now;
    // Update the interval
    last_interval_ += period_;
    // If the time_to_sleep is negative or zero, don't sleep
    if (time_to_sleep <= std::chrono::seconds(0))
    {
      // If an entire cycle was missed then reset next interval.
      // This might happen if the loop took more than a cycle.
      // Or if time jumps forward.
      if (now > next_interval + period_)
      {
        last_interval_ = now + period_;
      }
      // Either way do not sleep and return false
      return false;
    }
    // Sleep (will get interrupted by ctrl-c, may not sleep full time)
    while (clock_->now() < next_interval)
    {
      rclcpp::spin_some(node_);
      clock_->sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  virtual bool is_steady() const
  {
    return false;
  }

  virtual void reset()
  {
    last_interval_ = clock_->now();
  }

  std::chrono::nanoseconds period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

private:
  RCLCPP_DISABLE_COPY(RosRate)

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::Time last_interval_;
};

TEST_F(TrajectoryTrackerTest, StraightStop)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector3d> poses;
  for (double x = 0.0; x < 0.5; x += 0.01)
    poses.push_back(Eigen::Vector3d(x, 0.0, 0.0));
  poses.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  RosRate rate(50, *this);
  const rclcpp::Time start = now();
  while (rclcpp::ok())
  {
    if (now() > start + rclcpp::Duration::from_seconds(10.0))
    {
      FAIL() << "Timeout" << std::endl
             << "Pos " << pos_ << std::endl
             << "Yaw " << yaw_ << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }
    publishTransform();
    rate.sleep();
    rclcpp::spin_some(get_node_base_interface());
    if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
      break;
  }
  for (int j = 0; j < 5; ++j)
  {
    for (int i = 0; i < 5; ++i)
    {
      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(yaw_, 0.0, error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[0], 0.5, error_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[1], 0.0, error_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
}

TEST_F(TrajectoryTrackerTest, StraightStopOvershoot)
{
  const double resolutions[] = {
      0.1,
      0.001,  // default epsilon
      0.0001,
  };
  for (const double resolution : resolutions)
  {
    const std::string info_message = "resolution: " + std::to_string(resolution);

    initState(Eigen::Vector2d(1, 0), 0);

    std::vector<Eigen::Vector3d> poses;
    for (double x = 0.0; x < 0.5 - resolution; x += 0.1)
      poses.push_back(Eigen::Vector3d(x, 0, 0));
    poses.push_back(Eigen::Vector3d(0.5 - resolution, 0, 0));
    poses.push_back(Eigen::Vector3d(0.5, 0, 0));
    waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

    RosRate rate(50, *this);
    const rclcpp::Time start = now();
    while (rclcpp::ok())
    {
      if (now() > start + rclcpp::Duration::from_seconds(10.0))
      {
        FAIL() << "Timeout" << std::endl
               << "Pos " << pos_ << std::endl
               << "Yaw " << yaw_ << std::endl
               << "Status " << std::endl
               << status_ << std::endl
               << info_message;
      }

      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
        break;
    }
    for (int j = 0; j < 5; ++j)
    {
      for (int i = 0; i < 5; ++i)
      {
        publishTransform();
        rate.sleep();
        rclcpp::spin_some(get_node_base_interface());
      }

      // Check multiple times to assert overshoot.
      ASSERT_NEAR(yaw_, 0.0, error_ang_) << "[overshoot after goal (" << j << ")] " << info_message;
      EXPECT_NEAR(pos_[0], 0.5, error_lin_) << "[overshoot after goal (" << j << ")] " << info_message;
      ASSERT_NEAR(pos_[1], 0.0, error_lin_) << "[overshoot after goal (" << j << ")] " << info_message;
    }
    ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp) << info_message;
  }
}

TEST_F(TrajectoryTrackerTest, StraightStopConvergence)
{
  const double vels[] = {0.02, 0.05, 0.1, 0.2, 0.5, 1.0};
  const double path_length = 2.0;
  for (const double vel : vels)
  {
    const std::string info_message = "linear vel: " + std::to_string(vel);

    initState(Eigen::Vector2d(0, 0.01), 0);

    std::vector<Eigen::Vector4d> poses;
    for (double x = 0.0; x < path_length; x += 0.01)
      poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, vel));
    poses.push_back(Eigen::Vector4d(path_length, 0.0, 0.0, vel));
    waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPathVelocity, this, poses));

    RosRate rate(50, *this);
    const rclcpp::Time start = now();
    while (rclcpp::ok())
    {
      if (now() > start + rclcpp::Duration::from_seconds(5.0 + path_length / vel))
      {
        FAIL() << "Timeout" << std::endl
               << "Pos " << pos_ << std::endl
               << "Yaw " << yaw_ << std::endl
               << "Status " << std::endl
               << status_ << std::endl
               << info_message;
      }

      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
      if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
        break;
    }
    for (int j = 0; j < 5; ++j)
    {
      for (int i = 0; i < 5; ++i)
      {
        publishTransform();
        rate.sleep();
        rclcpp::spin_some(get_node_base_interface());
      }

      // Check multiple times to assert overshoot.
      EXPECT_NEAR(yaw_, 0.0, error_ang_) << "[overshoot after goal (" << j << ")] " << info_message;
      EXPECT_NEAR(pos_[0], path_length, error_lin_) << "[overshoot after goal (" << j << ")] " << info_message;
      EXPECT_NEAR(pos_[1], 0.0, error_lin_) << "[overshoot after goal (" << j << ")] " << info_message;
    }
    ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
  }
}

TEST_F(TrajectoryTrackerTest, StraightVelocityChange)
{
  initState(Eigen::Vector2d(0, 0), 0);

  std::vector<Eigen::Vector4d> poses;
  for (double x = 0.0; x < 0.6; x += 0.01)
    poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, 0.3));
  for (double x = 0.6; x < 1.5; x += 0.01)
    poses.push_back(Eigen::Vector4d(x, 0.0, 0.0, 0.5));
  poses.push_back(Eigen::Vector4d(1.5, 0.0, 0.0, 0.5));
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPathVelocity, this, poses));

  RosRate rate(50, *this);
  const rclcpp::Time start = now();
  while (rclcpp::ok())
  {
    if (now() > start + rclcpp::Duration::from_seconds(10.0))
    {
      FAIL() << "Timeout" << std::endl
             << "Pos " << pos_ << std::endl
             << "Yaw " << yaw_ << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }

    publishTransform();
    rate.sleep();
    rclcpp::spin_some(get_node_base_interface());

    if (0.3 < pos_[0] && pos_[0] < 0.35)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.3, error_lin_);
    }
    else if (0.95 < pos_[0] && pos_[0] < 1.0)
    {
      ASSERT_NEAR(cmd_vel_->linear.x, 0.5, error_lin_);
    }

    if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
      break;
  }
  for (int j = 0; j < 5; ++j)
  {
    for (int i = 0; i < 5; ++i)
    {
      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(yaw_, 0.0, error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[0], 1.5, error_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[1], 0.0, error_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
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
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  RosRate rate(50, *this);
  const rclcpp::Time start = now();
  while (rclcpp::ok())
  {
    if (now() > start + rclcpp::Duration::from_seconds(20.0))
    {
      FAIL() << "Timeout" << std::endl
             << "Pos " << pos_ << std::endl
             << "Yaw " << yaw_ << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }

    publishTransform();
    rate.sleep();
    rclcpp::spin_some(get_node_base_interface());
    if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
      break;
  }
  for (int j = 0; j < 5; ++j)
  {
    for (int i = 0; i < 5; ++i)
    {
      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(yaw_, p[2], error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[0], p[0], error_large_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[1], p[1], error_large_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
}

TEST_F(TrajectoryTrackerTest, InPlaceTurn)
{
  const float init_yaw_array[] = {0.0, 3.0};
  for (const float init_yaw : init_yaw_array)
  {
    const std::vector<float> target_angle_array[] = {
        {0.5},
        {-0.5},
        {0.1, 0.2, 0.3, 0.4, 0.5},
        {-0.1, -0.2, -0.3, -0.4, -0.5},
    };
    for (const auto& angles : target_angle_array)
    {
      for (const bool& has_short_path : {false, true})
      {
        std::stringstream condition_name;
        condition_name << "init_yaw: " << init_yaw << ", angles: " << angles.front() << "-" << angles.back()
                       << ", has_short_path: " << has_short_path;
        initState(Eigen::Vector2d(0, 0), init_yaw);

        std::vector<Eigen::Vector3d> poses;
        if (has_short_path)
        {
          poses.push_back(Eigen::Vector3d(-std::cos(init_yaw) * 0.01, std::sin(init_yaw) * 0.01, init_yaw));
        }
        for (float ang : angles)
        {
          poses.push_back(Eigen::Vector3d(0.0, 0.0, init_yaw + ang));
        }
        waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

        RosRate rate(50, *this);
        const rclcpp::Time start = now();
        for (int i = 0; rclcpp::ok(); ++i)
        {
          if (now() > start + rclcpp::Duration::from_seconds(10.0))
          {
            FAIL() << condition_name.str() << "Timeout" << std::endl
                   << "Pos " << pos_ << std::endl
                   << "Yaw " << yaw_ << std::endl
                   << "Status " << std::endl
                   << status_ << std::endl;
          }

          publishTransform();
          rate.sleep();
          rclcpp::spin_some(get_node_base_interface());

          if (cmd_vel_ && i > 5)
          {
            ASSERT_GT(cmd_vel_->angular.z * std::copysign(1.0, angles.back()), -error_ang_)
                << "[overshoot detected] " << condition_name.str();
          }
          if (status_ && i > 5)
          {
            ASSERT_LT(status_->angle_remains * std::copysign(1.0, angles.back()), error_ang_)
                << "[overshoot detected] " << condition_name.str();
          }

          if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
            break;
        }
        ASSERT_TRUE(static_cast<bool>(cmd_vel_)) << condition_name.str();
        for (int j = 0; j < 5; ++j)
        {
          for (int i = 0; i < 5; ++i)
          {
            publishTransform();
            rate.sleep();
            rclcpp::spin_some(get_node_base_interface());
          }

          // Check multiple times to assert overshoot.
          ASSERT_NEAR(yaw_, init_yaw + angles.back(), error_ang_)
              << "[overshoot after goal (" << j << ")] " << condition_name.str();
        }
        ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
      }
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
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  RosRate rate(50, *this);
  const rclcpp::Time start = now();
  while (rclcpp::ok())
  {
    if (now() > start + rclcpp::Duration::from_seconds(10.0))
    {
      FAIL() << "Timeout" << std::endl
             << "Pos " << pos_ << std::endl
             << "Yaw " << yaw_ << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }

    publishTransform();
    rate.sleep();
    rclcpp::spin_some(get_node_base_interface());
    if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
      break;
  }
  for (int j = 0; j < 5; ++j)
  {
    for (int i = 0; i < 5; ++i)
    {
      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(yaw_, p[2], error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[0], p[0], error_large_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[1], p[1], error_large_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
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
  waitUntilStart(std::bind(&TrajectoryTrackerTest::publishPath, this, poses));

  int cnt_arrive_local_goal(0);
  RosRate rate(50, *this);
  const rclcpp::Time start = now();
  for (int i = 0; rclcpp::ok(); i++)
  {
    if (now() > start + rclcpp::Duration::from_seconds(15.0))
    {
      FAIL() << "Timeout" << std::endl
             << "Pos " << pos_ << std::endl
             << "Yaw " << yaw_ << std::endl
             << "Status " << std::endl
             << status_ << std::endl;
    }

    publishTransform();
    rate.sleep();
    rclcpp::spin_some(get_node_base_interface());
    if (status_->status == trajectory_tracker_msgs::msg::TrajectoryTrackerStatus::GOAL)
      break;

    if ((pos_local_goal - pos_).norm() < 0.1)
      cnt_arrive_local_goal++;

    if (i % 5)
    {
      // Republish path in 10Hz
      if (cnt_arrive_local_goal > 25)
      {
        publishPath(poses_second_half);
      }
      else
      {
        publishPath(poses);
      }
    }
  }
  ASSERT_GT(cnt_arrive_local_goal, 25) << "failed to update path";
  for (int j = 0; j < 5; ++j)
  {
    for (int i = 0; i < 5; ++i)
    {
      publishTransform();
      rate.sleep();
      rclcpp::spin_some(get_node_base_interface());
    }

    // Check multiple times to assert overshoot.
    ASSERT_NEAR(yaw_, p[2], error_ang_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[0], p[0], error_large_lin_) << "[overshoot after goal (" << j << ")] ";
    ASSERT_NEAR(pos_[1], p[1], error_large_lin_) << "[overshoot after goal (" << j << ")] ";
  }
  ASSERT_EQ(last_path_header_.stamp, status_->path_header.stamp);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
