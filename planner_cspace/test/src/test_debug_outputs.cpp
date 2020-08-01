/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <sensor_msgs/PointCloud.h>

#include <gtest/gtest.h>

class DebugOutputsTest : public ::testing::Test
{
public:
  DebugOutputsTest()
    : cnt_planner_ready_(0)
    , cnt_path_(0)
    , cnt_hysteresis_(0)
    , cnt_remembered_(0)
  {
    sub_status_ = nh_.subscribe("/planner_3d/status", 1, &DebugOutputsTest::cbStatus, this);
    sub_path_ = nh_.subscribe("path", 1, &DebugOutputsTest::cbPath, this);
    sub_hysteresis_ = nh_.subscribe("/planner_3d/hysteresis_map", 1, &DebugOutputsTest::cbHysteresis, this);
    sub_remembered_ = nh_.subscribe("/planner_3d/remembered_map", 1, &DebugOutputsTest::cbRemembered, this);
    sub_distance_ = nh_.subscribe("/planner_3d/distance_map", 1, &DebugOutputsTest::cbDistance, this);

    // Wait planner
    while (ros::ok())
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      if (cnt_planner_ready_ > 5 && cnt_path_ > 5)
        break;
    }
    map_hysteresis_ = nullptr;
    map_remembered_ = nullptr;
    map_distance_ = nullptr;
    cnt_hysteresis_ = 0;
    cnt_remembered_ = 0;
    cnt_distance_ = 0;

    // Wait receiving the messages
    while (ros::ok())
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      // First hysteresis map doesn't have previous path information.
      if (cnt_hysteresis_ > 2 && cnt_remembered_ > 2)
        break;
    }
  }
  void showMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    std::cerr << std::hex;
    for (size_t y = 0; y < msg->info.height; ++y)
    {
      for (size_t x = 0; x < msg->info.width; ++x)
      {
        std::cerr << (msg->data[x + y * msg->info.width] / 11);
      }
      std::cerr << std::endl;
    }
    std::cerr << std::dec;

    std::streamsize ss = std::cerr.precision();
    std::cerr << "path:" << std::endl
              << std::setprecision(3);
    for (const auto& p : path_->poses)
      std::cerr << p.pose.position.x << ", " << p.pose.position.y << ", " << std::endl;

    std::cerr << std::setprecision(ss);
  }

protected:
  void cbHysteresis(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    cnt_hysteresis_++;
    map_hysteresis_ = msg;
  }
  void cbRemembered(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    cnt_remembered_++;
    map_remembered_ = msg;
  }
  void cbDistance(const sensor_msgs::PointCloud::ConstPtr& msg)
  {
    cnt_distance_++;
    map_distance_ = msg;
  }
  void cbStatus(const planner_cspace_msgs::PlannerStatus::ConstPtr& msg)
  {
    if (msg->error == planner_cspace_msgs::PlannerStatus::GOING_WELL &&
        msg->status == planner_cspace_msgs::PlannerStatus::DOING)
      ++cnt_planner_ready_;
  }
  void cbPath(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.size() > 0)
    {
      ++cnt_path_;
      path_ = msg;
    }
  }

  ros::NodeHandle nh_;
  nav_msgs::OccupancyGrid::ConstPtr map_hysteresis_;
  nav_msgs::OccupancyGrid::ConstPtr map_remembered_;
  sensor_msgs::PointCloud::ConstPtr map_distance_;
  nav_msgs::Path::ConstPtr path_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_path_;
  ros::Subscriber sub_hysteresis_;
  ros::Subscriber sub_remembered_;
  ros::Subscriber sub_distance_;
  int cnt_planner_ready_;
  int cnt_path_;
  int cnt_hysteresis_;
  int cnt_remembered_;
  int cnt_distance_;
};

struct PositionAndValue
{
  int x;
  int y;
  char value;
};

TEST_F(DebugOutputsTest, Hysteresis)
{
  ASSERT_TRUE(static_cast<bool>(map_hysteresis_));

  // Robot is at (25, 4) and goal is at (10, 4)
  const PositionAndValue data_set[] =
      {
        { 25, 4, 0 },
        { 20, 4, 0 },
        { 15, 4, 0 },
        { 10, 4, 0 },
        { 25, 1, 100 },
        { 20, 1, 100 },
        { 15, 1, 100 },
        { 10, 1, 100 },
        { 25, 7, 100 },
        { 20, 7, 100 },
        { 15, 7, 100 },
        { 10, 7, 100 },
      };
  for (auto data : data_set)
  {
    const size_t addr = data.x + data.y * map_hysteresis_->info.width;
    EXPECT_EQ(data.value, map_hysteresis_->data[addr]) << "x: " << data.x << ", y: " << data.y;
  }
  if (::testing::Test::HasFailure())
  {
    showMap(map_hysteresis_);
  }
}

TEST_F(DebugOutputsTest, Remembered)
{
  ASSERT_TRUE(static_cast<bool>(map_remembered_));

  // Robot is at (25, 5) and obstacles are placed at
  // (0, 0)-(31, 0) and (18, 10)-(31, 10).
  // Costmap is expanded by 1 grid.
  const PositionAndValue data_set[] =
      {
        { 17, 0, 100 },   // occupied
        { 17, 1, 100 },   // expanded
        { 17, 2, 0 },     // free
        { 17, 8, 0 },     // free
        { 17, 9, 100 },   // expanded
        { 17, 10, 100 },  // occupied
      };
  for (auto data : data_set)
  {
    const size_t addr = data.x + data.y * map_remembered_->info.width;
    EXPECT_EQ(data.value, map_remembered_->data[addr]) << "x: " << data.x << ", y: " << data.y;
  }
  if (::testing::Test::HasFailure())
  {
    showMap(map_remembered_);
  }
}

TEST_F(DebugOutputsTest, Distance)
{
  ASSERT_TRUE(static_cast<bool>(map_distance_));

  // Robot is at (2.5, 0.5) and goal is at (1.0, 0.5)
  for (size_t i = 0; i < map_distance_->points.size(); ++i)
  {
    const float x = map_distance_->points[i].x;
    const float y = map_distance_->points[i].y;
    const float d = map_distance_->channels[0].values[i];
    if (std::abs(y - 0.5) < 0.05)
    {
      const float dist_from_goal = std::abs(x - 1.0);
      ASSERT_NEAR(dist_from_goal, d, 0.15);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_debug_outputs");

  return RUN_ALL_TESTS();
}
