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

#include <initializer_list>

#include <ros/ros.h>

#include <costmap_cspace/pointcloud_accumulator.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtest/gtest.h>

void fillInPointcloudMsg(sensor_msgs::PointCloud2& cloud, const std::initializer_list<float>& points)
{
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(points.size() / 3);
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.width = points.size();
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  std::initializer_list<float>::iterator it;
  for (it = points.begin(); it != points.end(); it += 3)
  {
    *iter_x = *it;
    *iter_y = *(it + 1);
    *iter_z = *(it + 2);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
}

TEST(PointcloudAccumulator, PushPointCloud)
{
  ros::Duration accum_duration(1.0);
  costmap_cspace::PointcloudAccumulator<sensor_msgs::PointCloud2> accum(accum_duration);

  ros::Time stamp(0);
  ros::Duration dt(0.25);
  sensor_msgs::PointCloud2 cloud;
  for (int i = 0; i < 10; i++)
  {
    fillInPointcloudMsg(cloud, {static_cast<float>(i), 0, 0});  // NOLINT
    cloud.header.stamp = stamp;
    accum.push(costmap_cspace::PointcloudAccumulator<sensor_msgs::PointCloud2>::Points(cloud, stamp));
    // check the number of clouds accumulated so far
    ASSERT_EQ(i < 5 ? i + 1 : 5, std::distance(accum.begin(), accum.end()));
    stamp += dt;
  }

  // check the timestamp difference between the oldest and latest clouds in the accumulator
  const auto& oldest = accum.begin();
  const auto& latest = std::prev(accum.end());
  ASSERT_LE(latest->header.stamp - oldest->header.stamp, accum_duration);

  // check the content of the clouds
  float expected_xs[] = {5.0, 6.0, 7.0, 8.0, 9.0};
  int idx = 0;
  for (auto& pc : accum)
  {
    sensor_msgs::PointCloud2Iterator<float> it_x(pc, "x");
    ASSERT_EQ(expected_xs[idx++], *it_x);
  }

  // check the accumulator is empty after clearing it
  accum.clear();
  ASSERT_EQ(0, std::distance(accum.begin(), accum.end()));

  // check the accumulation difference is properly updated after calling reset
  accum.reset(ros::Duration(2.0));
  for (int i = 0; i < 10; i++)
  {
    fillInPointcloudMsg(cloud, {static_cast<float>(i), 0, 0});  // NOLINT
    cloud.header.stamp = stamp;
    accum.push(costmap_cspace::PointcloudAccumulator<sensor_msgs::PointCloud2>::Points(cloud, stamp));
    stamp += dt;
  }
  ASSERT_EQ(9, std::distance(accum.begin(), accum.end()));
}

TEST(PointcloudAccumulator, BackwardCompatibility)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // check accumulator can be instantiated using the PointcloudAccumurator class
  costmap_cspace::PointcloudAccumurator<sensor_msgs::PointCloud2> accum;
  accum.reset(ros::Duration(1.0));
#pragma GCC diagnostic pop
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
