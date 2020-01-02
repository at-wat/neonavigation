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

#include <cmath>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtest/gtest.h>

constexpr float TOLERANCE = 1e-6;

bool isOnBottomSurface(const float x, const float y, const float z)
{
  return -TOLERANCE <= x && x <= 0.5 + TOLERANCE &&
         -TOLERANCE <= y && y <= 0.1 + TOLERANCE &&
         -TOLERANCE <= z && z <= 0.001 + TOLERANCE;
}
bool isOnSideSurface(const float x, const float y, const float z)
{
  return -TOLERANCE <= x && x <= 0.001 + TOLERANCE &&
         -TOLERANCE <= y && y <= 0.1 + TOLERANCE &&
         -TOLERANCE <= z && z <= 0.2 + TOLERANCE;
}
bool isOnCorner(const float x, const float y, const float z)
{
  return -TOLERANCE <= x && x <= 0.05 + TOLERANCE &&
         -TOLERANCE <= y && y <= 0.1 + TOLERANCE &&
         -TOLERANCE <= z && z <= 0.05 + TOLERANCE;
}

TEST(ObjToPointCloud, PointCloud)
{
  ros::NodeHandle nh;
  sensor_msgs::PointCloud2::ConstPtr cloud;

  const boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> cb =
      [&cloud](const sensor_msgs::PointCloud2::ConstPtr& msg) -> void
  {
    cloud = msg;
  };
  ros::Subscriber sub = nh.subscribe("mapcloud", 1, cb);

  ros::Rate rate(10.0);
  for (int i = 0; i < 30 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (cloud)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(cloud));
  ASSERT_NEAR(
      cloud->width * cloud->height,
      std::lround((0.5 * 0.1 + 0.1 * 0.2) / (0.05 * 0.05)),
      10);

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z)
  {
    ASSERT_TRUE(
        isOnBottomSurface(*iter_x, *iter_y, *iter_z) ||
        isOnSideSurface(*iter_x, *iter_y, *iter_z) ||
        isOnCorner(*iter_x, *iter_y, *iter_z))
        << *iter_x << ", " << *iter_y << ", " << *iter_z << " is not on the expected surface";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_obj_to_pointcloud");

  return RUN_ALL_TESTS();
}
