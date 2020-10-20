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
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <gtest/gtest.h>

class TfProjectionTest : public ::testing::TestWithParam<const char*>
{
public:
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  std::string projected_frame_;

  TfProjectionTest()
    : tfl_(tfbuf_)
  {
  }
  void SetUp() override
  {
    projected_frame_ = std::string(GetParam());
  }
};

TEST_P(TfProjectionTest, ProjectionTransform)
{
  EXPECT_TRUE(tfbuf_.canTransform("map", projected_frame_, ros::Time(0), ros::Duration(10.0)));

  geometry_msgs::TransformStamped out;
  try
  {
    out = tfbuf_.lookupTransform("map", projected_frame_, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException& e)
  {
    FAIL() << e.what();
  }

  if (projected_frame_ == "base_link_tilt_projected_with_aligned")
  {
    const tf2::Transform trans(
        tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), -M_PI / 6));
    const tf2::Vector3 result = trans * tf2::Vector3(1, 2, 3);
    ASSERT_NEAR(out.transform.translation.x, result[0], 1e-4);
  }
  else
  {
    ASSERT_NEAR(out.transform.translation.x, 1, 1e-4);
  }
  ASSERT_NEAR(out.transform.translation.y, 2, 1e-4);
  ASSERT_NEAR(out.transform.translation.z, 0, 1e-4);
  ASSERT_NEAR(out.transform.rotation.x, 0, 1e-4);
  ASSERT_NEAR(out.transform.rotation.y, 0, 1e-4);
  ASSERT_NEAR(out.transform.rotation.z, 0.7071, 1e-4);
  ASSERT_NEAR(out.transform.rotation.w, 0.7071, 1e-4);
  ASSERT_EQ(out.header.frame_id, "map");
  ASSERT_EQ(out.child_frame_id, projected_frame_);
}

INSTANTIATE_TEST_CASE_P(
    ProjectionTransformInstance, TfProjectionTest,
    ::testing::Values(
        "base_link_projected",
        "base_link_projected2",
        "base_link_tilt_projected",
        "base_link_tilt_projected_with_aligned"));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_tf_projection_node");

  return RUN_ALL_TESTS();
}
