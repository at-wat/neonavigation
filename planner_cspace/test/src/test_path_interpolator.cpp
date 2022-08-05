/*
 * Copyright (c) 2022, the neonavigation authors
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

#include <list>

#include <gtest/gtest.h>

#include <costmap_cspace_msgs/MapMetaData3D.h>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/path_interpolator.h>
#include <planner_cspace/planner_3d/grid_metric_converter.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(PathInterpolator, SimpleStraight)
{
  PathInterpolator pi;
  pi.reset(4.0, 5);
  const std::list<CyclicVecInt<3, 2>> in =
      {
          CyclicVecInt<3, 2>(0, 0, 0),
          CyclicVecInt<3, 2>(1, 0, 0),
          CyclicVecInt<3, 2>(2, 0, 0),
      };
  const std::list<CyclicVecFloat<3, 2>> out = pi.interpolate(in, 0.25, 0);

  const std::list<CyclicVecFloat<3, 2>> expected =
      {
          CyclicVecFloat<3, 2>(0.00f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(0.25f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(0.50f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(0.75f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(1.00f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(1.25f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(1.50f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(1.75f, 0.f, 0.f),
          CyclicVecFloat<3, 2>(2.00f, 0.f, 0.f),
      };

  ASSERT_EQ(expected, out);
}

TEST(PathInterpolator, DuplicatedPose)
{
  PathInterpolator pi;
  pi.reset(4.0, 5);
  const std::list<CyclicVecInt<3, 2>> in =
      {
          CyclicVecInt<3, 2>(0, 0, 0),
          CyclicVecInt<3, 2>(1, 0, 0),
          CyclicVecInt<3, 2>(2, 0, 0),
      };
  const std::list<CyclicVecFloat<3, 2>> out = pi.interpolate(in, 0.4999999, 0);

  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.linear_resolution = 0.1;
  map_info.angular_resolution = M_PI / 2;
  map_info.origin.position.x = -100.5;
  map_info.origin.position.y = -200.5;

  CyclicVecFloat<3, 2> m_prev;
  for (const auto& g : out)
  {
    CyclicVecFloat<3, 2> m;

    // Introduce quantization error
    grid_metric_converter::grid2Metric<float>(
        map_info,
        g[0], g[1], g[2],
        m[0], m[1], m[2]);

    ASSERT_NE(m_prev, m);
    m_prev = m;
  }
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
