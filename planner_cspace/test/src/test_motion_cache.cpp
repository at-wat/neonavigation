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

#include <cmath>
#include <cstddef>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/blockmem_gridmap.h>
#include <planner_cspace/planner_3d/motion_cache.h>

#include <gtest/gtest.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(MotionCache, Generate)
{
  const int range = 4;
  const int angle = 4;
  const float angular_resolution = M_PI * 2 / angle;
  const float linear_resolution = 0.5;

  BlockMemGridmap<char, 3, 2, 0x20> gm;
  MotionCache cache;
  cache.reset(
      linear_resolution, angular_resolution, range,
      gm.getAddressor());

  // Straight motions
  const int xy_yaw_straight[][3] =
      {
          {1, 0, 0},
          {0, 1, 1},
          {-1, 0, 2},
          {0, -1, 3},
      };
  for (auto& xy_yaw : xy_yaw_straight)
  {
    for (int i = 1; i <= range + 1; ++i)
    {
      const CyclicVecInt<3, 2> goal(
          i * xy_yaw[0], i * xy_yaw[1], xy_yaw[2]);
      const auto c = cache.find(xy_yaw[2], goal);
      if (i > range)
      {
        ASSERT_EQ(c, cache.end(xy_yaw[2]));
        continue;
      }
      ASSERT_NE(c, cache.end(xy_yaw[2]));
      for (const auto& p : c->second.getMotion())
      {
        // Must be in the same quadrant
        if (xy_yaw[0] == 0)
          ASSERT_EQ(xy_yaw[0], 0);
        else
          ASSERT_GE(p[0] * xy_yaw[0], 0);
        if (xy_yaw[1] == 0)
          ASSERT_EQ(xy_yaw[1], 0);
        else
          ASSERT_GE(p[1] * xy_yaw[1], 0);
      }
      ASSERT_EQ(c->second.getDistance(), i);
      ASSERT_EQ(static_cast<int>(c->second.getMotion().size()), i);
    }
  }

  // 90 deg rotation
  const int xy_syaw_gyaw_90[][4] =
      {
          {1, 1, 0, 1},
          {-1, 1, 1, 2},
          {-1, -1, 2, 3},
          {1, -1, 3, 0},
          {1, -1, 0, 3},
          {1, 1, 1, 0},
          {-1, 1, 2, 1},
          {-1, -1, 3, 2},
      };
  for (auto& xy_syaw_gyaw : xy_syaw_gyaw_90)
  {
    for (int i = 1; i <= range + 1; ++i)
    {
      const CyclicVecInt<3, 2> goal(
          i * xy_syaw_gyaw[0], i * xy_syaw_gyaw[1], xy_syaw_gyaw[3]);
      const auto c = cache.find(xy_syaw_gyaw[2], goal);
      if (i * std::sqrt(2.0) >= range)
      {
        ASSERT_EQ(c, cache.end(xy_syaw_gyaw[2]));
        continue;
      }
      ASSERT_NE(c, cache.end(xy_syaw_gyaw[2]));

      for (const auto& p : c->second.getMotion())
      {
        // Must be in the same quadrant
        ASSERT_GE(p[0] * xy_syaw_gyaw[0], 0);
        ASSERT_GE(p[1] * xy_syaw_gyaw[1], 0);
      }

      const float arc_length = i * 2 * M_PI / 4;
      EXPECT_NEAR(c->second.getDistance(), arc_length, 0.1);
    }
  }
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
