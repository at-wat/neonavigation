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
#include <planner_cspace/planner_3d/grid_astar_model.h>
#include <planner_cspace/planner_3d/motion_primitive_builder.h>
#include <planner_cspace/planner_3d/motion_cache.h>

#include <costmap_cspace_msgs/MapMetaData3D.h>

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
      gm.getAddressor(), 0.5, 0.1);

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

TEST(MotionCache, BezierAngleTravelNonZeroForLaneChange)
{
  const int range = 6;
  const int angle = 16;
  const float angular_resolution = static_cast<float>(M_PI * 2 / angle);
  const float linear_resolution = 0.5f;

  BlockMemGridmap<char, 3, 2, 0x20> gm;
  MotionCache cache;
  cache.reset(
      linear_resolution, angular_resolution, range,
      gm.getAddressor(), 0.5f, 0.1f,
      MotionPrimitiveType::BEZIER, 0.5f, 0);

  // Net yaw difference is 0, but a lateral offset typically requires turning.
  const int start_yaw = 0;
  const CyclicVecInt<3, 2> goal(4, 2, start_yaw);
  const auto it = cache.find(start_yaw, goal);
  ASSERT_NE(it, cache.end(start_yaw));
  EXPECT_GT(it->second.getAngleTravel(), 0.0f);
}

TEST(MotionCache, BezierMotionPrimitiveMappingMatchesBuilder)
{
  const int range = 6;
  const float linear_resolution = 0.1f;
  const int angle = 16;
  const float angular_resolution = static_cast<float>(M_PI * 2 / angle);

  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.linear_resolution = linear_resolution;
  map_info.angular_resolution = angular_resolution;
  map_info.angle = angle;

  CostCoeff cc;
  cc.min_curve_radius_ = 0.4f;
  cc.motion_primitive_type_ = MotionPrimitiveType::BEZIER;
  cc.bezier_cp_dist_ = 0.4f;
  cc.bezier_cp_mode_ = CostCoeff::BezierCpMode::OPTIMIZE;
  cc.angle_resolution_aspect_ = 2.0f / std::tan(map_info.angular_resolution);

  const auto primitives = MotionPrimitiveBuilder::build(map_info, cc, range);

  BlockMemGridmap<char, 3, 2, 0x20> gm;
  MotionCache cache;
  cache.reset(
      linear_resolution, angular_resolution, range,
      gm.getAddressor(), 0.5f, 0.1f,
      MotionPrimitiveType::BEZIER, cc.bezier_cp_dist_, 1);

  for (int syaw = 0; syaw < angle; ++syaw)
  {
    for (const auto& p : primitives[syaw])
    {
      // Rotation primitives are handled separately (in-place turn) and are not stored in MotionCache.
      if (p[0] == 0 && p[1] == 0)
        continue;

      const int goal_yaw = (syaw + p[2] + angle) % angle;
      const CyclicVecInt<3, 2> goal(p[0], p[1], goal_yaw);
      const auto it = cache.find(syaw, goal);
      ASSERT_NE(it, cache.end(syaw))
          << "Missing cache entry for syaw=" << syaw
          << " goal=(" << goal[0] << "," << goal[1] << "," << goal[2] << ")";
    }
  }
}

TEST(MotionCache, BezierHasStraightPrimitivePerStartYaw)
{
  // Regression test for angular_resolution=22.5deg (angle=16):
  // ensure each start yaw has at least one forward-straight and one backward-straight
  // primitive that does not wiggle in yaw.
  const int range = 6;
  const int angle = 16;
  const float angular_resolution = static_cast<float>(M_PI * 2 / angle);
  const float linear_resolution = 0.1f;

  BlockMemGridmap<char, 3, 2, 0x20> gm;
  MotionCache cache;
  cache.reset(
      linear_resolution, angular_resolution, range,
      gm.getAddressor(), 0.2f, 0.05f,
      MotionPrimitiveType::BEZIER, 0.4f, 1);

  auto normalizeAngle = [](float a)
  {
    while (a > static_cast<float>(M_PI))
      a -= static_cast<float>(2.0 * M_PI);
    while (a < static_cast<float>(-M_PI))
      a += static_cast<float>(2.0 * M_PI);
    return a;
  };

  for (int syaw = 0; syaw < angle; ++syaw)
  {
    bool found_fwd = false;
    bool found_bwd = false;
    for (int dx = -range; dx <= range && !(found_fwd && found_bwd); ++dx)
    {
      for (int dy = -range; dy <= range && !(found_fwd && found_bwd); ++dy)
      {
        if (dx == 0 && dy == 0)
          continue;
        if (dx * dx + dy * dy > range * range)
          continue;

        const CyclicVecInt<3, 2> goal(dx, dy, syaw);
        const auto it = cache.find(syaw, goal);
        if (it == cache.end(syaw))
          continue;

        const auto& interp = it->second.getInterpolatedMotion();
        if (interp.empty())
          continue;

        const float yaw_ref = syaw * angular_resolution;
        bool yaw_constant = true;
        for (const auto& p : interp)
        {
          const float yaw = p[2] * angular_resolution;
          if (std::abs(normalizeAngle(yaw - yaw_ref)) > 1.0e-4f)
          {
            yaw_constant = false;
            break;
          }
        }

        if (yaw_constant)
        {
          const float x = dx * linear_resolution;
          const float y = dy * linear_resolution;
          const float dot = std::cos(yaw_ref) * x + std::sin(yaw_ref) * y;
          if (dot > 1.0e-6f)
            found_fwd = true;
          else if (dot < -1.0e-6f)
            found_bwd = true;
        }
      }
    }
    EXPECT_TRUE(found_fwd) << "No forward-straight primitive for start yaw index=" << syaw;
    EXPECT_TRUE(found_bwd) << "No backward-straight primitive for start yaw index=" << syaw;
  }
}
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
