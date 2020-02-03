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

#include <algorithm>
#include <vector>

#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/motion_primitive_builder.h>

#include <gtest/gtest.h>

namespace planner_cspace
{
namespace planner_3d
{
using Vec = MotionPrimitiveBuilder::Vec;

bool compareVecs(const Vec& v1, const Vec& v2)
{
  if (v1[0] != v2[0])
    return v1[0] < v2[0];
  if (v1[1] != v2[1])
    return v1[1] < v2[1];
  return v1[2] < v2[2];
}

std::vector<Vec> buildExpectedPrimitives(const std::vector<std::vector<Vec>> original_primitives_vector,
                                         const int quadrant)
{
  std::vector<Vec> results;
  for (const auto& original_primitives : original_primitives_vector)
  {
    for (const auto& original_primitive : original_primitives)
    {
      Vec primitive;
      switch (quadrant)
      {
        case 0:
          primitive = original_primitive;
          break;
        case 1:
          primitive = Vec(original_primitive[1], -original_primitive[0], original_primitive[2]);
          primitive.cycleUnsigned(16);
          break;
        case 2:
          primitive = Vec(-original_primitive[0], -original_primitive[1], original_primitive[2]);
          primitive.cycleUnsigned(16);
          break;
        case 3:
          primitive = Vec(-original_primitive[1], original_primitive[0], original_primitive[2]);
          primitive.cycleUnsigned(16);
          break;
        default:
          break;
      }
      results.push_back(primitive);
      const Vec opposite = Vec(-primitive[0], -primitive[1], primitive[2]);
      results.push_back(opposite);
    }
  }
  // In-place turns
  for (int i = 1; i < 16; ++i)
  {
    results.push_back(Vec(0, 0, i));
  }
  std::sort(results.begin(), results.end(), &compareVecs);
  return results;
}

TEST(MotionPrimitiveBuilder, Generate)
{
  // clang-format off
  const std::vector<Vec> expected_0deg_straight_primitives =
  {
      Vec(1, 0, 0),
      Vec(2, 0, 0),
      Vec(3, 0, 0),
      Vec(4, 0, 0),
  };
  const std::vector<Vec> expected_0deg_rotation_1_primitives =
  {
      Vec(3, -2, 15),
      Vec(3, -1, 15),
      Vec(3,  1,  1),
      Vec(3,  2,  1),
  };
  const std::vector<Vec> expected_0deg_rotation_2_primitives =
  {
      Vec(3, -2, 14),
      Vec(3, -1, 14),
      Vec(3,  1,  2),
      Vec(3,  2,  2),
  };
  const std::vector<Vec> expected_0deg_rotation_3_primitives =
  {
      Vec(3, -2, 13),
      Vec(3, -1, 13),
      Vec(3,  1,  3),
      Vec(3,  2,  3),
  };
  const std::vector<std::vector<Vec>> expected_0deg_primitives =
  {
    expected_0deg_straight_primitives,
    expected_0deg_rotation_1_primitives,
    expected_0deg_rotation_2_primitives,
    expected_0deg_rotation_3_primitives,
  };

  const std::vector<Vec> expected_23deg_straight_primitives =
  {
      Vec(2, 1, 0),
      Vec(3, 1, 0),
      Vec(3, 2, 0),
  };
  const std::vector<Vec> expected_23deg_rotation_1_primitives =
  {
      Vec(2, 3,  1),
      Vec(3, 0, 15),
      Vec(3, 2,  1),
      Vec(4, 0, 15),
  };
  const std::vector<Vec> expected_23deg_rotation_2_primitives =
  {
      Vec(2,  3,  2),
      Vec(3,  2,  2),
      Vec(3, -1, 14),
      Vec(3,  0, 14),
      Vec(4,  0, 14),
  };
  const std::vector<Vec> expected_23deg_rotation_3_primitives =
  {
      Vec(1,  3,  3),
      Vec(2,  3,  3),
      Vec(3, -1, 13),
      Vec(3,  0, 13),
      Vec(4,  0, 13),
  };
  const std::vector<std::vector<Vec>> expected_23deg_primitives =
  {
    expected_23deg_straight_primitives,
    expected_23deg_rotation_1_primitives,
    expected_23deg_rotation_2_primitives,
    expected_23deg_rotation_3_primitives,
  };

  const std::vector<Vec> expected_45deg_straight_primitives =
  {
      Vec(1, 1, 0),
      Vec(2, 2, 0),
      Vec(3, 2, 0),
      Vec(2, 3, 0),
  };
  const std::vector<Vec> expected_45deg_rotation_1_primitives =
  {
      Vec(1, 3,  1),
      Vec(2, 3,  1),
      Vec(3, 1, 15),
      Vec(3, 2, 15),
  };
  const std::vector<Vec> expected_45deg_rotation_2_primitives =
  {
      Vec(0, 3,  2),
      Vec(1, 3,  2),
      Vec(2, 3,  2),
      Vec(3, 0, 14),
      Vec(3, 1, 14),
      Vec(3, 2, 14),
  };
  const std::vector<Vec> expected_45deg_rotation_3_primitives =
  {
      Vec(0, 3,  3),
      Vec(0, 4,  3),
      Vec(1, 3,  3),
      Vec(3, 0, 13),
      Vec(3, 1, 13),
      Vec(4, 0, 13),
  };
  const std::vector<std::vector<Vec>> expected_45deg_primitives =
  {
    expected_45deg_straight_primitives,
    expected_45deg_rotation_1_primitives,
    expected_45deg_rotation_2_primitives,
    expected_45deg_rotation_3_primitives,
  };

  const std::vector<Vec> expected_67deg_straight_primitives =
  {
      Vec(1, 2, 0),
      Vec(1, 3, 0),
      Vec(2, 3, 0),
  };
  const std::vector<Vec> expected_67deg_rotation_1_primitives =
  {
      Vec(0, 3,  1),
      Vec(0, 4,  1),
      Vec(2, 3, 15),
      Vec(3, 2, 15),
  };
  const std::vector<Vec> expected_67deg_rotation_2_primitives =
  {
      Vec(0, 3,  2),
      Vec(0, 4,  2),
      Vec(-1, 3,  2),
      Vec(2, 3, 14),
      Vec(3, 2, 14),
  };
  const std::vector<Vec> expected_67deg_rotation_3_primitives =
  {
      Vec(0,  3,  3),
      Vec(0,  4,  3),
      Vec(1, -3,  3),
      Vec(3,  1, 13),
      Vec(3,  2, 13),
  };
  const std::vector<std::vector<Vec>> expected_67deg_primitives =
  {
    expected_67deg_straight_primitives,
    expected_67deg_rotation_1_primitives,
    expected_67deg_rotation_2_primitives,
    expected_67deg_rotation_3_primitives,
  };

  const std::vector<std::vector<std::vector<Vec>>> expected_original_primitives =
  {
    expected_0deg_primitives,
    expected_23deg_primitives,
    expected_45deg_primitives,
    expected_67deg_primitives,
  };
  // clang-format on

  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.linear_resolution = 0.1f;
  map_info.angular_resolution = static_cast<float>(M_PI / 8);
  map_info.angle = 16;

  CostCoeff cc;
  cc.min_curve_radius_ = 0.1f;
  cc.angle_resolution_aspect_ = 2.0f / tanf(map_info.angular_resolution);

  const int range = 4;
  const std::vector<std::vector<MotionPrimitiveBuilder::Vec>> motion_primitives =
      MotionPrimitiveBuilder::build(map_info, cc, range);

  EXPECT_EQ(map_info.angle, static_cast<unsigned int>(motion_primitives.size()));
  for (size_t i = 0; i < motion_primitives.size(); ++i)
  {
    std::vector<Vec> current_primitives = motion_primitives[i];
    std::sort(current_primitives.begin(), current_primitives.end(), &compareVecs);
    const auto expected_primitives = buildExpectedPrimitives(expected_original_primitives[i % 4], i / 4);
    ASSERT_EQ(expected_primitives.size(), current_primitives.size());
    for (size_t j = 0; j < current_primitives.size(); ++j)
    {
      const Vec& expected_prim = expected_primitives[j];
      const Vec& actual_prim = current_primitives[j];

      EXPECT_EQ(expected_prim, actual_prim)
          << "Error at " << i << "," << j << "\n"
          << " Expected: (" << expected_prim[0] << "," << expected_prim[1] << "," << expected_prim[2] << ")\n"
          << " Actual: (" << actual_prim[0] << "," << actual_prim[1] << "," << actual_prim[2] << ")\n";
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
