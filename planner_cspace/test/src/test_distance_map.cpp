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

#include <limits>
#include <string>

#include <gtest/gtest.h>

#include <omp.h>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/distance_map.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>

namespace planner_cspace
{
namespace planner_3d
{
TEST(DistanceMap, Generate)
{
  const int w = 10;
  const int h = 10;
  const int angle = 10;

  using Astar = GridAstar<3, 2>;
  Astar::Gridmap<char, 0x40> cm;
  Astar::Gridmap<char, 0x80> cm_hyst;
  Astar::Gridmap<char, 0x80> cm_rough;
  CostmapBBF bbf_costmap;

  DistanceMap dm(cm_rough, bbf_costmap);
  const CostCoeff cc =
      {
          .weight_costmap_ = 1.0f,
      };
  dm.setParams(cc, 2);

  const Astar::Vecf ec(0.5f, 0.5f, 0.2f);
  const int range = 4;
  const int local_range = 10;

  const Astar::Vec size3d(w, h, angle);
  const Astar::Vec size2d(w, h, 1);
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.width = w;
  map_info.height = h;
  map_info.angle = angle;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI * 2 / angle;

  GridAstarModel3D::Ptr model(
      new GridAstarModel3D(
          map_info,
          ec,
          local_range,
          dm.gridmap(), cm, cm_hyst, cm_rough,
          cc, range));
  cm.reset(size3d);
  cm_hyst.reset(size3d);
  cm_rough.reset(size2d);
  bbf_costmap.reset(size2d);

  const DistanceMap::Params dmp =
      {
          .euclid_cost = ec,
          .range = range,
          .local_range = local_range,
          .longcut_range = 10,
          .size = size2d,
          .resolution = map_info.linear_resolution,
      };
  dm.init(model, dmp);
  omp_set_num_threads(2);

  for (int x = 0; x < w; x++)
  {
    cm_rough[Astar::Vec(x, 5, 0)] = 100;
  }
  cm_rough[Astar::Vec(3, 5, 0)] = 0;

  const Astar::Vec s(2, 2, 0);
  const Astar::Vec e(8, 8, 0);

  const float tolerance = 0.4;
  const auto validateDistance =
      [tolerance](
          const Astar::Vec p,
          const float d,
          const std::string& msg)
  {
    if (p[1] < 5)
    {
      ASSERT_NEAR(
          2.7 + 0.5 * (p - Astar::Vec(3, 5, 0)).norm(),
          d, tolerance)
          << msg;
    }
    else if (p[1] > 5)
    {
      ASSERT_NEAR(
          0.5 * (p - Astar::Vec(8, 8, 0)).norm(),
          d, tolerance)
          << msg;
    }
    else if (p == Astar::Vec(3, 5, 0))
    {
      ASSERT_NEAR(2.7, d, tolerance) << msg;
    }
  };
  const auto validate =
      [&validateDistance, &dm](const std::string& msg) -> bool
  {
    for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
      {
        const Astar::Vec pos(x, y, 0);
        validateDistance(
            pos, dm[pos],
            msg + " failed at " +
                "(" + std::to_string(x) + ", " + std::to_string(y) + ")");
        if (::testing::Test::HasFatalFailure())
          return false;
      }
    }
    return true;
  };
  const auto debug_output = [&dm, &cm_rough]()
  {
    for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
      {
        const Astar::Vec pos(x, y, 0);
        const float d = dm[pos];
        if (d == std::numeric_limits<float>::max() ||
            cm_rough[pos] == 100)
        {
          fprintf(stderr, "xxx ");
          continue;
        }
        fprintf(stderr, "%3.1f ", d);
      }
      fprintf(stderr, "\n");
    }
  };

  dm.create(s, e);
  debug_output();

  if (!validate("create"))
  {
    debug_output();
    return;
  }

  // Check all combination of updated area
  for (int update_y0 = 0; update_y0 < w; update_y0++)
  {
    for (int update_x0 = 0; update_x0 < w; update_x0++)
    {
      const std::string from =
          "(" + std::to_string(update_x0) + ", " + std::to_string(update_y0) + ")";
      for (int update_y1 = update_y0; update_y1 < w; update_y1++)
      {
        for (int update_x1 = update_x0; update_x1 < w; update_x1++)
        {
          const std::string to =
              "(" + std::to_string(update_x1) + ", " + std::to_string(update_y1) + ")";
          dm.create(s, e);
          dm.update(s, e, update_x0, update_x1, update_y0, update_y1);
          if (!validate("update " + from + "-" + to))
          {
            debug_output();
            return;
          }
        }
      }
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
