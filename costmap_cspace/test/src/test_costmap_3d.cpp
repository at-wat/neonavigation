/*
 * Copyright (c) 2016-2018, the neonavigation authors
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

#include <cstddef>
#include <string>
#include <algorithm>

#include <gtest/gtest.h>

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_3d.h>

const std::string footprint_str(
    "<value><array><data>"
    "  <value><array><data>"
    "    <value><double>1.5</double></value>"
    "    <value><double>0.0</double></value>"
    "  </data></array></value>"
    "  <value><array><data>"
    "    <value><double>-0.5</double></value>"
    "    <value><double>-0.5</double></value>"
    "  </data></array></value>"
    "  <value><array><data>"
    "    <value><double>-0.5</double></value>"
    "    <value><double>0.5</double></value>"
    "  </data></array></value>"
    "</data></array></value>");
// This footprint with resolution of 0.1 means:
//  0 0 0
//  1 1 0  <--x
//  0 0 0
const char temp_dir[4][2] =
    {
      // x, y which must be occupied in the template
      -1, 0,
      0, -1,
      1, 0,
      0, 1
    };

TEST(Costmap3dLayerFootprintTest, testCSpaceTemplate)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Settings: 4 angular grids, no expand/spread, unknown cost is 0
  cm.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::MAX);

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint;
  ASSERT_TRUE(footprint.fromXml(footprint_str, &footprint_offset));
  ASSERT_TRUE(cm.setFootprint(footprint));

  // Check local footprint
  const costmap_cspace::Polygon polygon = cm.getFootprint();
  ASSERT_EQ(polygon.v.size(), 3 + 1);
  ASSERT_EQ(polygon.v[0][0], 1.5);
  ASSERT_EQ(polygon.v[0][1], 0.0);
  ASSERT_EQ(polygon.v[1][0], -0.5);
  ASSERT_EQ(polygon.v[1][1], -0.5);
  ASSERT_EQ(polygon.v[2][0], -0.5);
  ASSERT_EQ(polygon.v[2][1], 0.5);
  // Last point is same as the first point
  ASSERT_EQ(polygon.v[3][0], 1.5);
  ASSERT_EQ(polygon.v[3][1], 0.0);
  ASSERT_EQ(cm.getFootprintRadius(), 1.5);

  // Generate CSpace pattern around the robot
  costmap_cspace::MapMetaData3D map_info;
  map_info.width = 3;
  map_info.height = 3;
  map_info.angle = 4;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI / 2.0;
  map_info.origin.orientation.w = 1.0;

  cm.generateCSpaceTemplate(map_info);

  ASSERT_EQ(cm.getRangeMax(), static_cast<int>(ceilf(1.5 / 1.0)));

  const costmap_cspace::CSpace3Cache &temp = cm.getTemplate();
  // Check template size
  int x, y, a;
  int cx, cy, ca;
  temp.getSize(x, y, a);
  temp.getCenter(cx, cy, ca);
  ASSERT_EQ(x, 2 * 2 + 1);
  ASSERT_EQ(y, 2 * 2 + 1);
  ASSERT_EQ(a, 4);
  ASSERT_EQ(cx, 2);
  ASSERT_EQ(cy, 2);
  ASSERT_EQ(ca, 0);

  // Check generated template
  for (int k = -ca; k < a - ca; ++k)
  {
    for (int j = -cy; j < y - cy; ++j)
    {
      for (int i = -cx; i < x - cx; ++i)
      {
        if (i == 0 && j == 0)
        {
          ASSERT_EQ(temp.e(i, j, k), 100);
        }
        else if (i == temp_dir[k + ca][0] && j == temp_dir[k + ca][1])
        {
          ASSERT_EQ(temp.e(i, j, k), 100);
        }
        else
        {
          ASSERT_EQ(temp.e(i, j, k), 0);
        }
        // std::cout << std::setfill(' ') << std::setw(3) << static_cast<int>(temp.e(i, j, k)) << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }
}

TEST(Costmap3dLayerPlainTest, testCSpaceTemplate)
{
  costmap_cspace::Costmap3dLayerPlain cm;

  // Settings: 4 angular grids, no expand/spread, unknown cost is 0
  cm.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::MAX);

  // Generate CSpace pattern around the robot
  costmap_cspace::MapMetaData3D map_info;
  map_info.width = 3;
  map_info.height = 3;
  map_info.angle = 4;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI / 2.0;
  map_info.origin.orientation.w = 1.0;

  cm.generateCSpaceTemplate(map_info);

  ASSERT_EQ(cm.getRangeMax(), 1);

  const costmap_cspace::CSpace3Cache &temp = cm.getTemplate();
  // Check template size
  int x, y, a;
  int cx, cy, ca;
  temp.getSize(x, y, a);
  temp.getCenter(cx, cy, ca);
  ASSERT_EQ(x, 3);
  ASSERT_EQ(y, 3);
  ASSERT_EQ(a, 4);
  ASSERT_EQ(cx, 1);
  ASSERT_EQ(cy, 1);
  ASSERT_EQ(ca, 0);

  // Check generated template
  for (int k = -ca; k < a - ca; ++k)
  {
    for (int j = -cy; j < y - cy; ++j)
    {
      for (int i = -cx; i < x - cx; ++i)
      {
        if (i == 0 && j == 0)
        {
          ASSERT_EQ(temp.e(i, j, k), 100);
        }
        else
        {
          ASSERT_EQ(temp.e(i, j, k), 0);
        }
      }
    }
  }
}

TEST(Costmap3dLayerFootprintTest, testCSpaceGenerate)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint;
  footprint.fromXml(footprint_str, &footprint_offset);
  cm.setFootprint(footprint);

  // Settings: 4 angular grids, no expand/spread, unknown cost is 0
  cm.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::MAX);

  // Generate sample map
  nav_msgs::OccupancyGrid map;
  map.info.width = 7;
  map.info.height = 7;
  map.info.resolution = 1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);

  // Apply empty map
  cm.setBaseMap(map);
  cm.processMap();

  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = 0; j < map.info.height; ++j)
    {
      for (size_t i = 0; i < map.info.width; ++i)
      {
        const int cost = cm.getCost(i, j, k);
        // All grid must be 0
        ASSERT_EQ(cost, 0);
        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }

  // std::cout << "========" << std::endl;
  for (auto &g : map.data)
  {
    g = 100;
  }
  cm.setBaseMap(map);
  cm.processMap();
  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = 0; j < map.info.height; ++j)
    {
      for (size_t i = 0; i < map.info.width; ++i)
      {
        const int cost = cm.getCost(i, j, k);
        // All grid must be 100
        ASSERT_EQ(cost, 100);
        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }

  // C shape wall in the map
  for (auto &g : map.data)
  {
    g = 0;
  }
  for (size_t i = map.info.width / 2 - 2; i < map.info.width / 2 + 2; ++i)
  {
    map.data[i + (map.info.height / 2 - 2) * map.info.width] = 100;
    map.data[i + (map.info.height / 2 + 2) * map.info.width] = 100;
  }
  for (size_t i = map.info.height / 2 - 2; i < map.info.height / 2 + 2; ++i)
  {
    map.data[(map.info.width / 2 - 2) + i * map.info.width] = 100;
  }
  cm.setBaseMap(map);
  cm.processMap();
  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = 0; j < map.info.height; ++j)
    {
      for (size_t i = 0; i < map.info.width; ++i)
      {
        const int cost = cm.getCost(i, j, k);

        // Offset according to the template shape
        int cost_offset = 0;
        const int i_offset = static_cast<int>(i) - temp_dir[k][0];
        const int j_offset = static_cast<int>(j) - temp_dir[k][1];
        if (static_cast<size_t>(i_offset) < map.info.width &&
            static_cast<size_t>(j_offset) < map.info.height)
        {
          cost_offset = map.data[i_offset + j_offset * map.info.width];
        }
        if (map.data[i + j * map.info.width] == 100 || cost_offset == 100)
        {
          ASSERT_EQ(cost, 100);
        }
        else
        {
          ASSERT_EQ(cost, 0);
        }
        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }
}

TEST(Costmap3dLayerFootprintTest, testCSpaceExpandSpread)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint;
  footprint.fromXml(footprint_str, &footprint_offset);
  cm.setFootprint(footprint);

  // Settings: 4 angular grids, expand 1.0, spread 2.0, unknown cost is 0
  const float expand = 1.0;
  const float spread = 2.0;
  cm.setCSpaceConfig(4, expand, spread, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::MAX);

  // Generate sample map
  nav_msgs::OccupancyGrid map;
  map.info.width = 9;
  map.info.height = 9;
  map.info.resolution = 1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);

  const int max_cost = 80;
  map.data[map.info.width / 2 + (map.info.height / 2) * map.info.width] = max_cost;
  cm.setBaseMap(map);
  cm.processMap();

  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    const int i_center = map.info.width / 2;
    const int j_center = map.info.height / 2;
    const int i_center2 = map.info.width / 2 + temp_dir[k][0];
    const int j_center2 = map.info.height / 2 + temp_dir[k][1];

    for (size_t j = 0; j < map.info.height; ++j)
    {
      for (size_t i = 0; i < map.info.width; ++i)
      {
        const int cost = cm.getCost(i, j, k);
        const float dist1 = hypotf(static_cast<int>(i) - i_center, static_cast<int>(j) - j_center);
        const float dist2 = hypotf(static_cast<int>(i) - i_center2, static_cast<int>(j) - j_center2);
        const float dist = std::min(dist1, dist2);
        if (dist <= expand)
        {
          // Inside expand range must be max_cost
          EXPECT_EQ(cost, max_cost);
        }
        else if (dist <= expand + spread)
        {
          // Between expand and spread must be intermidiate value
          EXPECT_NE(cost, 0);
          EXPECT_NE(cost, 100);
        }
        else if (dist > expand + spread + 1)
        {
          // Outside must be zero
          // Since the template is calculated by the precised footprint not by the grid,
          // tolerance of test (+1) is needed.
          EXPECT_EQ(cost, 0);
        }
        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }
}

TEST(Costmap3dLayerFootprintTest, testCSpaceOverwrite)
{
  costmap_cspace::Costmap3dLayerFootprint cm;
  costmap_cspace::Costmap3dLayerFootprint cm_ref;
  costmap_cspace::Costmap3dLayerFootprint cm_base;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint;
  footprint.fromXml(footprint_str, &footprint_offset);
  cm.setFootprint(footprint);
  cm_ref.setFootprint(footprint);
  cm_base.setFootprint(footprint);

  // Settings: 4 angular grids, no expand/spread, unknown cost is 0
  cm.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::OVERWRITE);
  cm_ref.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::OVERWRITE);
  cm_base.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::OVERWRITE);

  // Generate two sample maps
  nav_msgs::OccupancyGrid map;
  map.info.width = 9;
  map.info.height = 9;
  map.info.resolution = 1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);

  nav_msgs::OccupancyGrid map2 = map;

  const int num_points_base_map = 2;
  const int points_base_map[num_points_base_map][2] =
      {
        2, 3,
        4, 4
      };
  for (int i = 0; i < num_points_base_map; ++i)
  {
    map.data[points_base_map[i][0] + points_base_map[i][1] * map.info.width] = 100;
  }

  const int num_points_local_map = 3;
  const int points_local_map[num_points_local_map][2] =
      {
        3, 4,
        5, 3,
        4, 4
      };
  for (int i = 0; i < num_points_local_map; ++i)
  {
    map2.data[points_local_map[i][0] + points_local_map[i][1] * map.info.width] = 100;
  }

  // Apply base map
  cm.setBaseMap(map);
  cm.processMap();

  // Overlay local map
  costmap_cspace::CSpace3DUpdate updated = cm.processMapOverlay(map2);

  // In this case, updated map must have same size as the base map. Check it.
  ASSERT_EQ(updated.x, 0);
  ASSERT_EQ(updated.y, 0);
  ASSERT_EQ(updated.yaw, 0);
  ASSERT_EQ(updated.width, map.info.width);
  ASSERT_EQ(updated.height, map.info.height);
  ASSERT_EQ(updated.angle, cm.getAngularGrid());

  // Generate reference local and base cspace map
  cm_ref.setBaseMap(map2);
  cm_ref.processMap();
  cm_base.setBaseMap(map);
  cm_base.processMap();

  // Compare to confirm MAX mode
  // note: boundary of the local map is not completely overwritten for keeping the spread effect.
  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = cm.getRangeMax(); j < map.info.height - cm.getRangeMax(); ++j)
    {
      for (size_t i = cm.getRangeMax(); i < map.info.width - cm.getRangeMax(); ++i)
      {
        const size_t addr = ((k * map.info.height + j) * map.info.width) + i;
        assert(addr < updated.data.size());
        const int cost = updated.data[addr];
        const int cost_ref = cm_ref.getCost(i, j, k);

        ASSERT_EQ(cost, cost_ref);

        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      /*std::cout << "  |  ";
      for (int i = cm.getRangeMax(); i < map.info.width - cm.getRangeMax(); ++i)
      {
        const int cost_ref = cm_ref.getCost(i, j, k);
        std::cout << std::setfill(' ') << std::setw(3) << cost_ref << " ";
      }
      std::cout << std::endl;
      */
    }
    // std::cout << "----" << std::endl;
  }

  // Set MAX mode and check
  cm.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::MAX);
  cm.setBaseMap(map);
  cm.processMap();
  costmap_cspace::CSpace3DUpdate updated_max = cm.processMapOverlay(map2);

  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (int j = cm.getRangeMax(); j < static_cast<int>(map.info.height) - cm.getRangeMax(); ++j)
    {
      for (int i = cm.getRangeMax(); i < static_cast<int>(map.info.width) - cm.getRangeMax(); ++i)
      {
        const size_t addr = ((k * map.info.height + j) * map.info.width) + i;
        assert(addr < updated_max.data.size());
        const int cost = updated_max.data[addr];
        const int cost_ref = cm_ref.getCost(i, j, k);
        const int cost_base = cm_base.getCost(i, j, k);
        const int cost_max = std::max(cost_ref, cost_base);

        ASSERT_EQ(cost, cost_max);

        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      /*
      std::cout << "  |  ";
      for (int i = cm.getRangeMax(); i < map.info.width - cm.getRangeMax(); ++i)
      {
        const int cost_ref = cm_ref.getCost(i, j, k);
        std::cout << std::setfill(' ') << std::setw(3) << cost_ref << " ";
      }
      std::cout << "  |  ";
      for (int i = cm.getRangeMax(); i < map.info.width - cm.getRangeMax(); ++i)
      {
        const int cost_base = cm_base.getCost(i, j, k);
        std::cout << std::setfill(' ') << std::setw(3) << cost_base << " ";
      }
      std::cout << std::endl;
      */
    }
    // std::cout << "----" << std::endl;
  }
}

TEST(Costmap3dLayerFootprintTest, testCSpaceOverlayMove)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint;
  footprint.fromXml(footprint_str, &footprint_offset);
  cm.setFootprint(footprint);

  // Settings: 4 angular grids, no expand/spread, unknown cost is 0
  cm.setCSpaceConfig(4, 0.0, 0.0, 0, costmap_cspace::Costmap3dLayerFootprint::map_overlay_mode::MAX);

  // Generate sample map
  nav_msgs::OccupancyGrid map;
  map.info.width = 5;
  map.info.height = 5;
  map.info.resolution = 1.0;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(map.info.width * map.info.height);

  const int max_cost = 100;
  map.data[map.info.width / 2 + (map.info.height / 2) * map.info.width] = max_cost;
  cm.setBaseMap(map);
  cm.processMap();

  // Generate local sample map
  nav_msgs::OccupancyGrid map2 = map;

  for (int xp = -1; xp <= 1; ++xp)
  {
    for (int yp = -1; yp <= 1; ++yp)
    {
      map2.info.origin.position.x = map2.info.resolution * xp;
      map2.info.origin.position.y = map2.info.resolution * yp;
      /*
      std::cout << "=== origin: ("
                << map2.info.origin.position.x << ", " << map2.info.origin.position.y
                << ")" << std::endl;
      */
      cm.processMapOverlay(map2);
      for (int k = 0; k < cm.getAngularGrid(); ++k)
      {
        const size_t i_center = map.info.width / 2;
        const size_t j_center = map.info.height / 2;
        const size_t i_center2 = map.info.width / 2 + temp_dir[k][0];
        const size_t j_center2 = map.info.height / 2 + temp_dir[k][1];

        for (size_t j = 0; j < map.info.height; ++j)
        {
          for (size_t i = 0; i < map.info.width; ++i)
          {
            const int cost = cm.getCost(i, j, k);

            if ((i == i_center && j == j_center) ||
                (i == i_center2 && j == j_center2) ||
                (i == i_center + xp && j == j_center + yp) ||
                (i == i_center2 + xp && j == j_center2 + yp))
            {
              ASSERT_EQ(cost, max_cost);
            }
            else
            {
              ASSERT_EQ(cost, 0);
            }
            // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
          }
          // std::cout << std::endl;
        }
        // std::cout << "----" << std::endl;
      }
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
