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

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace/costmap_3d.h>

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

TEST(Costmap3dLayerFootprint, CSpaceTemplate)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Settings: 4 angular grids, no expand/spread
  cm.setAngleResolution(4);
  cm.setExpansion(0.0, 0.0);
  cm.setOverlayMode(costmap_cspace::MapOverlayMode::MAX);

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint_xml;
  ASSERT_TRUE(footprint_xml.fromXml(footprint_str, &footprint_offset));
  cm.setFootprint(costmap_cspace::Polygon(footprint_xml));

  // Check local footprint
  const costmap_cspace::Polygon polygon = cm.getFootprint();
  ASSERT_EQ(polygon.v.size(), 3u + 1u);
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
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.width = 3;
  map_info.height = 3;
  map_info.angle = 4;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI / 2.0;
  map_info.origin.orientation.w = 1.0;

  cm.setMapMetaData(map_info);

  ASSERT_EQ(cm.getRangeMax(), static_cast<int>(ceilf(1.5 / 1.0)));

  const costmap_cspace::CSpace3Cache& temp = cm.getTemplate();
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

TEST(Costmap3dLayerPlain, CSpaceTemplate)
{
  costmap_cspace::Costmap3dLayerPlain cm;

  // Settings: 4 angular grids, no expand/spread
  cm.setAngleResolution(4);
  cm.setExpansion(0.0, 0.0);
  cm.setOverlayMode(costmap_cspace::MapOverlayMode::MAX);

  // Generate CSpace pattern around the robot
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.width = 1;
  map_info.height = 1;
  map_info.angle = 4;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI / 2.0;
  map_info.origin.orientation.w = 1.0;

  cm.setMapMetaData(map_info);

  ASSERT_EQ(cm.getRangeMax(), 0);

  const costmap_cspace::CSpace3Cache& temp = cm.getTemplate();
  // Check template size
  int x, y, a;
  int cx, cy, ca;
  temp.getSize(x, y, a);
  temp.getCenter(cx, cy, ca);
  ASSERT_EQ(x, 1);
  ASSERT_EQ(y, 1);
  ASSERT_EQ(a, 4);
  ASSERT_EQ(cx, 0);
  ASSERT_EQ(cy, 0);
  ASSERT_EQ(ca, 0);

  // Check generated template
  for (int k = -ca; k < a - ca; ++k)
  {
    ASSERT_EQ(temp.e(0, 0, k), 100);
  }
}

TEST(Costmap3dLayerFootprint, CSpaceGenerate)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint_xml;
  footprint_xml.fromXml(footprint_str, &footprint_offset);
  cm.setFootprint(costmap_cspace::Polygon(footprint_xml));

  // Settings: 4 angular grids, no expand/spread
  cm.setAngleResolution(4);
  cm.setExpansion(0.0, 0.0);
  cm.setOverlayMode(costmap_cspace::MapOverlayMode::MAX);

  // Generate sample map
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
  map->info.width = 7;
  map->info.height = 7;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data.resize(map->info.width * map->info.height);

  // Apply empty map
  cm.setBaseMap(map);

  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = 0; j < map->info.height; ++j)
    {
      for (size_t i = 0; i < map->info.width; ++i)
      {
        const int cost = cm.getMapOverlay()->getCost(i, j, k);
        // All grid must be 0
        ASSERT_EQ(cost, 0);
        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }

  // std::cout << "========" << std::endl;
  for (auto& g : map->data)
  {
    g = 100;
  }
  cm.setBaseMap(map);
  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = 0; j < map->info.height; ++j)
    {
      for (size_t i = 0; i < map->info.width; ++i)
      {
        const int cost = cm.getMapOverlay()->getCost(i, j, k);
        // All grid must be 100
        ASSERT_EQ(cost, 100);
        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      // std::cout << std::endl;
    }
    // std::cout << "----" << std::endl;
  }

  // C shape wall in the map
  for (auto& g : map->data)
  {
    g = 0;
  }
  for (size_t i = map->info.width / 2 - 2; i < map->info.width / 2 + 2; ++i)
  {
    map->data[i + (map->info.height / 2 - 2) * map->info.width] = 100;
    map->data[i + (map->info.height / 2 + 2) * map->info.width] = 100;
  }
  for (size_t i = map->info.height / 2 - 2; i < map->info.height / 2 + 2; ++i)
  {
    map->data[(map->info.width / 2 - 2) + i * map->info.width] = 100;
  }
  cm.setBaseMap(map);
  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    for (size_t j = 0; j < map->info.height; ++j)
    {
      for (size_t i = 0; i < map->info.width; ++i)
      {
        const int cost = cm.getMapOverlay()->getCost(i, j, k);

        // Offset according to the template shape
        int cost_offset = 0;
        const int i_offset = static_cast<int>(i) - temp_dir[k][0];
        const int j_offset = static_cast<int>(j) - temp_dir[k][1];
        if (static_cast<size_t>(i_offset) < map->info.width &&
            static_cast<size_t>(j_offset) < map->info.height)
        {
          cost_offset = map->data[i_offset + j_offset * map->info.width];
        }
        if (map->data[i + j * map->info.width] == 100 || cost_offset == 100)
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

TEST(Costmap3dLayerFootprint, CSpaceExpandSpread)
{
  costmap_cspace::Costmap3dLayerFootprint cm;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint_xml;
  footprint_xml.fromXml(footprint_str, &footprint_offset);
  cm.setFootprint(costmap_cspace::Polygon(footprint_xml));

  // Settings: 4 angular grids, expand 1.0, spread 2.0
  const float expand = 1.0;
  const float spread = 2.0;
  cm.setAngleResolution(4);
  cm.setExpansion(expand, spread);
  cm.setOverlayMode(costmap_cspace::MapOverlayMode::MAX);

  // Generate sample map
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
  map->info.width = 9;
  map->info.height = 9;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data.resize(map->info.width * map->info.height);

  const int max_cost = 80;
  map->data[map->info.width / 2 + (map->info.height / 2) * map->info.width] = max_cost;
  cm.setBaseMap(map);

  for (int k = 0; k < cm.getAngularGrid(); ++k)
  {
    const int i_center = map->info.width / 2;
    const int j_center = map->info.height / 2;
    const int i_center2 = map->info.width / 2 + temp_dir[k][0];
    const int j_center2 = map->info.height / 2 + temp_dir[k][1];

    for (size_t j = 0; j < map->info.height; ++j)
    {
      for (size_t i = 0; i < map->info.width; ++i)
      {
        const int cost = cm.getMapOverlay()->getCost(i, j, k);
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

TEST(Costmap3dLayerFootprint, CSpaceOverwrite)
{
  costmap_cspace::Costmap3dLayerFootprint cm_ref;
  costmap_cspace::Costmap3dLayerFootprint cm_base;

  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint_xml;
  footprint_xml.fromXml(footprint_str, &footprint_offset);
  costmap_cspace::Polygon footprint(footprint_xml);
  cm_ref.setFootprint(footprint);
  cm_base.setFootprint(footprint);

  // Settings: 4 angular grids, no expand/spread
  costmap_cspace::Costmap3d cms(4);
  auto cm = cms.addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
  cm->setExpansion(0.0, 0.0);
  cm->setFootprint(footprint);
  auto cm_over = cms.addLayer<costmap_cspace::Costmap3dLayerFootprint>(
      costmap_cspace::MapOverlayMode::OVERWRITE);
  cm_over->setExpansion(0.0, 0.0);
  cm_over->setFootprint(footprint);
  auto cm_output = cms.addLayer<costmap_cspace::Costmap3dLayerOutput>();

  cm_ref.setAngleResolution(4);
  cm_ref.setExpansion(0.0, 0.0);
  cm_ref.setOverlayMode(costmap_cspace::MapOverlayMode::OVERWRITE);
  cm_base.setAngleResolution(4);
  cm_base.setExpansion(0.0, 0.0);
  cm_base.setOverlayMode(costmap_cspace::MapOverlayMode::OVERWRITE);

  // Generate two sample maps
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
  map->info.width = 9;
  map->info.height = 9;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data.resize(map->info.width * map->info.height);

  nav_msgs::OccupancyGrid::Ptr map2(new nav_msgs::OccupancyGrid);
  *map2 = *map;

  const int num_points_base_map = 2;
  const int points_base_map[num_points_base_map][2] =
      {
        2, 3,
        4, 4
      };
  for (int i = 0; i < num_points_base_map; ++i)
  {
    map->data[points_base_map[i][0] + points_base_map[i][1] * map->info.width] = 100;
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
    map2->data[points_local_map[i][0] + points_local_map[i][1] * map->info.width] = 100;
  }

  // Apply base map
  cm->setBaseMap(map);

  // Overlay local map
  costmap_cspace_msgs::CSpace3DUpdate::Ptr updated(new costmap_cspace_msgs::CSpace3DUpdate);
  auto cb = [&updated](
      const costmap_cspace::CSpace3DMsg::Ptr map,
      const costmap_cspace_msgs::CSpace3DUpdate::Ptr& update) -> bool
  {
    updated = update;
    return true;
  };
  cm_output->setHandler(cb);
  cm_over->processMapOverlay(map2);

  // In this case, updated map must have same size as the base map. Check it.
  ASSERT_EQ(updated->x, 0u);
  ASSERT_EQ(updated->y, 0u);
  ASSERT_EQ(updated->yaw, 0u);
  ASSERT_EQ(updated->width, map->info.width);
  ASSERT_EQ(updated->height, map->info.height);
  ASSERT_EQ(updated->angle, static_cast<size_t>(cm_over->getAngularGrid()));

  // Generate reference local and base cspace map
  cm_ref.setBaseMap(map2);
  cm_base.setBaseMap(map);

  // Compare to confirm MAX mode
  // note: boundary of the local map is not completely overwritten for keeping the spread effect.
  for (int k = 0; k < cm_over->getAngularGrid(); ++k)
  {
    for (size_t j = cm_over->getRangeMax(); j < map->info.height - cm_over->getRangeMax(); ++j)
    {
      for (size_t i = cm_over->getRangeMax(); i < map->info.width - cm_over->getRangeMax(); ++i)
      {
        const size_t addr = ((k * map->info.height + j) * map->info.width) + i;
        ROS_ASSERT(addr < updated->data.size());
        const int cost = updated->data[addr];
        const int cost_ref = cm_ref.getMapOverlay()->getCost(i, j, k);

        ASSERT_EQ(cost, cost_ref);

        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      /*std::cout << "  |  ";
      for (int i = cm_over->getRangeMax(); i < map->info.width - cm_over->getRangeMax(); ++i)
      {
        const int cost_ref = cm_ref.getMapOverlay()->getCost(i, j, k);
        std::cout << std::setfill(' ') << std::setw(3) << cost_ref << " ";
      }
      std::cout << std::endl;
      */
    }
    // std::cout << "----" << std::endl;
  }
  // Set MAX mode and check
  cm_over->setAngleResolution(4);
  cm_over->setExpansion(0.0, 0.0);
  cm_over->setOverlayMode(costmap_cspace::MapOverlayMode::MAX);
  costmap_cspace_msgs::CSpace3DUpdate::Ptr updated_max(new costmap_cspace_msgs::CSpace3DUpdate);
  auto cb_max = [&updated_max](
      const costmap_cspace::CSpace3DMsg::Ptr map,
      const costmap_cspace_msgs::CSpace3DUpdate::Ptr& update) -> bool
  {
    updated_max = update;
    return true;
  };
  cm_output->setHandler(cb_max);
  cm_over->processMapOverlay(map2);

  ASSERT_EQ(updated_max->x, 0u);
  ASSERT_EQ(updated_max->y, 0u);
  ASSERT_EQ(updated_max->yaw, 0u);
  ASSERT_EQ(updated_max->width, map->info.width);
  ASSERT_EQ(updated_max->height, map->info.height);
  ASSERT_EQ(updated_max->angle, static_cast<size_t>(cm_over->getAngularGrid()));

  for (int k = 0; k < cm_over->getAngularGrid(); ++k)
  {
    for (int j = cm_over->getRangeMax(); j < static_cast<int>(map->info.height) - cm_over->getRangeMax(); ++j)
    {
      for (int i = cm_over->getRangeMax(); i < static_cast<int>(map->info.width) - cm_over->getRangeMax(); ++i)
      {
        const size_t addr = ((k * map->info.height + j) * map->info.width) + i;
        ROS_ASSERT(addr < updated_max->data.size());
        const int cost = updated_max->data[addr];
        const int cost_ref = cm_ref.getMapOverlay()->getCost(i, j, k);
        const int cost_base = cm_base.getMapOverlay()->getCost(i, j, k);
        const int cost_max = std::max(cost_ref, cost_base);

        ASSERT_EQ(cost, cost_max);

        // std::cout << std::setfill(' ') << std::setw(3) << cost << " ";
      }
      /*
      std::cout << "  |  ";
      for (int i = cm_over->getRangeMax(); i < map->info.width - cm_over->getRangeMax(); ++i)
      {
        const int cost_ref = cm_ref.getMapOverlay()->getCost(i, j, k);
        std::cout << std::setfill(' ') << std::setw(3) << cost_ref << " ";
      }
      std::cout << "  |  ";
      for (int i = cm_over->getRangeMax(); i < map->info.width - cm_over->getRangeMax(); ++i)
      {
        const int cost_base = cm_base.getMapOverlay()->getCost(i, j, k);
        std::cout << std::setfill(' ') << std::setw(3) << cost_base << " ";
      }
      std::cout << std::endl;
      */
    }
    // std::cout << "----" << std::endl;
  }
}

TEST(Costmap3dLayerFootprint, CSpaceOverlayMove)
{
  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint_xml;
  footprint_xml.fromXml(footprint_str, &footprint_offset);
  costmap_cspace::Polygon footprint(footprint_xml);

  // Settings: 4 angular grids, no expand/spread
  costmap_cspace::Costmap3d cms(4);
  auto cm = cms.addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
  cm->setExpansion(0.0, 0.0);
  cm->setFootprint(footprint);
  auto cm_over = cms.addLayer<costmap_cspace::Costmap3dLayerFootprint>(
      costmap_cspace::MapOverlayMode::MAX);
  cm_over->setExpansion(0.0, 0.0);
  cm_over->setFootprint(footprint);

  // Generate sample map
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
  map->info.width = 5;
  map->info.height = 5;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data.resize(map->info.width * map->info.height);

  const int max_cost = 100;
  map->data[map->info.width / 2 + (map->info.height / 2) * map->info.width] = max_cost;
  cm->setBaseMap(map);

  // Generate local sample map
  nav_msgs::OccupancyGrid::Ptr map2(new nav_msgs::OccupancyGrid);
  *map2 = *map;

  for (int xp = -1; xp <= 1; ++xp)
  {
    for (int yp = -1; yp <= 1; ++yp)
    {
      map2->info.origin.position.x = map2->info.resolution * xp;
      map2->info.origin.position.y = map2->info.resolution * yp;
      /*
      std::cout << "=== origin: ("
                << map2->info.origin.position.x << ", " << map2->info.origin.position.y
                << ")" << std::endl;
      */
      cm_over->processMapOverlay(map2);
      for (int k = 0; k < cm_over->getAngularGrid(); ++k)
      {
        const size_t i_center = map->info.width / 2;
        const size_t j_center = map->info.height / 2;
        const size_t i_center2 = map->info.width / 2 + temp_dir[k][0];
        const size_t j_center2 = map->info.height / 2 + temp_dir[k][1];

        for (size_t j = 0; j < map->info.height; ++j)
        {
          for (size_t i = 0; i < map->info.width; ++i)
          {
            const int cost = cm_over->getMapOverlay()->getCost(i, j, k);

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
