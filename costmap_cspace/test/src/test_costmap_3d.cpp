/*
 * Copyright (c) 2016-2019, the neonavigation authors
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
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace/costmap_3d.h>
#include <nav_msgs/OccupancyGrid.h>

#include <gtest/gtest.h>

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
        {-1, 0},
        {0, -1},
        {1, 0},
        {0, 1},
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
  ASSERT_EQ(3u + 1u, polygon.v.size());
  ASSERT_EQ(1.5, polygon.v[0][0]);
  ASSERT_EQ(0.0, polygon.v[0][1]);
  ASSERT_EQ(-0.5, polygon.v[1][0]);
  ASSERT_EQ(-0.5, polygon.v[1][1]);
  ASSERT_EQ(-0.5, polygon.v[2][0]);
  ASSERT_EQ(0.5, polygon.v[2][1]);
  // Last point is same as the first point
  ASSERT_EQ(1.5, polygon.v[3][0]);
  ASSERT_EQ(0.0, polygon.v[3][1]);
  ASSERT_EQ(1.5, cm.getFootprintRadius());

  // Generate CSpace pattern around the robot
  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.width = 3;
  map_info.height = 3;
  map_info.angle = 4;
  map_info.linear_resolution = 1.0;
  map_info.angular_resolution = M_PI / 2.0;
  map_info.origin.orientation.w = 1.0;

  cm.setMapMetaData(map_info);

  ASSERT_EQ(static_cast<int>(std::ceil(1.5 / 1.0)), cm.getRangeMax());

  const costmap_cspace::CSpace3Cache& temp = cm.getTemplate();
  // Check template size
  int x, y, a;
  int cx, cy, ca;
  temp.getSize(x, y, a);
  temp.getCenter(cx, cy, ca);
  ASSERT_EQ(2 * 2 + 1, x);
  ASSERT_EQ(2 * 2 + 1, y);
  ASSERT_EQ(4, a);
  ASSERT_EQ(2, cx);
  ASSERT_EQ(2, cy);
  ASSERT_EQ(0, ca);

  // Check generated template
  for (int k = -ca; k < a - ca; ++k)
  {
    for (int j = -cy; j < y - cy; ++j)
    {
      for (int i = -cx; i < x - cx; ++i)
      {
        if (i == 0 && j == 0)
        {
          ASSERT_EQ(100, temp.e(i, j, k));
        }
        else if (i == temp_dir[k + ca][0] && j == temp_dir[k + ca][1])
        {
          ASSERT_EQ(100, temp.e(i, j, k));
        }
        else
        {
          ASSERT_EQ(0, temp.e(i, j, k));
        }
      }
    }
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

  ASSERT_EQ(0, cm.getRangeMax());

  const costmap_cspace::CSpace3Cache& temp = cm.getTemplate();
  // Check template size
  int x, y, a;
  int cx, cy, ca;
  temp.getSize(x, y, a);
  temp.getCenter(cx, cy, ca);
  ASSERT_EQ(1, x);
  ASSERT_EQ(1, y);
  ASSERT_EQ(4, a);
  ASSERT_EQ(0, cx);
  ASSERT_EQ(0, cy);
  ASSERT_EQ(0, ca);

  // Check generated template
  for (int k = -ca; k < a - ca; ++k)
  {
    ASSERT_EQ(100, temp.e(0, 0, k));
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
        // All grid must be unknown at initialization
        ASSERT_EQ(0, cost);
      }
    }
  }

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
        ASSERT_EQ(100, cost);
      }
    }
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
          ASSERT_EQ(100, cost);
        }
        else
        {
          ASSERT_EQ(0, cost);
        }
      }
    }
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
  map->data[map->info.width / 2 + 1 + (map->info.height / 2) * map->info.width] = -1;

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
        const float dist1 = std::hypot(static_cast<int>(i) - i_center, static_cast<int>(j) - j_center);
        const float dist2 = std::hypot(static_cast<int>(i) - i_center2, static_cast<int>(j) - j_center2);
        const float dist = std::min(dist1, dist2);

        if (i == static_cast<size_t>(i_center + 1) &&
            j == static_cast<size_t>(j_center))
        {
          // Unknown cell must be unknown
          EXPECT_EQ(-1, cost);
        }
        else if (dist <= expand)
        {
          // Inside expand range must be max_cost
          EXPECT_EQ(max_cost, cost);
        }
        else if (dist <= expand + spread)
        {
          // Between expand and spread must be intermidiate value
          EXPECT_NE(0, cost);
          EXPECT_NE(100, cost);
        }
        else if (dist > expand + spread + 1)
        {
          // Outside must be zero
          // Since the template is calculated by the precised footprint not by the grid,
          // tolerance of test (+1) is needed.
          EXPECT_EQ(0, cost);
        }
      }
    }
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
          {2, 3},
          {4, 4},
      };
  for (int i = 0; i < num_points_base_map; ++i)
  {
    map->data[points_base_map[i][0] + points_base_map[i][1] * map->info.width] = 100;
  }

  const int num_points_local_map = 3;
  const int points_local_map[num_points_local_map][2] =
      {
          {3, 4},
          {5, 3},
          {4, 4},
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
  ASSERT_EQ(0u, updated->x);
  ASSERT_EQ(0u, updated->y);
  ASSERT_EQ(0u, updated->yaw);
  ASSERT_EQ(map->info.width, updated->width);
  ASSERT_EQ(map->info.height, updated->height);
  ASSERT_EQ(static_cast<size_t>(cm_over->getAngularGrid()), updated->angle);

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

        ASSERT_EQ(cost_ref, cost);
      }
    }
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

  ASSERT_EQ(0u, updated_max->x);
  ASSERT_EQ(0u, updated_max->y);
  ASSERT_EQ(0u, updated_max->yaw);
  ASSERT_EQ(map->info.width, updated_max->width);
  ASSERT_EQ(map->info.height, updated_max->height);
  ASSERT_EQ(static_cast<size_t>(cm_over->getAngularGrid()), updated_max->angle);

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

        ASSERT_EQ(cost_max, cost);
      }
    }
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
              ASSERT_EQ(max_cost, cost);
            }
            else
            {
              ASSERT_EQ(0, cost);
            }
          }
        }
      }
    }
  }
}

TEST(Costmap3dLayerOutput, CSpaceOutOfBoundary)
{
  struct TestData
  {
    struct Input
    {
      float x;
      float y;
    };
    struct Expected
    {
      unsigned int x;
      unsigned int y;
      unsigned int yaw;
      unsigned int width;
      unsigned int height;
      unsigned int angle;
    };
    std::string name;
    Input input;
    bool valid;
    Expected expected;
  };
  const TestData dataset[] =
      {
          {"inside0", {0.0, 0.0}, true, {0u, 0u, 0u, 2u, 2u, 4u}},
          {"inside1", {1.0, 0.0}, true, {1u, 0u, 0u, 2u, 2u, 4u}},
          {"half-outside x0", {-1.0, 0.0}, true, {0u, 0u, 0u, 1u, 2u, 4u}},
          {"half-outside x1", {3.0, 0.0}, true, {3u, 0u, 0u, 1u, 2u, 4u}},
          {"half-outside y0", {0.0, -1.0}, true, {0u, 0u, 0u, 2u, 1u, 4u}},
          {"half-outside y1", {0.0, 3.0}, true, {0u, 3u, 0u, 2u, 1u, 4u}},
          {"half-outside xy0", {-1.0, -1.0}, true, {0u, 0u, 0u, 1u, 1u, 4u}},
          {"half-outside xy1", {3.0, -1.0}, true, {3u, 0u, 0u, 1u, 1u, 4u}},
          {"half-outside xy2", {3.0, 3.0}, true, {3u, 3u, 0u, 1u, 1u, 4u}},
          {"half-outside xy3", {-1.0, 3.0}, true, {0u, 3u, 0u, 1u, 1u, 4u}},
          {"boundary x0", {-2.0, 0.0}, false},
          {"boundary x1", {4, 0.0}, false},
          {"boundary y0", {0, -2.0}, false},
          {"boundary y1", {0, 4.0}, false},
          {"boundary xy0", {-2.0, -2.0}, false},
          {"boundary xy1", {4.0, -2.0}, false},
          {"boundary xy2", {4.0, 4.0}, false},
          {"boundary xy3", {-2.0, 4.0}, false},
          {"outside x0", {-3.0, 0.0}, false},
          {"outside x1", {5, 0.0}, false},
          {"outside y0", {0, -3.0}, false},
          {"outside y1", {0, 5.0}, false},
          {"outside xy0", {-3.0, -3.0}, false},
          {"outside xy1", {5.0, -3.0}, false},
          {"outside xy2", {5.0, 5.0}, false},
          {"outside xy3", {-3.0, 5.0}, false},
      };

  for (auto& d : dataset)
  {
    const std::string test_name = "Case [" + d.name + "]";
    // Settings: 4 angular grids
    costmap_cspace::Costmap3d cms(4);
    auto cm = cms.addRootLayer<costmap_cspace::Costmap3dLayerPlain>();
    auto cm_stop = cms.addLayer<costmap_cspace::Costmap3dLayerStopPropagation>();
    auto cm_over = cms.addLayer<costmap_cspace::Costmap3dLayerPlain>(
        costmap_cspace::MapOverlayMode::OVERWRITE);
    auto cm_output = cms.addLayer<costmap_cspace::Costmap3dLayerOutput>();

    // Generate two sample maps
    nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
    map->info.width = 4;
    map->info.height = 4;
    map->info.resolution = 1.0;
    map->info.origin.orientation.w = 1.0;
    map->data.resize(map->info.width * map->info.height);

    nav_msgs::OccupancyGrid::Ptr map2(new nav_msgs::OccupancyGrid);
    map2->info.width = 2;
    map2->info.height = 2;
    map2->info.resolution = 1.0;
    map2->info.origin.orientation.w = 1.0;
    map2->info.origin.position.x = d.input.x;
    map2->info.origin.position.y = d.input.y;
    map2->data.resize(map->info.width * map->info.height);

    // Apply base map
    cm->setBaseMap(map);

    // Overlay local map
    costmap_cspace_msgs::CSpace3DUpdate::Ptr updated;
    auto cb = [&updated](
                  const costmap_cspace::CSpace3DMsg::Ptr& map,
                  const costmap_cspace_msgs::CSpace3DUpdate::Ptr& update) -> bool
    {
      updated = update;
      return true;
    };
    cm_output->setHandler(cb);
    // First pass of the processing contains parent layer updates
    cm_over->processMapOverlay(map2);

    if (d.valid)
    {
      ASSERT_TRUE(static_cast<bool>(updated)) << test_name;
      EXPECT_EQ(0u, updated->x) << test_name;
      EXPECT_EQ(0u, updated->y) << test_name;
      EXPECT_EQ(0u, updated->yaw) << test_name;
      EXPECT_EQ(map->info.width, updated->width) << test_name;
      EXPECT_EQ(map->info.height, updated->height) << test_name;
      EXPECT_EQ(4u, updated->angle) << test_name;
    }
    else
    {
      EXPECT_FALSE(static_cast<bool>(updated)) << test_name;
    }

    // Second pass has only local updates
    cm_over->processMapOverlay(map2);

    if (d.valid)
    {
      ASSERT_TRUE(static_cast<bool>(updated)) << test_name;
      EXPECT_EQ(d.expected.x, updated->x) << test_name;
      EXPECT_EQ(d.expected.y, updated->y) << test_name;
      EXPECT_EQ(d.expected.yaw, updated->yaw) << test_name;
      EXPECT_EQ(d.expected.width, updated->width) << test_name;
      EXPECT_EQ(d.expected.height, updated->height) << test_name;
      EXPECT_EQ(d.expected.angle, updated->angle) << test_name;
    }
    else
    {
      EXPECT_FALSE(static_cast<bool>(updated)) << test_name;
    }
  }
}

TEST(Costmap3dLayerFootprint, CSpaceKeepUnknown)
{
  // Set example footprint
  int footprint_offset = 0;
  XmlRpc::XmlRpcValue footprint_xml;
  footprint_xml.fromXml(footprint_str, &footprint_offset);
  costmap_cspace::Polygon footprint(footprint_xml);

  const size_t unknown_x = 3;
  const size_t unknown_y = 4;
  const size_t width = 6;
  const size_t height = 5;
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
  map->info.width = width;
  map->info.height = height;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data = std::vector<int8_t>(width * height, 0);
  map->data[2 + width * 3] = 100;
  map->data[3 + width * 3] = -1;

  nav_msgs::OccupancyGrid::Ptr map_overlay(new nav_msgs::OccupancyGrid);
  map_overlay->info.width = width;
  map_overlay->info.height = height;
  map_overlay->info.resolution = 1.0;
  map_overlay->info.origin.orientation.w = 1.0;
  map_overlay->data = std::vector<int8_t>(width * height, 0);
  map_overlay->data[2 + width * 4] = 100;
  map_overlay->data[4 + width * 4] = -1;

  map->data[unknown_x + width * unknown_y] = -1;
  map_overlay->data[unknown_x + width * unknown_y] = -1;

  costmap_cspace::Costmap3d cms1(4);
  auto cm_base1 = cms1.addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
  cm_base1->setExpansion(0.0, 2.0);
  cm_base1->setFootprint(footprint);
  auto cm_normal = cms1.addLayer<costmap_cspace::Costmap3dLayerFootprint>(
      costmap_cspace::MapOverlayMode::MAX);
  cm_normal->setExpansion(0.0, 2.0);
  cm_normal->setFootprint(footprint);
  cm_normal->setKeepUnknown(false);
  cm_base1->setBaseMap(map);
  cm_normal->processMapOverlay(map_overlay);

  costmap_cspace::Costmap3d cms2(4);
  auto cm_base2 = cms2.addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
  cm_base2->setExpansion(0.0, 2.0);
  cm_base2->setFootprint(footprint);
  auto cm_keep_uknown = cms2.addLayer<costmap_cspace::Costmap3dLayerFootprint>(
      costmap_cspace::MapOverlayMode::MAX);
  cm_keep_uknown->setExpansion(0.0, 2.0);
  cm_keep_uknown->setFootprint(footprint);
  cm_keep_uknown->setKeepUnknown(true);
  cm_base2->setBaseMap(map);
  cm_keep_uknown->processMapOverlay(map_overlay);

  const costmap_cspace::CSpace3DMsg::Ptr normal_result = cm_normal->getMapOverlay();
  const costmap_cspace::CSpace3DMsg::Ptr keep_unknown_result = cm_keep_uknown->getMapOverlay();
  for (size_t yaw = 0; yaw < normal_result->info.angle; ++yaw)
  {
    for (size_t y = 0; y < normal_result->info.height; ++y)
    {
      for (size_t x = 0; x < normal_result->info.width; ++x)
      {
        if ((x == unknown_x) && (y == unknown_y))
        {
          EXPECT_GT(normal_result->getCost(x, y, yaw), 0);
          EXPECT_EQ(static_cast<int>(keep_unknown_result->getCost(x, y, yaw)), -1);
        }
        else
        {
          EXPECT_EQ(normal_result->getCost(x, y, yaw), keep_unknown_result->getCost(x, y, yaw))
              << " x:" << x << " y:" << y << " yaw:" << yaw;
        }
      }
    }
  }
}

TEST(Costmap3dLayerFootprint, Costmap3dLayerPlain)
{
  costmap_cspace::Polygon footprint;
  footprint.v.resize(3);
  for (auto& p : footprint.v)
  {
    p[0] = p[1] = 0.0;
  }

  const size_t unknown_x = 3;
  const size_t unknown_y = 4;
  const size_t width = 6;
  const size_t height = 5;
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid);
  map->info.width = width;
  map->info.height = height;
  map->info.resolution = 1.0;
  map->info.origin.orientation.w = 1.0;
  map->data = std::vector<int8_t>(width * height, 0);
  map->data[2 + width * 3] = 100;
  map->data[3 + width * 3] = -1;

  nav_msgs::OccupancyGrid::Ptr map_overlay(new nav_msgs::OccupancyGrid);
  map_overlay->info.width = width;
  map_overlay->info.height = height;
  map_overlay->info.resolution = 1.0;
  map_overlay->info.origin.orientation.w = 1.0;
  map_overlay->data = std::vector<int8_t>(width * height, 0);
  map_overlay->data[2 + width * 4] = 100;
  map_overlay->data[4 + width * 4] = -1;

  map->data[unknown_x + width * unknown_y] = -1;
  map_overlay->data[unknown_x + width * unknown_y] = -1;

  costmap_cspace::Costmap3d cms1(4);
  auto cm_base1 = cms1.addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
  cm_base1->setExpansion(0.0, 2.0);
  cm_base1->setFootprint(footprint);
  auto cm_normal = cms1.addLayer<costmap_cspace::Costmap3dLayerFootprint>(
      costmap_cspace::MapOverlayMode::MAX);
  cm_normal->setExpansion(0.0, 2.0);
  cm_normal->setFootprint(footprint);
  cm_normal->setKeepUnknown(false);
  cm_base1->setBaseMap(map);
  cm_normal->processMapOverlay(map_overlay);

  costmap_cspace::Costmap3d cms2(4);
  auto cm_base2 = cms2.addRootLayer<costmap_cspace::Costmap3dLayerPlain>();
  cm_base2->setExpansion(0.0, 2.0);
  auto cm_plain = cms2.addLayer<costmap_cspace::Costmap3dLayerPlain>(
      costmap_cspace::MapOverlayMode::MAX);
  cm_plain->setExpansion(0.0, 2.0);
  cm_plain->setKeepUnknown(false);
  cm_base2->setBaseMap(map);
  cm_plain->processMapOverlay(map_overlay);

  const costmap_cspace::CSpace3DMsg::Ptr normal_result = cm_normal->getMapOverlay();
  const costmap_cspace::CSpace3DMsg::Ptr plain_result = cm_plain->getMapOverlay();
  for (size_t yaw = 0; yaw < normal_result->info.angle; ++yaw)
  {
    for (size_t y = 0; y < normal_result->info.height; ++y)
    {
      for (size_t x = 0; x < normal_result->info.width; ++x)
      {
        EXPECT_EQ(normal_result->getCost(x, y, yaw), plain_result->getCost(x, y, yaw))
            << " x:" << x << " y:" << y << " yaw:" << yaw;
        EXPECT_EQ(plain_result->getCost(x, y, yaw), plain_result->getCost(x, y, 0))
            << " x:" << x << " y:" << y << " yaw:" << yaw;
      }
    }
  }
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
