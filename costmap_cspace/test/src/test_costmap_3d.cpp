/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
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

#include <gtest/gtest.h>

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_3d.h>

class Costmap3DOFTester : public costmap_cspace::Costmap3DOF
{
public:
  costmap_cspace::CSpace3Cache &getTemplate()
  {
    return cs_;
  }
  costmap_cspace::Polygon &getFootprint()
  {
    return footprint_p_;
  }
  int getRangeMax()
  {
    return range_max_;
  }
  float getFootprintRadius()
  {
    return footprint_radius_;
  }
};

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

TEST(Costmap3DOFTest, testCSpaceTemplate)
{
  Costmap3DOFTester cm;
  cm.setCSpaceConfig(1.0, 0.0, 0.0, 0, costmap_cspace::Costmap3DOF::map_overlay_mode::MAX);

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
  int x, y, a;
  temp.getSize(x, y, a);
  ASSERT_EQ(x, 2 * 2 + 1);
  ASSERT_EQ(y, 2 * 2 + 1);
  ASSERT_EQ(a, 4);
}

TEST(Costmap3DOFTest, testCSpace)
{
}

TEST(Costmap3DOFTest, testOverwrite)
{
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
