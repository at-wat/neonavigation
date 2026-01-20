/*
 * Copyright (c) 2018-2019, the neonavigation authors
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
 *       contributors may be used to endorse or produce products derived from
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

/*
 * Copyright (c) 2018-2019, the neonavigation authors
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
 *       contributors may be used to endorse or produce products derived from
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

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <functional>
#include <chrono>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/planner_3d/motion_primitive_builder.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>
#include <planner_cspace/planner_3d/motion_cache.h>

using planner_cspace::planner_3d::CostCoeff;
using planner_cspace::planner_3d::MotionCache;
using planner_cspace::planner_3d::MotionPrimitiveBuilder;
using planner_cspace::planner_3d::MotionPrimitiveType;

int main(int argc, char** argv)
{
  if (argc < 6)
  {
    std::cerr
        << "Usage: primitive_exporter [type: 0:DEFAULT, 1:BEZIER] [range] "
        << "[linear_res] [angular_res] [min_curve_radius] [bezier_cp_dist] "
        << "[cp_mode: 0:FIXED, 1:OPTIMIZE]"
        << std::endl;
    return 1;
  }

  int type_int = std::stoi(argv[1]);
  float range_metric = std::stof(argv[2]);
  float linear_res = std::stof(argv[3]);
  int range = static_cast<int>(std::round(range_metric / linear_res));
  float angular_res = std::stof(argv[4]);
  float min_curve_radius = std::stof(argv[5]);
  float bezier_cp_dist = (argc >= 7) ? std::stof(argv[6]) : 0.5f;
  int cp_mode_int = (argc >= 8) ? std::stoi(argv[7]) : 0;

  costmap_cspace_msgs::MapMetaData3D map_info;
  map_info.linear_resolution = linear_res;
  map_info.angular_resolution = angular_res;
  map_info.angle = std::lround(2.0 * M_PI / angular_res);

  CostCoeff cc;
  cc.min_curve_radius_ = min_curve_radius;
  cc.motion_primitive_type_ = static_cast<MotionPrimitiveType>(type_int);
  cc.bezier_cp_dist_ = bezier_cp_dist;
  cc.bezier_cp_mode_ = static_cast<CostCoeff::BezierCpMode>(cp_mode_int);
  cc.angle_resolution_aspect_ =
      2.0f / std::tan(map_info.angular_resolution);

  const auto start_time = std::chrono::high_resolution_clock::now();
  const auto primitives = MotionPrimitiveBuilder::build(map_info, cc, range);
  const auto end_time = std::chrono::high_resolution_clock::now();
  const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               end_time - start_time)
                               .count();

  std::cerr << "Build time: " << duration_ms << " ms (mode: "
            << (cp_mode_int == 0 ? "FIXED" : "OPTIMIZE") << ")" << std::endl;

  MotionCache motion_cache;
  auto dummy_addr = [](const planner_cspace::CyclicVecInt<3, 2>&, size_t& baddr, size_t& addr)
  {
    baddr = 0;
    addr = 0;
  };
  motion_cache.reset(linear_res, angular_res, range, dummy_addr, 0.1, 0.05,
                     cc.motion_primitive_type_, cc.bezier_cp_dist_, cp_mode_int);

  // JSON output
  std::cout << "{\"map_info\": {\"linear_resolution\": " << linear_res
            << ", \"angular_resolution\": " << angular_res
            << ", \"angle\": " << map_info.angle << "}," << std::endl;
  std::cout << " \"primitives\": [" << std::endl;

  for (size_t i = 0; i < primitives.size(); ++i)
  {
    std::cout << "  [" << std::endl;
    for (size_t j = 0; j < primitives[i].size(); ++j)
    {
      const auto& p = primitives[i][j];
      // MotionPrimitiveBuilder::Vec stores (dx, dy, dyaw). Cache keys expect (dx, dy, abs_yaw).
      const int goal_yaw = static_cast<int>(
          (static_cast<int>(i) + p[2] + map_info.angle) %
          static_cast<int>(map_info.angle));
      MotionPrimitiveBuilder::Vec goal(p[0], p[1], goal_yaw);

      std::cout << "    {\"end\": {\"x\": " << p[0] << ", \"y\": " << p[1]
                << ", \"yaw\": " << goal_yaw << "}, ";

      // Interpolated path
      std::cout << "\"path\": [";
      const auto it = motion_cache.find(static_cast<int>(i), goal);
      if (it != motion_cache.end(static_cast<int>(i)))
      {
        const auto& path = it->second.getInterpolatedMotion();
        for (size_t k = 0; k < path.size(); ++k)
        {
          std::cout << "{\"x\": " << path[k][0] * linear_res << ", \"y\": "
                    << path[k][1] * linear_res << ", \"yaw\": "
                    << path[k][2] * angular_res << "}, ";
        }
      }
      // Always add the true endpoint for visualization (centered at start, matching path points)
      MotionPrimitiveBuilder::Vecf end_pos(p[0] * linear_res, p[1] * linear_res, goal_yaw * angular_res);
      std::cout << "{\"x\": " << end_pos[0] << ", \"y\": " << end_pos[1] << ", \"yaw\": " << end_pos[2] << "}";
      std::cout << "]}";

      if (j < primitives[i].size() - 1)
        std::cout << ",";
      std::cout << std::endl;
    }
    std::cout << "  ]";
    if (i < primitives.size() - 1)
      std::cout << ",";
    std::cout << std::endl;
  }
  std::cout << " ]" << std::endl;
  std::cout << "}" << std::endl;

  return 0;
}
