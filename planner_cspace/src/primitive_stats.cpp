/*
 * Utility to count motion primitives per start/end yaw and direction.
 * Input arguments mirror primitive_exporter:
 *   primitive_stats [type: 0:DEFAULT, 1:BEZIER] [range] [linear_res] [angular_res]
 *                   [min_curve_radius] [bezier_cp_dist] [cp_mode: 0:FIXED, 1:OPTIMIZE]
 *
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
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)\n * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <planner_cspace/cyclic_vec.h>
#include <planner_cspace/planner_3d/motion_primitive_builder.h>
#include <planner_cspace/planner_3d/rotation_cache.h>

using planner_cspace::planner_3d::CostCoeff;
using planner_cspace::planner_3d::MotionPrimitiveBuilder;
using planner_cspace::planner_3d::MotionPrimitiveType;
using planner_cspace::planner_3d::RotationCache;

int main(int argc, char** argv)
{
  if (argc < 6)
  {
    std::cerr << "Usage: primitive_stats [type: 0:DEFAULT, 1:BEZIER] [range] "
              << "[linear_res] [angular_res] [min_curve_radius] [bezier_cp_dist] "
              << "[cp_mode: 0:FIXED, 1:OPTIMIZE]"
              << std::endl;
    return 1;
  }

  const int type_int = std::stoi(argv[1]);
  const int range = std::stoi(argv[2]);
  const float linear_res = std::stof(argv[3]);
  const float angular_res = std::stof(argv[4]);
  const float min_curve_radius = std::stof(argv[5]);
  const float bezier_cp_dist = (argc >= 7) ? std::stof(argv[6]) : 0.5f;
  const int cp_mode_int = (argc >= 8) ? std::stoi(argv[7]) : 0;

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

  RotationCache rot_cache;
  rot_cache.reset(map_info.linear_resolution, map_info.angular_resolution, range);

  const int angle = static_cast<int>(map_info.angle);

  struct Counts
  {
    int total = 0;           // includes rotations
    int forward_total = 0;   // excludes rotations
    int backward_total = 0;  // excludes rotations
    std::vector<int> forward_by_end;
    std::vector<int> backward_by_end;
  };

  std::vector<Counts> stats(angle);
  for (auto& s : stats)
  {
    s.forward_by_end.assign(angle, 0);
    s.backward_by_end.assign(angle, 0);
  }

  for (int syaw = 0; syaw < angle; ++syaw)
  {
    const auto& plist = primitives[syaw];
    stats[syaw].total = static_cast<int>(plist.size());

    for (const auto& p : plist)
    {
      // In-place rotation: count only in total, skip direction breakdown
      if (p[0] == 0 && p[1] == 0)
        continue;

      const int goal_yaw = (syaw + p[2] + angle) % angle;
      const planner_cspace::CyclicVecInt<3, 2> d2(p[0] + range, p[1] + range, goal_yaw);
      const auto motion = rot_cache.getMotion(syaw, d2);

      const bool forward = (motion[0] >= 0);
      if (forward)
      {
        ++stats[syaw].forward_total;
        ++stats[syaw].forward_by_end[goal_yaw];
      }
      else
      {
        ++stats[syaw].backward_total;
        ++stats[syaw].backward_by_end[goal_yaw];
      }
    }
  }

  // Pretty print
  std::cout << "Parameters: type=" << type_int
            << " cp_mode=" << cp_mode_int
            << " range=" << range
            << " linear_res=" << linear_res
            << " angular_res=" << angular_res
            << " min_curve_radius=" << min_curve_radius
            << " bezier_cp_dist=" << bezier_cp_dist
            << " angle_count=" << angle << std::endl;

  for (int syaw = 0; syaw < angle; ++syaw)
  {
    const auto& s = stats[syaw];
    std::cout << "start_yaw=" << syaw
              << " total=" << s.total
              << " forward(non-rot)=" << s.forward_total
              << " backward(non-rot)=" << s.backward_total << std::endl;

    for (int gyaw = 0; gyaw < angle; ++gyaw)
    {
      const int f = s.forward_by_end[gyaw];
      const int b = s.backward_by_end[gyaw];
      if (f == 0 && b == 0)
        continue;
      std::cout << "  end_yaw=" << gyaw << " forward=" << f << " backward=" << b << std::endl;
    }
  }

  return 0;
}
