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

#ifndef PLANNER_CSPACE_PLANNER_3D_GRID_METRIC_CONVERTER_H
#define PLANNER_CSPACE_PLANNER_3D_GRID_METRIC_CONVERTER_H

#include <costmap_cspace_msgs/MapMetaData3D.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <planner_cspace/cyclic_vec.h>

#include <list>
#include <memory>

namespace grid_metric_converter
{
void grid2Metric(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    const int x, const int y, const int yaw,
    float& gx, float& gy, float& gyaw)
{
  gx = (x + 0.5) * map_info.linear_resolution + map_info.origin.position.x;
  gy = (y + 0.5) * map_info.linear_resolution + map_info.origin.position.y;
  gyaw = yaw * map_info.angular_resolution;
}
void metric2Grid(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    int& x, int& y, int& yaw,
    const float gx, const float gy, const float gyaw)
{
  x = static_cast<int>(floor((gx - map_info.origin.position.x) / map_info.linear_resolution));
  y = static_cast<int>(floor((gy - map_info.origin.position.y) / map_info.linear_resolution));
  yaw = lroundf(gyaw / map_info.angular_resolution);
}

template <template <class, class> class STL_CONTAINER = std::list>
void grid2MetricPath(
    const costmap_cspace_msgs::MapMetaData3D& map_info,
    const float local_range,
    const STL_CONTAINER<CyclicVecInt<3, 2>, std::allocator<CyclicVecInt<3, 2>>>& path_grid,
    nav_msgs::Path& path, const CyclicVecInt<3, 2>& v_start)
{
  float x_prev = 0, y_prev = 0, yaw_prev = 0;
  // FIXME(at-wat): remove NOLINT after clang-format or roslint supports it
  CyclicVecInt<3, 2> p_prev({ 0, 0, 0 });  // NOLINT(whitespace/braces)
  bool init = false;

  for (const auto& p : path_grid)
  {
    float x, y, yaw;
    grid2Metric(map_info, p[0], p[1], p[2], x, y, yaw);
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;

    if (init)
    {
      const auto ds = v_start - p;
      const auto d = p - p_prev;
      const float diff_val[3] =
          {
            d[0] * map_info.linear_resolution,
            d[1] * map_info.linear_resolution,
            p[2] * map_info.angular_resolution
          };
      CyclicVecFloat<3, 2> motion(diff_val);
      motion.rotate(-p_prev[2] * map_info.angular_resolution);

      const float inter = 0.1 / d.len();

      const float cos_v = cosf(motion[2]);
      const float sin_v = sinf(motion[2]);
      if (d[0] == 0 && d[1] == 0)
      {
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0;
        ps.pose.orientation =
            tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
        path.poses.push_back(ps);
      }
      else if (fabs(sin_v) < 0.001 ||
               ds.sqlen() > local_range * local_range)
      {
        for (float i = 0; i < 1.0; i += inter)
        {
          const float x2 = x_prev * (1 - i) + x * i;
          const float y2 = y_prev * (1 - i) + y * i;
          const float yaw2 = yaw_prev * (1 - i) + yaw * i;
          ps.pose.position.x = x2;
          ps.pose.position.y = y2;
          ps.pose.position.z = 0;
          ps.pose.orientation =
              tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw2));
          path.poses.push_back(ps);
        }
      }
      else
      {
        const float r1 = motion[1] + motion[0] * cos_v / sin_v;
        const float r2 = std::copysign(
            sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0)),
            motion[0] * sin_v < 0);

        float dyaw = yaw - yaw_prev;
        if (dyaw < -M_PI)
          dyaw += 2 * M_PI;
        else if (dyaw > M_PI)
          dyaw -= 2 * M_PI;

        const float cx = x + r2 * cosf(yaw + M_PI / 2);
        const float cy = y + r2 * sinf(yaw + M_PI / 2);
        const float cx_prev = x_prev + r1 * cosf(yaw_prev + M_PI / 2);
        const float cy_prev = y_prev + r1 * sinf(yaw_prev + M_PI / 2);

        for (float i = 0; i < 1.0; i += inter)
        {
          const float r = r1 * (1.0 - i) + r2 * i;
          const float cx2 = cx_prev * (1.0 - i) + cx * i;
          const float cy2 = cy_prev * (1.0 - i) + cy * i;
          const float cyaw = yaw_prev + i * dyaw;

          const float x2 = cx2 - r * cosf(cyaw + M_PI / 2);
          const float y2 = cy2 - r * sinf(cyaw + M_PI / 2);
          const float yaw2 = cyaw;
          ps.pose.position.x = x2;
          ps.pose.position.y = y2;
          ps.pose.position.z = 0;
          ps.pose.orientation =
              tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw2));
          path.poses.push_back(ps);
        }
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0;
        ps.pose.orientation =
            tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
        path.poses.push_back(ps);
      }
    }

    x_prev = x;
    y_prev = y;
    yaw_prev = yaw;
    p_prev = p;
    init = true;
  }
}
}  // namespace grid_metric_converter

#endif  // PLANNER_CSPACE_PLANNER_3D_GRID_METRIC_CONVERTER_H
