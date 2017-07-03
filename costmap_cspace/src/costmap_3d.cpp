/*
 * Copyright (c) 2014-2017, the neonavigation authors
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

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include <string>
#include <vector>

#include <costmap_cspace/node_handle_float.h>
#include <costmap_cspace/CSpace3D.h>
#include <costmap_cspace/CSpace3DUpdate.h>

class costmap_3d
{
private:
  ros::NodeHandle_f nh;
  ros::Subscriber sub_map;
  ros::Subscriber sub_map_overlay;
  ros::Publisher pub_costmap;
  ros::Publisher pub_costmap_update;
  ros::Publisher pub_footprint;
  ros::Publisher pub_debug;
  geometry_msgs::PolygonStamped footprint;

  int ang_resolution;
  float linear_expand;
  float linear_spread;
  int unknown_cost;
  enum map_overlay_mode
  {
    OVERWRITE,
    MAX
  };
  map_overlay_mode overlay_mode;

  float footprint_radius;

  class cspace3_cache
  {
  public:
    std::unique_ptr<char[]> c;
    int size[3];
    int center[3];
    void reset(const int &x, const int &y, const int &yaw)
    {
      size[0] = x * 2 + 1;
      size[1] = y * 2 + 1;
      size[2] = yaw;
      center[0] = x;
      center[1] = y;
      center[2] = 0;
      c.reset(new char[size[0] * size[1] * size[2]]);
    }
    char &e(const int &x, const int &y, const int &yaw)
    {
      return c[(yaw * size[1] + (y + center[1])) * size[0] + x + center[0]];
    }
  };
  cspace3_cache cs;

  class vec
  {
  public:
    float c[2];
    float &operator[](const int &i)
    {
      return c[i];
    }
    const float &operator[](const int &i) const
    {
      return c[i];
    }
    vec operator-(const vec &a) const
    {
      vec out = *this;
      out[0] -= a[0];
      out[1] -= a[1];
      return out;
    }
    float cross(const vec &a) const
    {
      return (*this)[0] * a[1] - (*this)[1] * a[0];
    }
    float dot(const vec &a) const
    {
      return (*this)[0] * a[0] + (*this)[1] * a[1];
    }
    float dist(const vec &a) const
    {
      return hypotf((*this)[0] - a[0], (*this)[1] - a[1]);
    }
    float dist_line(const vec &a, const vec &b) const
    {
      return (b - a).cross((*this) - a) / b.dist(a);
    }
    float dist_linestrip(const vec &a, const vec &b) const
    {
      if ((b - a).dot((*this) - a) <= 0)
        return this->dist(a);
      if ((a - b).dot((*this) - b) <= 0)
        return this->dist(b);
      return fabs(this->dist_line(a, b));
    }
  };
  class polygon
  {
  public:
    std::vector<vec> v;
    void move(const float &x, const float &y, const float &yaw)
    {
      float cos_v = cosf(yaw);
      float sin_v = sinf(yaw);
      for (auto &p : v)
      {
        auto tmp = p;
        p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
        p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
      }
    }
    bool inside(const vec &a)
    {
      int cn = 0;
      for (size_t i = 0; i < v.size() - 1; i++)
      {
        auto &v1 = v[i];
        auto &v2 = v[i + 1];
        if ((v1[1] <= a[1] && a[1] < v2[1]) ||
            (v2[1] <= a[1] && a[1] < v1[1]))
        {
          float lx;
          lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
          if (a[0] < lx)
            cn++;
        }
      }
      return ((cn & 1) == 1);
    }
    float dist(const vec &a)
    {
      float dist = FLT_MAX;
      for (size_t i = 0; i < v.size() - 1; i++)
      {
        auto &v1 = v[i];
        auto &v2 = v[i + 1];
        auto d = a.dist_linestrip(v1, v2);
        if (d < dist)
          dist = d;
      }
      return dist;
    }
  };

  polygon footprint_p;
  int range_max;

  bool XmlRpc_isNumber(XmlRpc::XmlRpcValue &value)
  {
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
           value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
  }

  costmap_cspace::CSpace3D map;
  costmap_cspace::CSpace3D map_overlay;
  nav_msgs::OccupancyGrid map_base;
  void cb_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    map_base = *msg;
    ROS_DEBUG("2D costmap received");

    map.header = msg->header;
    map.info.width = msg->info.width;
    map.info.height = msg->info.height;
    map.info.angle = ang_resolution;
    map.info.linear_resolution = msg->info.resolution;
    map.info.angular_resolution = 2.0 * M_PI / ang_resolution;
    map.info.origin = msg->info.origin;
    map.data.resize(msg->data.size() * map.info.angle);

    range_max =
        ceilf((footprint_radius + linear_expand + linear_spread) / map.info.linear_resolution);
    cs.reset(range_max, range_max, map.info.angle);

    // C-Space template
    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (int y = -range_max; y <= range_max; y++)
      {
        for (int x = -range_max; x <= range_max; x++)
        {
          auto f = footprint_p;
          f.move(x * map.info.linear_resolution,
                 y * map.info.linear_resolution,
                 yaw * map.info.angular_resolution);
          vec p;
          p[0] = 0;
          p[1] = 0;
          if (f.inside(p))
          {
            cs.e(x, y, yaw) = 100;
          }
          else
          {
            float d = f.dist(p);
            if (d < linear_expand)
            {
              cs.e(x, y, yaw) = 100;
            }
            else if (d < linear_expand + linear_spread)
            {
              cs.e(x, y, yaw) = 100 - (d - linear_expand) * 100 / linear_spread;
            }
            else
            {
              cs.e(x, y, yaw) = 0;
            }
          }
        }
      }
    }
    ROS_DEBUG("C-Space template generated");

    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < msg->data.size(); i++)
      {
        map.data[i + yaw * msg->data.size()] = 0;
      }
    }
    map_copy(map, *msg);
    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < msg->data.size(); i++)
      {
        if (msg->data[i] < 0)
        {
          map.data[i + yaw * msg->data.size()] = -1;
        }
      }
    }
    ROS_DEBUG("C-Space costmap generated");

    pub_costmap.publish(map);
    publish_debug(map);
  }
  void cb_map_overlay(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    ROS_DEBUG("Overlay 2D costmap received");
    if (map.header.frame_id != msg->header.frame_id)
    {
      ROS_ERROR("map and map_overlay must have same frame_id");
      return;
    }
    if (map.info.width < 1 ||
        map.info.height < 1)
      return;

    int ox = lroundf((msg->info.origin.position.x - map.info.origin.position.x) / map.info.linear_resolution);
    int oy = lroundf((msg->info.origin.position.y - map.info.origin.position.y) / map.info.linear_resolution);
    auto omap = *msg;
    if (overlay_mode == OVERWRITE)
    {
      for (unsigned int i = 0; i < msg->data.size(); i++)
      {
        if (omap.data[i] < 0)
        {
          int mx = lroundf((i % msg->info.width) * msg->info.resolution / map.info.linear_resolution);
          if (msg->info.width <= (unsigned int)mx)
            continue;
          int my = lroundf((i / msg->info.width) * msg->info.resolution / map.info.linear_resolution);
          if (msg->info.height <= (unsigned int)my)
            continue;

          omap.data[i] = map_base.data[(my + oy) * map_base.info.width + (mx + ox)];
        }
      }
    }
    map_overlay = map;
    map_copy(map_overlay, omap, true);
    ROS_DEBUG("C-Space costmap updated");

    int w = lroundf(msg->info.width * msg->info.resolution / map.info.linear_resolution);
    int h = lroundf(msg->info.height * msg->info.resolution / map.info.linear_resolution);
    update_map(map_overlay, ox, oy, 0, w, h, map.info.angle);
    publish_debug(map_overlay);
  }
  void map_copy(costmap_cspace::CSpace3D &map, const nav_msgs::OccupancyGrid &msg, bool overlay = false)
  {
    int ox = lroundf((msg.info.origin.position.x - map.info.origin.position.x) / map.info.linear_resolution);
    int oy = lroundf((msg.info.origin.position.y - map.info.origin.position.y) / map.info.linear_resolution);
    // Clear travelable area in OVERWRITE mode
    if (overlay_mode == OVERWRITE && overlay)
    {
      for (size_t yaw = 0; yaw < map.info.angle; yaw++)
      {
        for (unsigned int i = 0; i < msg.data.size(); i++)
        {
          auto &val = msg.data[i];
          if (val < 0)
            continue;

          int x = lroundf((i % msg.info.width) * msg.info.resolution / map.info.linear_resolution);
          if (x < range_max || static_cast<int>(msg.info.width) - range_max <= x)
            continue;
          int y = lroundf((i / msg.info.width) * msg.info.resolution / map.info.linear_resolution);
          if (y < range_max || static_cast<int>(msg.info.height) - range_max <= y)
            continue;

          int res_up = ceilf(msg.info.resolution / map.info.linear_resolution);
          for (int yp = 0; yp < res_up; yp++)
          {
            for (int xp = 0; xp < res_up; xp++)
            {
              auto &m = map.data[(yaw * map.info.height + (y + oy + yp)) * map.info.width + (x + ox + xp)];
              m = 0;
            }
          }
        }
      }
    }
    // Get max
    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < msg.data.size(); i++)
      {
        int gx = lroundf((i % msg.info.width) * msg.info.resolution / map.info.linear_resolution) + ox;
        int gy = lroundf((i / msg.info.width) * msg.info.resolution / map.info.linear_resolution) + oy;
        if ((unsigned int)gx >= map.info.width ||
            (unsigned int)gy >= map.info.height)
          continue;

        auto val = msg.data[i];
        if (val < 0)
        {
          if (overlay_mode != OVERWRITE && overlay)
            continue;
          bool edge = false;
          int mx = lroundf((i % msg.info.width) * msg.info.resolution / map.info.linear_resolution);
          if (mx < 1 || static_cast<int>(msg.info.width) - 1 <= mx)
            continue;
          int my = lroundf((i / msg.info.width) * msg.info.resolution / map.info.linear_resolution);
          if (my < 1 || static_cast<int>(msg.info.height) - 1 <= my)
            continue;

          for (int y = -1; y <= 1; y++)
          {
            for (int x = -1; x <= 1; x++)
            {
              if (x == 0 && y == 0)
                continue;
              if ((unsigned int)(mx + x) >= map.info.width ||
                  (unsigned int)(my + y) >= map.info.height)
                continue;
              size_t addr = i + y * map.info.width + x;
              if (addr >= msg.data.size())
                continue;
              auto v = msg.data[addr];
              if (v >= 0)
              {
                edge = true;
                break;
              }
            }
          }
          if (edge)
          {
            val = unknown_cost;
          }
        }
        if (val <= 0)
          continue;

        for (int y = -range_max; y <= range_max; y++)
        {
          for (int x = -range_max; x <= range_max; x++)
          {
            int x2 = gx + x;
            int y2 = gy + y;
            if ((unsigned int)x2 >= map.info.width ||
                (unsigned int)y2 >= map.info.height)
              continue;

            auto &m = map.data[(yaw * map.info.height + y2) * map.info.width + x2];
            auto c = cs.e(x, y, yaw) * val / 100;
            if (m < c)
              m = c;
          }
        }
      }
    }
  }
  void publish_debug(costmap_cspace::CSpace3D &map)
  {
    sensor_msgs::PointCloud pc;
    pc.header = map.header;
    pc.header.stamp = ros::Time::now();
    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map.info.width * map.info.height; i++)
      {
        int gx = i % map.info.width;
        int gy = i / map.info.width;
        if (map.data[i + yaw * map.info.width * map.info.height] < 100)
          continue;
        geometry_msgs::Point32 p;
        p.x = gx * map.info.linear_resolution + map.info.origin.position.x;
        p.y = gy * map.info.linear_resolution + map.info.origin.position.y;
        p.z = yaw * 0.1;
        pc.points.push_back(p);
      }
    }
    pub_debug.publish(pc);
  }

public:
  void update_map(costmap_cspace::CSpace3D &map, const int &x, const int &y, const int &yaw,
                  const int &width, const int &height, const int &angle)
  {
    costmap_cspace::CSpace3DUpdate update;
    update.header = map.header;
    map.header.stamp = ros::Time::now();
    update.x = x - range_max;
    update.y = y - range_max;
    update.width = width + range_max * 2;
    update.height = height + range_max * 2;
    if (update.x < 0)
    {
      update.width += update.x;
      update.x = 0;
    }
    if (update.y < 0)
    {
      update.height += update.y;
      update.y = 0;
    }
    if (update.x + update.width > map.info.width)
    {
      update.width = map.info.width - update.x;
    }
    if (update.y + update.height > map.info.height)
    {
      update.height = map.info.height - update.y;
    }
    update.yaw = yaw;
    update.angle = angle;
    update.data.resize(update.width * update.height * update.angle);

    vec p;
    for (p[0] = 0; p[0] < update.width; p[0]++)
    {
      for (p[1] = 0; p[1] < update.height; p[1]++)
      {
        for (p[2] = 0; p[2] < update.angle; p[2]++)
        {
          int x2 = update.x + p[0];
          int y2 = update.y + p[1];
          int yaw2 = yaw + p[2];

          auto &m = map.data[(yaw2 * map.info.height + y2) * map.info.width + x2];
          auto &up = update.data[(p[2] * update.height + p[1]) * update.width + p[0]];
          up = m;
        }
      }
    }
    pub_costmap_update.publish(update);
  }
  void spin()
  {
    ros::Rate wait(1);
    while (ros::ok())
    {
      wait.sleep();
      ros::spinOnce();
      footprint.header.stamp = ros::Time::now();
      pub_footprint.publish(footprint);
    }
  }
  costmap_3d()
    : nh("~")
  {
    nh.param("ang_resolution", ang_resolution, 16);
    nh.param("linear_expand", linear_expand, 0.2f);
    nh.param("linear_spread", linear_spread, 0.5f);
    nh.param("unknown_cost", unknown_cost, 0);
    std::string overlay_mode_str;
    nh.param("overlay_mode", overlay_mode_str, std::string(""));
    if (overlay_mode_str.compare("overwrite") == 0)
    {
      overlay_mode = OVERWRITE;
    }
    else if (overlay_mode_str.compare("max") == 0)
    {
      overlay_mode = MAX;
    }
    else
    {
      ROS_ERROR("Unknown overlay_mode \"%s\"", overlay_mode_str.c_str());
      ros::shutdown();
      return;
    }
    ROS_INFO("costmap_3d: %s mode", overlay_mode_str.c_str());

    XmlRpc::XmlRpcValue footprint_xml;
    if (!nh.hasParam("footprint"))
    {
      ROS_FATAL("Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified");
    }
    nh.getParam("footprint", footprint_xml);
    if (footprint_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xml.size() < 3)
    {
      ROS_FATAL("Invalid footprint");
      throw std::runtime_error("Invalid footprint");
    }
    footprint.polygon.points.clear();
    footprint.header.frame_id = "base_link";
    footprint_radius = 0;
    for (int i = 0; i < footprint_xml.size(); i++)
    {
      if (!XmlRpc_isNumber(footprint_xml[i][0]) ||
          !XmlRpc_isNumber(footprint_xml[i][1]))
      {
        ROS_FATAL("Invalid footprint value");
        throw std::runtime_error("Invalid footprint value");
      }

      geometry_msgs::Point32 point;
      vec v;
      v[0] = point.x = static_cast<double>(footprint_xml[i][0]);
      v[1] = point.y = static_cast<double>(footprint_xml[i][1]);
      point.z = 0;
      footprint.polygon.points.push_back(point);

      footprint_p.v.push_back(v);

      auto dist = hypotf(point.x, point.y);
      if (dist > footprint_radius)
        footprint_radius = dist;
    }
    footprint_p.v.push_back(footprint_p.v.front());
    ROS_INFO("footprint radius: %0.3f", footprint_radius);
    footprint.polygon.points.push_back(footprint.polygon.points[0]);

    sub_map = nh.subscribe("/map", 1, &costmap_3d::cb_map, this);
    sub_map_overlay = nh.subscribe("/map_overlay", 1, &costmap_3d::cb_map_overlay, this);
    pub_costmap = nh.advertise<costmap_cspace::CSpace3D>("costmap", 1, true);
    pub_costmap_update = nh.advertise<costmap_cspace::CSpace3DUpdate>("costmap_update", 1, true);
    pub_footprint = nh.advertise<geometry_msgs::PolygonStamped>("footprint", 2, true);
    pub_debug = nh.advertise<sensor_msgs::PointCloud>("debug", 1, true);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "costmap_3d");

  costmap_3d cm;
  cm.spin();

  return 0;
}
