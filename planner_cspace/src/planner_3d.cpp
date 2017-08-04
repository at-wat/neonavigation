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
#include <costmap_cspace/CSpace3D.h>
#include <costmap_cspace/CSpace3DUpdate.h>
#include <planner_cspace/PlannerStatus.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>

#include <string>
#include <list>
#include <vector>

#include <costmap_cspace/node_handle_float.h>
#include <grid_astar.h>

float signf(float a)
{
  if (a < 0)
    return -1;
  else if (a > 0)
    return 1;
  return 0;
}

typedef grid_astar<3, 2> astar;

class planner_3d
{
private:
  ros::NodeHandle_f nh;
  ros::Subscriber sub_map;
  ros::Subscriber sub_map_update;
  ros::Subscriber sub_goal;
  ros::Publisher pub_path;
  ros::Publisher pub_debug;
  ros::Publisher pub_hist;
  ros::Publisher pub_start;
  ros::Publisher pub_end;
  ros::Publisher pub_status;
  ros::ServiceServer srs_forget;

  tf::TransformListener tfl;

  astar as;
  astar::gridmap<char, 0x40> cm;
  astar::gridmap<char, 0x40> cm_hist;
  astar::gridmap<char, 0x80> cm_rough;
  astar::gridmap<char, 0x40> cm_base;
  astar::gridmap<char, 0x80> cm_rough_base;
  astar::gridmap<char, 0x80> cm_hyst;
  astar::gridmap<float> cost_estim_cache;

  astar::vecf euclid_cost_coef;
  float euclid_cost(const astar::vec &v, const astar::vecf coef)
  {
    auto vc = v;
    float cost = 0;
    for (int i = 0; i < as.get_noncyclic(); i++)
    {
      cost += powf(coef[i] * vc[i], 2.0);
    }
    cost = sqrtf(cost);
    for (int i = as.get_noncyclic(); i < as.get_dim(); i++)
    {
      vc.cycle(vc[i], cm.size[i]);
      cost += fabs(coef[i] * vc[i]);
    }
    return cost;
  }
  float euclid_cost(const astar::vec &v)
  {
    return euclid_cost(v, euclid_cost_coef);
  }

  class rotation_cache
  {
  public:
    std::unique_ptr<astar::vecf[]> c;
    astar::vec size;
    int ser_size;
    void reset(const astar::vec &size)
    {
      size_t ser_size = 1;
      for (int i = 0; i < 3; i++)
      {
        ser_size *= size[i];
      }
      this->size = size;
      this->ser_size = ser_size;

      c.reset(new astar::vecf[ser_size]);
    }
    explicit rotation_cache(const astar::vec &size)
    {
      reset(size);
    }
    rotation_cache()
    {
    }
    astar::vecf &operator[](const astar::vec &pos)
    {
      size_t addr = pos[2];
      for (int i = 1; i >= 0; i--)
      {
        addr *= size[i];
        addr += pos[i];
      }
      return c[addr];
    }
  };
  std::vector<rotation_cache> rotgm;
  rotation_cache *rot_cache;

  costmap_cspace::MapMetaData3D map_info;
  std_msgs::Header map_header;
  float max_vel;
  float max_ang_vel;
  float freq;
  float freq_min;
  float search_range;
  int range;
  int local_range;
  double local_range_f;
  int longcut_range;
  double longcut_range_f;
  int esc_range;
  int esc_angle;
  double esc_range_f;
  int unknown_cost;
  bool has_map;
  bool has_goal;
  bool has_start;
  bool goal_updated;
  bool remember_updates;
  bool fast_map_update;
  std::vector<astar::vec> search_list;
  std::vector<astar::vec> search_list_rough;
  int hist_cost;
  int hist_cnt_max;
  int hist_cnt_thres;
  double hist_ignore_range_f;
  int hist_ignore_range;
  double hist_ignore_range_max_f;
  int hist_ignore_range_max;
  bool temporary_escape;

  double pos_jump;
  double yaw_jump;

  // Cost weights
  class cost_coeff
  {
  public:
    float weight_decel;
    float weight_backward;
    float weight_ang_vel;
    float weight_costmap;
    float weight_hysteresis;
    float in_place_turn;
    float hysteresis_max_dist;
  };
  cost_coeff cc;

  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped goal_raw;
  astar::vecf ec;
  astar::vecf ec_rough;
  astar::vecf resolution;
  double goal_tolerance_lin_f;
  double goal_tolerance_ang_f;
  double goal_tolerance_ang_finish;
  int goal_tolerance_lin;
  int goal_tolerance_ang;
  float angle_resolution_aspect;

  enum debug_mode
  {
    DEBUG_HYSTERESIS,
    DEBUG_HISTORY,
    DEBUG_COST_ESTIM
  };
  debug_mode debug_out;

  planner_cspace::PlannerStatus status;

  bool find_best;
  float sw_wait;

  float rough_cost_max;

  bool force_goal_orientation;

  bool escaping;

  bool cb_forget(std_srvs::EmptyRequest &req,
                 std_srvs::EmptyResponse &res)
  {
    ROS_WARN("Forgetting remembered costmap.");
    if (has_map)
      cm_hist.clear(0);

    return true;
  }
  void cb_goal(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    goal_raw = goal = *msg;

    double len2 =
        goal.pose.orientation.x * goal.pose.orientation.x +
        goal.pose.orientation.y * goal.pose.orientation.y +
        goal.pose.orientation.z * goal.pose.orientation.z +
        goal.pose.orientation.w * goal.pose.orientation.w;
    if (fabs(len2 - 1.0) < 0.1)
    {
      escaping = false;
      has_goal = true;
      status.status = planner_cspace::PlannerStatus::DOING;
      pub_status.publish(status);
      update_goal();
    }
    else
    {
      has_goal = false;
    }
  }
  void fill_costmap(reservable_priority_queue<astar::pq> &open,
                    astar::gridmap<float> &g,
                    const astar::vec &s, const astar::vec &e)
  {
    auto s_rough = s;
    s_rough[2] = 0;

    while (true)
    {
      if (open.size() < 1)
        break;
      const auto center = open.top();
      const auto p = center.v;
      const auto c = center.p_raw;
      open.pop();
      if (c > g[p])
        continue;
      if (c - ec_rough[0] * (range + local_range + longcut_range) > g[s_rough])
        continue;

      astar::vec d;
      d[2] = 0;

      const int range_rough = 4;
      for (d[0] = -range_rough; d[0] <= range_rough; d[0]++)
      {
        for (d[1] = -range_rough; d[1] <= range_rough; d[1]++)
        {
          if (d[0] == 0 && d[1] == 0)
            continue;
          if (d.sqlen() > range_rough * range_rough)
            continue;

          const auto next = p + d;
          if ((unsigned int)next[0] >= (unsigned int)map_info.width ||
              (unsigned int)next[1] >= (unsigned int)map_info.height)
            continue;
          auto &gnext = g[next];
          if (gnext < 0)
            continue;

          float cost = 0;

          {
            float v[3], dp[3], sum = 0;
            float distf = d.len();
            const int dist = distf;
            distf /= dist;
            v[0] = p[0];
            v[1] = p[1];
            v[2] = 0;
            dp[0] = static_cast<float>(d[0]) / dist;
            dp[1] = static_cast<float>(d[1]) / dist;
            astar::vec pos(v);
            char c = 0;
            for (int i = 0; i < dist; i++)
            {
              pos[0] = lroundf(v[0]);
              pos[1] = lroundf(v[1]);
              c = cm_rough[pos];
              if (c > 99)
                break;
              sum += c;
              v[0] += dp[0];
              v[1] += dp[1];
            }
            if (c > 99)
              continue;
            cost += sum * map_info.linear_resolution * distf * cc.weight_costmap / 100.0;
          }
          cost += euclid_cost(d, ec_rough);

          const auto gp = c + cost;
          if (gnext > gp)
          {
            gnext = gp;
            open.push(astar::pq(gp, gp, next));
          }
        }
      }
    }
    rough_cost_max = g[s_rough] + ec_rough[0] * (range + local_range);
  }
  bool search_available_pos(astar::vec &s, const int xy_range, const int angle_range,
                            const int cost_acceptable = 50, const int min_xy_range = 0)
  {
    astar::vec d;
    float range_min = FLT_MAX;
    astar::vec s_out;
    ROS_DEBUG("%d, %d  (%d,%d,%d)", xy_range, angle_range, s[0], s[1], s[2]);
    for (d[2] = -angle_range; d[2] <= angle_range; d[2]++)
    {
      for (d[0] = -xy_range; d[0] <= xy_range; d[0]++)
      {
        for (d[1] = -xy_range; d[1] <= xy_range; d[1]++)
        {
          if (d[0] == 0 && d[1] == 0 && d[2] == 0)
            continue;
          if (d.sqlen() > xy_range * xy_range)
            continue;
          if (d.sqlen() < min_xy_range * min_xy_range)
            continue;

          auto s2 = s + d;
          if ((unsigned int)s2[0] >= (unsigned int)map_info.width ||
              (unsigned int)s2[1] >= (unsigned int)map_info.height)
            continue;
          s2.cycle_unsigned(s2[2], map_info.angle);
          if (cm[s2] >= cost_acceptable)
            continue;
          auto cost = euclid_cost(d, ec);
          if (cost < range_min)
          {
            range_min = cost;
            s_out = s2;
          }
        }
      }
    }

    if (range_min == FLT_MAX)
    {
      if (cost_acceptable != 100)
      {
        return search_available_pos(s, xy_range, angle_range, 100);
      }
      return false;
    }
    s = s_out;
    s.cycle_unsigned(s[2], map_info.angle);
    ROS_DEBUG("    (%d,%d,%d)", s[0], s[1], s[2]);
    return true;
  }
  void update_goal(const bool goal_changed = true)
  {
    if (!has_map || !has_goal || !has_start)
    {
      ROS_ERROR("Goal received, however map/goal/start are not ready. (%d/%d/%d)",
                static_cast<int>(has_map), static_cast<int>(has_goal), static_cast<int>(has_start));
      return;
    }

    astar::vec s, e;
    metric2grid(s[0], s[1], s[2],
                start.pose.position.x, start.pose.position.y,
                tf::getYaw(start.pose.orientation));
    s.cycle_unsigned(s[2], map_info.angle);
    metric2grid(e[0], e[1], e[2],
                goal.pose.position.x, goal.pose.position.y,
                tf::getYaw(goal.pose.orientation));
    e.cycle_unsigned(e[2], map_info.angle);
    ROS_INFO("New goal received (%d, %d, %d)",
             e[0], e[1], e[2]);

    const auto ts = boost::chrono::high_resolution_clock::now();
    reservable_priority_queue<astar::pq> open;
    auto &g = cost_estim_cache;

    g.clear(FLT_MAX);
    if (cm[e] == 100)
    {
      if (!search_available_pos(e, esc_range, esc_angle))
      {
        ROS_WARN("Oops! Goal is in Rock!");
        return;
      }
      ROS_INFO("Goal moved (%d, %d, %d)",
               e[0], e[1], e[2]);
      float x, y, yaw;
      grid2metric(e[0], e[1], e[2], x, y, yaw);
      goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      goal.pose.position.x = x;
      goal.pose.position.y = y;
    }
    if (cm[s] == 100)
    {
      if (!search_available_pos(s, esc_range, esc_angle))
      {
        ROS_WARN("Oops! You are in Rock!");
        return;
      }
    }

    e[2] = 0;
    g[e] = -ec_rough[0] * 0.5;  // Decrement to reduce calculation error
    open.push(astar::pq(g[e], g[e], e));
    fill_costmap(open, g, s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache generated (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    g[e] = 0;

    if (goal_changed)
      cm_hyst.clear(0);

    publish_costmap();

    goal_updated = true;
  }
  void publish_costmap()
  {
    sensor_msgs::PointCloud debug;
    debug.header = map_header;
    debug.header.stamp = ros::Time::now();
    {
      astar::vec p;
      for (p[1] = 0; p[1] < cost_estim_cache.size[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cost_estim_cache.size[0]; p[0]++)
        {
          p[2] = 0;
          float x, y, yaw;
          grid2metric(p[0], p[1], p[2], x, y, yaw);
          geometry_msgs::Point32 point;
          point.x = x;
          point.y = y;
          switch (debug_out)
          {
            case DEBUG_HYSTERESIS:
              if (cost_estim_cache[p] == FLT_MAX)
                continue;
              point.z = cm_hyst[p] * 0.01;
              break;
            case DEBUG_HISTORY:
              if (cm_rough_base[p] != 0)
                continue;
              point.z = cm_hist[p] * 0.01;
              break;
            case DEBUG_COST_ESTIM:
              if (cost_estim_cache[p] == FLT_MAX)
                continue;
              point.z = cost_estim_cache[p] / 500;
              break;
          }
          debug.points.push_back(point);
        }
      }
    }
    pub_debug.publish(debug);
  }
  void cb_map_update(const costmap_cspace::CSpace3DUpdate::ConstPtr &msg)
  {
    if (!has_map)
      return;
    ROS_DEBUG("Map updated");

    cm = cm_base;
    cm_rough = cm_rough_base;
    if (remember_updates)
    {
      sensor_msgs::PointCloud pc;
      pc.header = map_header;
      pc.header.stamp = ros::Time::now();

      astar::vec p;
      for (p[1] = 0; p[1] < cm_hist.size[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cm_hist.size[0]; p[0]++)
        {
          p[2] = 0;
          if (cm_hist[p] > hist_cnt_thres)
          {
            astar::vec p2;
            p2 = p;
            for (p2[2] = 0; p2[2] < static_cast<int>(map_info.angle); p2[2]++)
            {
              if (cm[p2] < hist_cost)
                cm[p2] = hist_cost;
            }

            float x, y, yaw;
            grid2metric(p[0], p[1], p[2], x, y, yaw);
            geometry_msgs::Point32 point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            pc.points.push_back(point);
          }
        }
      }
      pub_hist.publish(pc);
    }

    {
      astar::vec p;
      astar::vec center;
      center[0] = msg->width / 2;
      center[1] = msg->height / 2;
      astar::vec gp;
      gp[0] = msg->x;
      gp[1] = msg->y;
      gp[2] = msg->yaw;
      astar::vec gp_rough = gp;
      gp_rough[2] = 0;
      const int hist_ignore_range_sq = hist_ignore_range * hist_ignore_range;
      const int hist_ignore_range_max_sq =
          hist_ignore_range_max * hist_ignore_range_max;
      for (p[0] = 0; p[0] < static_cast<int>(msg->width); p[0]++)
      {
        for (p[1] = 0; p[1] < static_cast<int>(msg->height); p[1]++)
        {
          int cost_min = 100;
          int cost_max = 0;
          for (p[2] = 0; p[2] < static_cast<int>(msg->angle); p[2]++)
          {
            const size_t addr = ((p[2] * msg->height) + p[1]) * msg->width + p[0];
            char c = msg->data[addr];
            if (c < cost_min)
              cost_min = c;
            if (c > cost_max)
              cost_max = c;
          }
          p[2] = 0;
          cm_rough[gp_rough + p] = cost_min;

          astar::vec pos = gp + p;
          pos[2] = 0;
          if (cost_min == 100)
          {
            astar::vec p2 = p - center;
            float sqlen = p2.sqlen();
            if (sqlen > hist_ignore_range_sq &&
                sqlen < hist_ignore_range_max_sq)
            {
              auto &ch = cm_hist[pos];
              ch++;
              if (ch > hist_cnt_max)
                ch = hist_cnt_max;
            }
          }
          else if (cost_max == 0)
          {
            astar::vec p2 = p - center;
            float sqlen = p2.sqlen();
            if (sqlen < hist_ignore_range_max_sq)
            {
              auto &ch = cm_hist[pos];
              ch--;
              if (ch < 0)
                ch = 0;
            }
          }

          for (p[2] = 0; p[2] < static_cast<int>(msg->angle); p[2]++)
          {
            const size_t addr = ((p[2] * msg->height) + p[1]) * msg->width + p[0];
            char c = msg->data[addr];
            if (c < 0)
              c = unknown_cost;
            cm[gp + p] = c;
          }
        }
      }
    }
    if (!has_goal || !has_start)
      return;

    if (!fast_map_update)
    {
      update_goal(false);
      return;
    }

    astar::vec s, e;
    metric2grid(s[0], s[1], s[2],
                start.pose.position.x, start.pose.position.y,
                tf::getYaw(start.pose.orientation));
    s.cycle_unsigned(s[2], map_info.angle);
    metric2grid(e[0], e[1], e[2],
                goal.pose.position.x, goal.pose.position.y,
                tf::getYaw(goal.pose.orientation));
    e.cycle_unsigned(e[2], map_info.angle);

    if (cm[e] == 100)
    {
      update_goal(false);
      return;
    }

    e[2] = 0;

    const auto ts = boost::chrono::high_resolution_clock::now();
    auto &g = cost_estim_cache;

    astar::vec p, p_cost_min;
    p[2] = 0;
    float cost_min = FLT_MAX;
    for (p[1] = static_cast<int>(msg->y); p[1] < static_cast<int>(msg->y + msg->height); p[1]++)
    {
      for (p[0] = static_cast<int>(msg->x); p[0] < static_cast<int>(msg->x + msg->width); p[0]++)
      {
        if (cost_min > g[p])
        {
          p_cost_min = p;
          cost_min = g[p];
        }
      }
    }

    reservable_priority_queue<astar::pq> open;
    reservable_priority_queue<astar::pq> erase;
    open.reserve(map_info.width * map_info.height / 2);
    erase.reserve(map_info.width * map_info.height / 2);

    if (cost_min != FLT_MAX)
      erase.push(astar::pq(cost_min, cost_min, p_cost_min));
    while (true)
    {
      if (erase.size() < 1)
        break;
      const auto center = erase.top();
      const auto p = center.v;
      erase.pop();

      if (g[p] == FLT_MAX)
        continue;
      g[p] = FLT_MAX;

      astar::vec d;
      d[2] = 0;
      for (d[0] = -1; d[0] <= 1; d[0]++)
      {
        for (d[1] = -1; d[1] <= 1; d[1]++)
        {
          if (!((d[0] == 0) ^ (d[1] == 0)))
            continue;
          const auto next = p + d;
          if ((unsigned int)next[0] >= (unsigned int)map_info.width ||
              (unsigned int)next[1] >= (unsigned int)map_info.height)
            continue;
          auto &gn = g[next];
          if (gn == FLT_MAX)
            continue;
          if (gn < cost_min)
          {
            open.push(astar::pq(gn, gn, next));
            continue;
          }
          erase.push(astar::pq(gn, gn, next));
        }
      }
    }
    if (open.size() == 0)
    {
      open.push(astar::pq(-ec_rough[0] * 0.5, -ec_rough[0] * 0.5, e));
    }
    {
      astar::vec p;
      auto &g = cost_estim_cache;
      p[2] = 0;
      for (p[0] = 0; p[0] < static_cast<int>(map_info.width); p[0]++)
      {
        for (p[1] = 0; p[1] < static_cast<int>(map_info.height); p[1]++)
        {
          auto &gp = g[p];
          if (gp > rough_cost_max)
          {
            open.push(astar::pq(gp, gp, p));
          }
        }
      }
    }

    fill_costmap(open, g, s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache updated (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    publish_costmap();
  }
  void cb_map(const costmap_cspace::CSpace3D::ConstPtr &msg)
  {
    ROS_INFO("Map received");
    ROS_INFO(" linear_resolution %0.2f x (%dx%d) px", msg->info.linear_resolution,
             msg->info.width, msg->info.height);
    ROS_INFO(" angular_resolution %0.2f x %d px", msg->info.angular_resolution,
             msg->info.angle);
    ROS_INFO(" origin %0.3f m, %0.3f m, %0.3f rad",
             msg->info.origin.position.x,
             msg->info.origin.position.y,
             tf::getYaw(msg->info.origin.orientation));

    float ec_val[3] =
        {
          1.0f / max_vel,
          1.0f / max_vel,
          1.0f * cc.weight_ang_vel / max_ang_vel
        };

    ec = astar::vecf(ec_val);
    ec_val[2] = 0;
    ec_rough = astar::vecf(ec_val);

    if (map_info.linear_resolution != msg->info.linear_resolution ||
        map_info.angular_resolution != msg->info.angular_resolution)
    {
      astar::vec d;
      range = static_cast<int>(search_range / msg->info.linear_resolution);

      search_list.clear();
      for (d[0] = -range; d[0] <= range; d[0]++)
      {
        for (d[1] = -range; d[1] <= range; d[1]++)
        {
          if (d.sqlen() > range * range)
            continue;
          for (d[2] = 0; d[2] < static_cast<int>(msg->info.angle); d[2]++)
          {
            search_list.push_back(d);
          }
        }
      }
      search_list_rough.clear();
      for (d[0] = -range; d[0] <= range; d[0]++)
      {
        for (d[1] = -range; d[1] <= range; d[1]++)
        {
          if (d.sqlen() > range * range)
            continue;
          d[2] = 0;
          search_list_rough.push_back(d);
        }
      }
      ROS_DEBUG("Search list updated (range: ang %d, lin %d) %d",
                msg->info.angle, range, static_cast<int>(search_list.size()));

      rotgm.resize(msg->info.angle);
      for (int i = 0; i < static_cast<int>(msg->info.angle); i++)
      {
        const int size[3] =
            {
              range * 2 + 1,
              range * 2 + 1,
              static_cast<int>(msg->info.angle)
            };
        auto &r = rotgm[i];
        r.reset(astar::vec(size));

        astar::vec d;

        for (d[0] = 0; d[0] <= range * 2; d[0]++)
        {
          for (d[1] = 0; d[1] <= range * 2; d[1]++)
          {
            for (d[2] = 0; d[2] < static_cast<int>(msg->info.angle); d[2]++)
            {
              const float val[3] =
                  {
                    (d[0] - range) * msg->info.linear_resolution,
                    (d[1] - range) * msg->info.linear_resolution,
                    d[2] * msg->info.angular_resolution
                  };
              auto v = astar::vecf(val);
              rotate(v, -i * msg->info.angular_resolution);
              r[d] = v;
            }
          }
        }
      }
      ROS_DEBUG("Rotation cache generated");
    }
    map_info = msg->info;
    map_header = msg->header;

    resolution[0] = 1.0 / map_info.linear_resolution;
    resolution[1] = 1.0 / map_info.linear_resolution;
    resolution[2] = 1.0 / map_info.angular_resolution;

    hist_ignore_range = lroundf(hist_ignore_range_f / map_info.linear_resolution);
    hist_ignore_range_max = lroundf(hist_ignore_range_max_f / map_info.linear_resolution);
    local_range = lroundf(local_range_f / map_info.linear_resolution);
    longcut_range = lroundf(longcut_range_f / map_info.linear_resolution);
    esc_range = lroundf(esc_range_f / map_info.linear_resolution);
    esc_angle = map_info.angle / 8;
    goal_tolerance_lin = lroundf(goal_tolerance_lin_f / map_info.linear_resolution);
    goal_tolerance_ang = lroundf(goal_tolerance_ang_f / map_info.angular_resolution);

    int size[3] =
        {
          static_cast<int>(map_info.width),
          static_cast<int>(map_info.height),
          static_cast<int>(map_info.angle)
        };
    as.reset(astar::vec(size));
    cm.reset(astar::vec(size));
    size[2] = 1;
    cost_estim_cache.reset(astar::vec(size));
    cm_rough.reset(astar::vec(size));
    cm_hyst.reset(astar::vec(size));
    cm_hist.reset(astar::vec(size));

    astar::vec p;
    for (p[0] = 0; p[0] < static_cast<int>(map_info.width); p[0]++)
    {
      for (p[1] = 0; p[1] < static_cast<int>(map_info.height); p[1]++)
      {
        int cost_min = 100;
        for (p[2] = 0; p[2] < static_cast<int>(map_info.angle); p[2]++)
        {
          const size_t addr = ((p[2] * size[1]) + p[1]) * size[0] + p[0];
          char c = msg->data[addr];
          if (c < 0)
            c = unknown_cost;
          cm[p] = c;
          if (c < cost_min)
            cost_min = c;
        }
        p[2] = 0;
        cm_rough[p] = cost_min;
      }
    }
    ROS_DEBUG("Map copied");
    cm_hyst.clear(0);

    has_map = true;

    cm_rough_base = cm_rough;
    cm_base = cm;
    cm_hist.clear(0);

    update_goal();
  }

public:
  planner_3d()
    : nh("~")
  {
    sub_map = nh.subscribe("costmap", 1, &planner_3d::cb_map, this);
    sub_map_update = nh.subscribe("costmap_update", 1, &planner_3d::cb_map_update, this);
    sub_goal = nh.subscribe("goal", 1, &planner_3d::cb_goal, this);
    pub_path = nh.advertise<nav_msgs::Path>("path", 1, true);
    pub_debug = nh.advertise<sensor_msgs::PointCloud>("debug", 1, true);
    pub_hist = nh.advertise<sensor_msgs::PointCloud>("remembered", 1, true);
    pub_start = nh.advertise<geometry_msgs::PoseStamped>("path_start", 1, true);
    pub_end = nh.advertise<geometry_msgs::PoseStamped>("path_end", 1, true);
    pub_status = nh.advertise<planner_cspace::PlannerStatus>("status", 1, true);
    srs_forget = nh.advertiseService("forget", &planner_3d::cb_forget, this);

    nh.param_cast("freq", freq, 4.0f);
    nh.param_cast("freq_min", freq_min, 2.0f);
    nh.param_cast("search_range", search_range, 0.4f);

    nh.param_cast("max_vel", max_vel, 0.3f);
    nh.param_cast("max_ang_vel", max_ang_vel, 0.6f);

    nh.param_cast("weight_decel", cc.weight_decel, 50.0f);
    nh.param_cast("weight_backward", cc.weight_backward, 0.9f);
    nh.param_cast("weight_ang_vel", cc.weight_ang_vel, 1.0f);
    nh.param_cast("weight_costmap", cc.weight_costmap, 50.0f);
    nh.param_cast("cost_in_place_turn", cc.in_place_turn, 30.0f);
    nh.param_cast("hysteresis_max_dist", cc.hysteresis_max_dist, 0.3f);
    nh.param_cast("weight_hysteresis", cc.weight_hysteresis, 5.0f);

    nh.param("goal_tolerance_lin", goal_tolerance_lin_f, 0.05);
    nh.param("goal_tolerance_ang", goal_tolerance_ang_f, 0.1);
    nh.param("goal_tolerance_ang_finish", goal_tolerance_ang_finish, 0.05);

    nh.param("unknown_cost", unknown_cost, 100);
    nh.param("hist_cnt_max", hist_cnt_max, 20);
    nh.param("hist_cnt_thres", hist_cnt_thres, 19);
    nh.param("hist_cost", hist_cost, 90);
    nh.param("hist_ignore_range", hist_ignore_range_f, 0.6);
    nh.param("hist_ignore_range_max", hist_ignore_range_max_f, 1.25);
    nh.param("remember_updates", remember_updates, false);

    nh.param("local_range", local_range_f, 2.5);
    nh.param("longcut_range", longcut_range_f, 0.0);
    nh.param("esc_range", esc_range_f, 0.25);

    nh.param_cast("sw_wait", sw_wait, 2.0f);
    nh.param("find_best", find_best, true);

    nh.param("pos_jump", pos_jump, 1.0);
    nh.param("yaw_jump", yaw_jump, 1.5);

    nh.param("force_goal_orientation", force_goal_orientation, true);

    nh.param("temporary_escape", temporary_escape, true);

    nh.param("fast_map_update", fast_map_update, false);
    if (fast_map_update)
    {
      ROS_WARN("planner_3d: Experimental fast_map_update is enabled. ");
    }
    std::string debug_mode;
    nh.param("debug_mode", debug_mode, std::string("cost_estim"));
    if (debug_mode == "hyst")
      debug_out = DEBUG_HYSTERESIS;
    else if (debug_mode == "hist")
      debug_out = DEBUG_HISTORY;
    else if (debug_mode == "cost_estim")
      debug_out = DEBUG_COST_ESTIM;

    int queue_size_limit;
    nh.param("queue_size_limit", queue_size_limit, 0);
    as.set_queue_size_limit(queue_size_limit);

    status.status = planner_cspace::PlannerStatus::DONE;

    has_map = false;
    has_goal = false;
    has_start = false;
    goal_updated = false;

    escaping = false;
  }
  void spin()
  {
    ros::Rate wait(freq);
    ROS_DEBUG("Initialized");

    geometry_msgs::PoseStamped start_prev;
    start_prev.pose.orientation.w = 1.0;

    while (ros::ok())
    {
      wait.sleep();
      ros::spinOnce();

      if (has_map)
      {
        start.header.frame_id = "base_link";
        start.header.stamp = ros::Time(0);
        start.pose.orientation.x = 0.0;
        start.pose.orientation.y = 0.0;
        start.pose.orientation.z = 0.0;
        start.pose.orientation.w = 1.0;
        start.pose.position.x = 0;
        start.pose.position.y = 0;
        start.pose.position.z = 0;
        try
        {
          tfl.waitForTransform(map_header.frame_id, "base_link",
                               map_header.stamp, ros::Duration(0.1));
          tfl.transformPose(map_header.frame_id, start, start);
        }
        catch (tf::TransformException &e)
        {
          // ROS_INFO("planner_cspace: Transform failed %s", e.what());
          continue;
        }
        {
          float x_diff = start.pose.position.x - start_prev.pose.position.x;
          float y_diff = start.pose.position.y - start_prev.pose.position.y;
          float yaw_diff = tf::getYaw(start.pose.orientation) - tf::getYaw(start_prev.pose.orientation);
          if (yaw_diff > M_PI)
            yaw_diff -= M_PI * 2;

          if (powf(x_diff, 2.0) + powf(y_diff, 2.0) > powf(pos_jump, 2.0) ||
              fabs(yaw_diff) > yaw_jump)
          {
            ROS_ERROR("Position jumped, history cleared");
            cm_hist.clear(0);
          }
          start_prev = start;
        }

        has_start = true;
        if (!goal_updated && has_goal)
          update_goal();
      }

      if (has_map && has_goal && has_start)
      {
        if (status.status == planner_cspace::PlannerStatus::FINISHING)
        {
          float yaw_s = tf::getYaw(start.pose.orientation);
          float yaw_g;
          if (force_goal_orientation)
            yaw_g = tf::getYaw(goal_raw.pose.orientation);
          else
            yaw_g = tf::getYaw(goal.pose.orientation);

          float yaw_diff = yaw_s - yaw_g;
          if (yaw_diff > M_PI)
            yaw_diff -= M_PI * 2.0;
          else if (yaw_diff < -M_PI)
            yaw_diff += M_PI * 2.0;
          if (fabs(yaw_diff) < goal_tolerance_ang_finish)
          {
            status.status = planner_cspace::PlannerStatus::DONE;
            has_goal = false;
            ROS_INFO("Path plan finished");
          }
        }
        else
        {
          if (escaping)
            status.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
          else
            status.error = planner_cspace::PlannerStatus::GOING_WELL;

          nav_msgs::Path path;
          make_plan(start.pose, goal.pose, path, true);
          pub_path.publish(path);

          if (switch_detect(path))
          {
            ROS_INFO("Will have switch back");
            if (sw_wait > 0.0)
            {
              ros::Duration(sw_wait).sleep();
            }
          }
        }
      }
      else if (!has_goal)
      {
        nav_msgs::Path path;
        path.header = map_header;
        path.header.stamp = ros::Time::now();
        pub_path.publish(path);
      }
      pub_status.publish(status);
    }
  }

private:
  void grid2metric(
      const int x, const int y, const int yaw,
      float &gx, float &gy, float &gyaw)
  {
    gx = x * map_info.linear_resolution + map_info.origin.position.x;
    gy = y * map_info.linear_resolution + map_info.origin.position.y;
    gyaw = yaw * map_info.angular_resolution;
  }
  void grid2metric(const std::list<astar::vec> &path_grid,
                   nav_msgs::Path &path, const astar::vec &v_start)
  {
    path.header = map_header;
    path.header.stamp = ros::Time::now();

    // static int cnt = 0;
    // cnt ++;
    float x_ = 0, y_ = 0, yaw_ = 0;
    astar::vec p_;
    bool init = false;
    for (auto &p : path_grid)
    {
      float x, y, yaw;
      grid2metric(p[0], p[1], p[2], x, y, yaw);
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;

      // printf("%d %d %d  %f\n", p[0], p[1], p[2], as.g[p]);
      if (init)
      {
        auto ds = v_start - p;
        auto d = p - p_;
        float diff_val[3] =
            {
              d[0] * map_info.linear_resolution,
              d[1] * map_info.linear_resolution,
              p[2] * map_info.angular_resolution
            };
        astar::vecf motion_(diff_val);
        rotate(motion_, -p_[2] * map_info.angular_resolution);

        float inter = 0.1 / d.len();

        const astar::vecf motion = motion_;
        float cos_v = cosf(motion[2]);
        float sin_v = sinf(motion[2]);
        if (d[0] == 0 && d[1] == 0)
        {
          ps.pose.position.x = x;
          ps.pose.position.y = y;
          ps.pose.position.z = 0;
          ps.pose.orientation =
              tf::createQuaternionMsgFromYaw(yaw);
          path.poses.push_back(ps);
        }
        else if (fabs(sin_v) < 0.001 ||
                 ds.sqlen() > local_range * local_range)
        {
          for (float i = 0; i < 1.0; i += inter)
          {
            float x2, y2, yaw2;
            x2 = x_ * (1 - i) + x * i;
            y2 = y_ * (1 - i) + y * i;
            yaw2 = yaw_ * (1 - i) + yaw * i;
            ps.pose.position.x = x2;
            ps.pose.position.y = y2;
            ps.pose.position.z = 0;
            ps.pose.orientation =
                tf::createQuaternionMsgFromYaw(yaw2);
            path.poses.push_back(ps);
          }
        }
        else
        {
          float r1 = motion[1] + motion[0] * cos_v / sin_v;
          float r2 = sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0));
          if (motion[0] * sin_v < 0)
            r2 = -r2;

          float cx, cy, cx_, cy_, dyaw;
          dyaw = yaw - yaw_;
          if (dyaw < -M_PI)
            dyaw += 2 * M_PI;
          else if (dyaw > M_PI)
            dyaw -= 2 * M_PI;

          cx = x + r2 * cosf(yaw + M_PI / 2);
          cy = y + r2 * sinf(yaw + M_PI / 2);
          cx_ = x_ + r1 * cosf(yaw_ + M_PI / 2);
          cy_ = y_ + r1 * sinf(yaw_ + M_PI / 2);

          for (float i = 0; i < 1.0; i += inter)
          {
            float r = r1 * (1.0 - i) + r2 * i;
            float cx2 = cx_ * (1.0 - i) + cx * i;
            float cy2 = cy_ * (1.0 - i) + cy * i;
            float cyaw = yaw_ + i * dyaw;

            float x2, y2, yaw2;
            x2 = cx2 - r * cosf(cyaw + M_PI / 2);
            y2 = cy2 - r * sinf(cyaw + M_PI / 2);
            yaw2 = cyaw;
            ps.pose.position.x = x2;
            ps.pose.position.y = y2;
            ps.pose.position.z = 0;
            ps.pose.orientation =
                tf::createQuaternionMsgFromYaw(yaw2);
            path.poses.push_back(ps);
          }
          ps.pose.position.x = x;
          ps.pose.position.y = y;
          ps.pose.position.z = 0;
          ps.pose.orientation =
              tf::createQuaternionMsgFromYaw(yaw);
          path.poses.push_back(ps);
        }
      }

      x_ = x;
      y_ = y;
      yaw_ = yaw;
      p_ = p;
      init = true;
    }
  }
  void metric2grid(
      int &x, int &y, int &yaw,
      const float gx, const float gy, const float gyaw)
  {
    x = lroundf((gx - map_info.origin.position.x) / map_info.linear_resolution);
    y = lroundf((gy - map_info.origin.position.y) / map_info.linear_resolution);
    yaw = lroundf(gyaw / map_info.angular_resolution);
  }
  bool make_plan(const geometry_msgs::Pose &gs, const geometry_msgs::Pose &ge,
                 nav_msgs::Path &path, bool hyst)
  {
    astar::vec s, e;
    metric2grid(s[0], s[1], s[2],
                gs.position.x, gs.position.y, tf::getYaw(gs.orientation));
    s.cycle_unsigned(s[2], map_info.angle);
    metric2grid(e[0], e[1], e[2],
                ge.position.x, ge.position.y, tf::getYaw(ge.orientation));
    e.cycle_unsigned(e[2], map_info.angle);

    geometry_msgs::PoseStamped p;
    p.header = map_header;
    float x, y, yaw;
    grid2metric(e[0], e[1], e[2], x, y, yaw);
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    p.pose.position.x = x;
    p.pose.position.y = y;
    pub_end.publish(p);

    if (cm[s] == 100)
    {
      if (!search_available_pos(s, esc_range, esc_angle))
      {
        ROS_WARN("Oops! You are in Rock!");
        status.error = planner_cspace::PlannerStatus::IN_ROCK;
        return false;
      }
      ROS_INFO("Start moved");
    }
    auto s_rough = s;
    s_rough[2] = 0;

    if (cost_estim_cache[s_rough] == FLT_MAX)
    {
      status.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      ROS_WARN("Goal unreachable. History cleared.");
      cm_hist.clear(0);
      if (!escaping && temporary_escape)
      {
        e = s;
        if (search_available_pos(e, esc_range, esc_angle, 50, esc_range / 2))
        {
          escaping = true;
          ROS_INFO("Temporary goal (%d, %d, %d)",
                   e[0], e[1], e[2]);
          float x, y, yaw;
          grid2metric(e[0], e[1], e[2], x, y, yaw);
          goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          goal.pose.position.x = x;
          goal.pose.position.y = y;

          update_goal();
          return false;
        }
      }
      return false;
    }

    grid2metric(s[0], s[1], s[2], x, y, yaw);
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    p.pose.position.x = x;
    p.pose.position.y = y;
    pub_start.publish(p);

    auto diff = s - e;
    diff.cycle(diff[2], map_info.angle);
    if (diff.sqlen() <= goal_tolerance_lin * goal_tolerance_lin &&
        abs(diff[2]) <= goal_tolerance_ang)
    {
      path.header = map_header;
      path.header.stamp = ros::Time::now();
      path.poses.resize(1);
      path.poses[0].header = path.header;
      if (force_goal_orientation)
        path.poses[0].pose = goal_raw.pose;
      else
        path.poses[0].pose = ge;

      if (escaping)
      {
        goal = goal_raw;
        escaping = false;
        update_goal();
        ROS_INFO("Escaped");
      }
      else
      {
        status.status = planner_cspace::PlannerStatus::FINISHING;
        ROS_INFO("Path plan finishing");
      }
      return true;
    }

    auto range_limit = cost_estim_cache[s_rough] - (local_range + range) * ec[0];
    angle_resolution_aspect = 1.0 / tanf(map_info.angular_resolution);

    // ROS_INFO("Planning from (%d, %d, %d) to (%d, %d, %d)",
    //   s[0], s[1], s[2], e[0], e[1], e[2]);
    std::list<astar::vec> path_grid;
    // const auto ts = boost::chrono::high_resolution_clock::now();
    if (!as.search(s, e, path_grid,
                   std::bind(&planner_3d::cb_cost,
                             this, std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3, std::placeholders::_4, hyst),
                   std::bind(&planner_3d::cb_cost_estim,
                             this, std::placeholders::_1, std::placeholders::_2),
                   std::bind(&planner_3d::cb_search,
                             this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3),
                   std::bind(&planner_3d::cb_progress,
                             this, std::placeholders::_1),
                   range_limit,
                   1.0f / freq_min,
                   true))
    {
      ROS_WARN("Path plan failed (goal unreachable)");
      status.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      if (!find_best)
        return false;
    }
    // const auto tnow = boost::chrono::high_resolution_clock::now();
    // ROS_INFO("Path found (%0.3f sec.)",
    //   boost::chrono::duration<float>(tnow - ts).count());

    grid2metric(path_grid, path, s);

    if (hyst)
    {
      std::unordered_map<astar::vec, bool, astar::vec> path_points;
      float max_dist = cc.hysteresis_max_dist / map_info.linear_resolution;
      int path_range = range + max_dist + 1;
      for (auto &p : path_grid)
      {
        astar::vec d;
        for (d[0] = -path_range; d[0] <= path_range; d[0]++)
        {
          for (d[1] = -path_range; d[1] <= path_range; d[1]++)
          {
            auto point = p + d;
            point[2] = 0;
            if ((unsigned int)point[0] >= (unsigned int)map_info.width ||
                (unsigned int)point[1] >= (unsigned int)map_info.height)
              continue;
            path_points[point] = true;
          }
        }
      }

      cm_hyst.clear(100);
      // const auto ts = boost::chrono::high_resolution_clock::now();
      for (auto &ps : path_points)
      {
        auto &p = ps.first;
        float d_min = FLT_MAX;
        auto it_prev = path_grid.begin();
        for (auto it = path_grid.begin(); it != path_grid.end(); it++)
        {
          if (it != it_prev)
          {
            auto d = p.dist_linestrip(*it_prev, *it);
            if (d < d_min)
              d_min = d;
          }
          it_prev = it;
        }
        if (d_min < 0)
          d_min = 0;
        if (d_min > max_dist)
          d_min = max_dist;
        cm_hyst[p] = d_min * 100.0 / max_dist;
      }
      // const auto tnow = boost::chrono::high_resolution_clock::now();
      // ROS_INFO("Hysteresis map generated (%0.3f sec.)",
      //   boost::chrono::duration<float>(tnow - ts).count());
      publish_costmap();
    }

    return true;
  }
  bool rough;
  std::vector<astar::vec> &cb_search(
      const astar::vec &p,
      const astar::vec &s, const astar::vec &e)
  {
    const auto ds = s - p;
    rot_cache = &rotgm[p[2]];

    if (ds.sqlen() < local_range * local_range)
    {
      rough = false;
      euclid_cost_coef = ec;
      return search_list;
    }
    rough = true;
    euclid_cost_coef = ec_rough;
    return search_list_rough;
  }
  bool cb_progress(const std::list<astar::vec> &path_grid)
  {
    nav_msgs::Path path;
    path.header = map_header;
    path.header.stamp = ros::Time::now();
    // grid2metric(path_grid, path);
    pub_path.publish(path);
    ROS_WARN("Search timed out");
    return true;
  }
  void rotate(astar::vecf &v, const float &ang)
  {
    const astar::vecf tmp = v;
    const float cos_v = cosf(ang);
    const float sin_v = sinf(ang);

    v[0] = cos_v * tmp[0] - sin_v * tmp[1];
    v[1] = sin_v * tmp[0] + cos_v * tmp[1];
    v[2] = v[2] + ang;
    if (v[2] > M_PI)
      v[2] -= 2 * M_PI;
    else if (v[2] < -M_PI)
      v[2] += 2 * M_PI;
  }
  float cb_cost_estim(const astar::vec &s, const astar::vec &e)
  {
    auto s2 = s;
    s2[2] = 0;
    auto cost = cost_estim_cache[s2];
    if (cost == FLT_MAX)
      return FLT_MAX;
    if (!rough)
    {
      if (s2[2] > static_cast<int>(map_info.angle) / 2)
        s2[2] -= map_info.angle;
      cost += ec_rough[2] * fabs(s[2]);
    }
    return cost;
  }
  bool switch_detect(const nav_msgs::Path &path)
  {
    geometry_msgs::Pose p_prev;
    bool first(true);
    bool dir_set(false);
    bool dir_prev(false);
    for (auto &p : path.poses)
    {
      if (!first)
      {
        float len = hypotf(
            p.pose.position.y - p_prev.position.y,
            p.pose.position.x - p_prev.position.x);
        if (len > 0.001)
        {
          float yaw = tf::getYaw(p.pose.orientation);
          float vel_yaw = atan2f(
              p.pose.position.y - p_prev.position.y,
              p.pose.position.x - p_prev.position.x);
          bool dir = false;
          if (cosf(yaw) * cosf(vel_yaw) + sinf(yaw) * sinf(vel_yaw) < 0)
            dir = true;

          if (dir_set && (dir_prev ^ dir))
          {
            return true;
          }
          dir_prev = dir;
          dir_set = true;
        }
      }
      first = false;
      p_prev = p.pose;
    }
    return false;
  }
  float cb_cost(const astar::vec &s, astar::vec &e,
                const astar::vec &v_goal,
                const astar::vec &v_start,
                const bool hyst)
  {
    const auto d = e - s;
    float cost = euclid_cost(d);

    if (d[0] == 0 && d[1] == 0)
    {
      // In-place turn
      int sum = 0;
      int dir = 1;
      if (d[2] > static_cast<int>(map_info.angle) / 2)
        dir = -1;
      astar::vec pos = s;
      for (int i = 0; i < abs(d[2]); i++)
      {
        pos[2] += dir;
        if (pos[2] < 0)
          pos[2] += map_info.angle;
        else if (pos[2] >= static_cast<int>(map_info.angle))
          pos[2] -= map_info.angle;
        const auto c = cm[pos];
        if (c > 99)
          return -1;
        sum += c;
      }

      float cost = sum * map_info.angular_resolution * ec[2] / ec[0];
      return cc.in_place_turn + cost;
    }

    astar::vec d2;
    d2[0] = d[0] + range;
    d2[1] = d[1] + range;
    d2[2] = e[2];
    const astar::vecf motion = (*rot_cache)[d2];

    const astar::vecf motion_grid = motion * resolution;
    // motion_grid[0] /= map_info.linear_resolution;
    // motion_grid[1] /= map_info.linear_resolution;
    // motion_grid[2] /= map_info.angular_resolution;

    if (lroundf(motion_grid[0]) == 0 && lroundf(motion_grid[1]) != 0)
    {
      // Not non-holonomic
      return -1;
    }

    if (fabs(motion[2]) >= 2.0 * M_PI / 4.0)
    {
      // Over 90 degree turn
      // must be separated into two curves
      return -1;
    }

    const float dist = motion.len();

    const float cos_v = cosf(motion[2]);
    const float sin_v = sinf(motion[2]);

    bool forward(true);
    if (motion[0] < 0)
      forward = false;

    if (!forward)
    {
      cost *= 1.0 + cc.weight_backward;
    }

    if (lroundf(motion_grid[2]) == 0)
    {
      float distf = d.len();

      if (motion_grid[0] == 0)
        return -1;  // side slip
      float aspect = motion[0] / motion[1];
      if (fabs(aspect) < angle_resolution_aspect * 2.0)
        return -1;  // large y offset

      // Go-straight
      float v[3], dp[3];
      int sum = 0, sum_hyst = 0;
      const int dist = distf;
      distf /= dist;
      v[0] = s[0];
      v[1] = s[1];
      v[2] = s[2];
      dp[0] = static_cast<float>(d[0]) / dist;
      dp[1] = static_cast<float>(d[1]) / dist;
      astar::vec pos(v);
      for (int i = 0; i < dist; i++)
      {
        pos[0] = lroundf(v[0]);
        pos[1] = lroundf(v[1]);
        const auto c = cm[pos];
        if (c > 99)
          return -1;
        if (hyst)
        {
          auto pos_rough = pos;
          pos_rough[2] = 0;
          sum_hyst += cm_hyst[pos_rough];
        }
        sum += c;
        v[0] += dp[0];
        v[1] += dp[1];
      }
      cost += sum * map_info.linear_resolution * distf * cc.weight_costmap / 100.0;
      cost += sum_hyst * map_info.linear_resolution * distf * cc.weight_hysteresis / 100.0;
    }
    else
    {
      // Curve
      if (motion[0] * motion[1] * motion[2] < 0)
        return -1;
      if (d.sqlen() < 4 * 4)
        return -1;
      if (fabs(motion[1]) <= map_info.linear_resolution * 0.5)
        return -1;

      const float r1 = motion[1] + motion[0] * cos_v / sin_v;
      float r2 = sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0));
      if (motion[0] * sin_v < 0)
        r2 = -r2;

      // curveture at the start pose and the end pose must be same
      if (fabs(r1 - r2) >= map_info.linear_resolution * 1.5)
      {
        // Drifted
        return -1;
      }

      const float curv_radius = (r1 + r2) / 2;

      float vel = max_vel;
      float ang_vel = cos_v * vel / (cos_v * motion[0] + sin_v * motion[1]);
      if (fabs(ang_vel) > max_ang_vel)
      {
        ang_vel = signf(ang_vel) * max_ang_vel;
        vel = fabs(curv_radius) * max_ang_vel;

        // Curve deceleration penalty
        cost += dist * fabs(vel / max_vel) * cc.weight_decel;
      }

      {
        float v[3], dp[3];
        int sum = 0, sum_hyst = 0;
        float distf = d.len();
        const int dist = distf;
        distf /= dist;
        v[0] = s[0];
        v[1] = s[1];
        v[2] = s[2];
        dp[0] = static_cast<float>(d[0]) / dist;
        dp[1] = static_cast<float>(d[1]) / dist;
        dp[2] = static_cast<float>(d[2]);
        if (dp[2] < -map_info.angle / 2)
          dp[2] += map_info.angle;
        else if (dp[2] >= map_info.angle / 2)
          dp[2] -= map_info.angle;
        dp[2] /= dist;
        astar::vec pos(v);
        for (int i = 0; i < dist; i++)
        {
          pos[0] = lroundf(v[0]);
          pos[1] = lroundf(v[1]);
          pos[2] = lroundf(v[2]);
          if (pos[2] < 0)
            pos[2] += map_info.angle;
          else if (pos[2] >= static_cast<int>(map_info.angle))
            pos[2] -= map_info.angle;
          const auto c = cm[pos];
          if (c > 99)
            return -1;
          sum += c;
          if (hyst)
          {
            auto pos_rough = pos;
            pos_rough[2] = 0;
            sum_hyst += cm_hyst[pos_rough];
          }
          v[0] += dp[0];
          v[1] += dp[1];
          v[2] += dp[2];
        }
        cost += sum * map_info.linear_resolution * distf * cc.weight_costmap / 100.0;
        cost += sum_hyst * map_info.linear_resolution * distf * cc.weight_hysteresis / 100.0;
      }
    }

    return cost;
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "planner_3d");

  planner_3d jy;
  jy.spin();

  return 0;
}
