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

class Planner3d
{
public:
  using Astar = GridAstar<3, 2>;

private:
  ros::NodeHandle_f nh_;
  ros::Subscriber subMap;
  ros::Subscriber subMapUpdate;
  ros::Subscriber subGoal;
  ros::Publisher pubPath;
  ros::Publisher pubDebug;
  ros::Publisher pubHist;
  ros::Publisher pubStart;
  ros::Publisher pub_end_;
  ros::Publisher pub_status_;
  ros::ServiceServer srs_forget_;

  tf::TransformListener tfl_;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;
  Astar::Gridmap<char, 0x40> cm_hist_;
  Astar::Gridmap<char, 0x80> cm_rough_;
  Astar::Gridmap<char, 0x40> cm_base_;
  Astar::Gridmap<char, 0x80> cm_rough_base_;
  Astar::Gridmap<char, 0x80> cm_hyst_;
  Astar::Gridmap<float> cost_estim_cache_;

  Astar::Vecf euclid_cost_coef_;

  float euclidCost(const Astar::Vec &v, const Astar::Vecf coef)
  {
    Astar::Vec vc = v;
    float cost = 0;
    for (int i = 0; i < as_.getNoncyclic(); i++)
    {
      cost += powf(coef[i] * vc[i], 2.0);
    }
    cost = sqrtf(cost);
    for (int i = as_.getNoncyclic(); i < as_.getDim(); i++)
    {
      vc.cycle(vc[i], cm_.size[i]);
      cost += fabs(coef[i] * vc[i]);
    }
    return cost;
  }
  float euclidCost(const Astar::Vec &v)
  {
    return euclidCost(v, euclid_cost_coef_);
  }

  class RotationCache
  {
  protected:
    std::unique_ptr<Astar::Vecf[]> c_;
    Astar::Vec size_;
    int ser_size_;

  public:
    void reset(const Astar::Vec &size)
    {
      size_t ser_size = 1;
      for (int i = 0; i < 3; i++)
      {
        ser_size *= size[i];
      }
      this->size_ = size;
      this->ser_size_ = ser_size;

      c_.reset(new Astar::Vecf[ser_size]);
    }
    explicit RotationCache(const Astar::Vec &size)
    {
      reset(size);
    }
    RotationCache()
    {
    }
    Astar::Vecf &operator[](const Astar::Vec &pos)
    {
      size_t addr = pos[2];
      for (int i = 1; i >= 0; i--)
      {
        addr *= size_[i];
        addr += pos[i];
      }
      return c_[addr];
    }
  };
  std::vector<RotationCache> rotgm_;
  RotationCache *rot_cache_;

  costmap_cspace::MapMetaData3D map_info_;
  std_msgs::Header map_header_;
  float max_vel_;
  float max_ang_vel_;
  float freq_;
  float freq_min_;
  float search_range_;
  int range_;
  int local_range_;
  double local_range_f_;
  int longcut_range_;
  double longcut_range_f_;
  int esc_range_;
  int esc_angle_;
  double esc_range_f_;
  int unknown_cost_;
  bool has_map_;
  bool has_goal_;
  bool has_start_;
  bool goal_updated_;
  bool remember_updates_;
  bool fast_map_update_;
  std::vector<Astar::Vec> search_list_;
  std::vector<Astar::Vec> search_list_rough_;
  int hist_cost_;
  int hist_cnt_max_;
  int hist_cnt_thres_;
  double hist_ignore_range_f_;
  int hist_ignore_range_;
  double hist_ignore_range_max_f_;
  int hist_ignore_range_max_;
  bool temporary_escape_;

  double pos_jump_;
  double yaw_jump_;

  // Cost weights
  class CostCoeff
  {
  public:
    float weight_decel_;
    float weight_backward_;
    float weight_ang_vel_;
    float weight_costmap_;
    float weight_hysteresis_;
    float in_place_turn_;
    float hysteresis_max_dist_;
  };
  CostCoeff cc_;

  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped goal_raw_;
  Astar::Vecf ec_;
  Astar::Vecf ec_rough_;
  Astar::Vecf resolution_;
  double goal_tolerance_lin_f_;
  double goal_tolerance_ang_f_;
  double goal_tolerance_ang_finish_;
  int goal_tolerance_lin_;
  int goal_tolerance_ang_;
  float angle_resolution_aspect_;

  enum DebugMode
  {
    DEBUG_HYSTERESIS,
    DEBUG_HISTORY,
    DEBUG_COST_ESTIM
  };
  DebugMode debug_out_;

  planner_cspace::PlannerStatus status_;

  bool find_best_;
  float sw_wait_;

  float rough_cost_max_;
  bool rough_;

  bool force_goal_orientation_;

  bool escaping_;

  bool cbForget(std_srvs::EmptyRequest &req,
                std_srvs::EmptyResponse &res)
  {
    ROS_WARN("Forgetting remembered costmap.");
    if (has_map_)
      cm_hist_.clear(0);

    return true;
  }
  void cbGoal(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    goal_raw_ = goal_ = *msg;

    double len2 =
        goal_.pose.orientation.x * goal_.pose.orientation.x +
        goal_.pose.orientation.y * goal_.pose.orientation.y +
        goal_.pose.orientation.z * goal_.pose.orientation.z +
        goal_.pose.orientation.w * goal_.pose.orientation.w;
    if (fabs(len2 - 1.0) < 0.1)
    {
      escaping_ = false;
      has_goal_ = true;
      status_.status = planner_cspace::PlannerStatus::DOING;
      pub_status_.publish(status_);
      updateGoal();
    }
    else
    {
      has_goal_ = false;
    }
  }
  void fillCostmap(reservable_priority_queue<Astar::PriorityVec> &open,
                   Astar::Gridmap<float> &g,
                   const Astar::Vec &s, const Astar::Vec &e)
  {
    Astar::Vec s_rough = s;
    s_rough[2] = 0;

    while (true)
    {
      if (open.size() < 1)
        break;
      const auto center = open.top();
      const Astar::Vec p = center.v_;
      const auto c = center.p_raw_;
      open.pop();
      if (c > g[p])
        continue;
      if (c - ec_rough_[0] * (range_ + local_range_ + longcut_range_) > g[s_rough])
        continue;

      Astar::Vec d;
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

          const Astar::Vec next = p + d;
          if ((unsigned int)next[0] >= (unsigned int)map_info_.width ||
              (unsigned int)next[1] >= (unsigned int)map_info_.height)
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
            Astar::Vec pos(v);
            char c = 0;
            for (int i = 0; i < dist; i++)
            {
              pos[0] = lroundf(v[0]);
              pos[1] = lroundf(v[1]);
              c = cm_rough_[pos];
              if (c > 99)
                break;
              sum += c;
              v[0] += dp[0];
              v[1] += dp[1];
            }
            if (c > 99)
              continue;
            cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / 100.0;
          }
          cost += euclidCost(d, ec_rough_);

          const auto gp = c + cost;
          if (gnext > gp)
          {
            gnext = gp;
            open.push(Astar::PriorityVec(gp, gp, next));
          }
        }
      }
    }
    rough_cost_max_ = g[s_rough] + ec_rough_[0] * (range_ + local_range_);
  }
  bool searchAvailablePos(Astar::Vec &s, const int xy_range, const int angle_range,
                          const int cost_acceptable = 50, const int min_xy_range = 0)
  {
    Astar::Vec d;
    float range_min = FLT_MAX;
    Astar::Vec s_out;
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

          Astar::Vec s2 = s + d;
          if ((unsigned int)s2[0] >= (unsigned int)map_info_.width ||
              (unsigned int)s2[1] >= (unsigned int)map_info_.height)
            continue;
          s2.cycle_unsigned(s2[2], map_info_.angle);
          if (cm_[s2] >= cost_acceptable)
            continue;
          auto cost = euclidCost(d, ec_);
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
        return searchAvailablePos(s, xy_range, angle_range, 100);
      }
      return false;
    }
    s = s_out;
    s.cycle_unsigned(s[2], map_info_.angle);
    ROS_DEBUG("    (%d,%d,%d)", s[0], s[1], s[2]);
    return true;
  }
  void updateGoal(const bool goal_changed = true)
  {
    if (!has_map_ || !has_goal_ || !has_start_)
    {
      ROS_ERROR("Goal received, however map/goal/start are not ready. (%d/%d/%d)",
                static_cast<int>(has_map_), static_cast<int>(has_goal_), static_cast<int>(has_start_));
      return;
    }

    Astar::Vec s, e;
    metric2Grid(s[0], s[1], s[2],
                start_.pose.position.x, start_.pose.position.y,
                tf::getYaw(start_.pose.orientation));
    s.cycle_unsigned(s[2], map_info_.angle);
    metric2Grid(e[0], e[1], e[2],
                goal_.pose.position.x, goal_.pose.position.y,
                tf::getYaw(goal_.pose.orientation));
    e.cycle_unsigned(e[2], map_info_.angle);
    ROS_INFO("New goal received (%d, %d, %d)",
             e[0], e[1], e[2]);

    const auto ts = boost::chrono::high_resolution_clock::now();
    reservable_priority_queue<Astar::PriorityVec> open;
    auto &g = cost_estim_cache_;

    g.clear(FLT_MAX);
    if (cm_[e] == 100)
    {
      if (!searchAvailablePos(e, esc_range_, esc_angle_))
      {
        ROS_WARN("Oops! Goal is in Rock!");
        return;
      }
      ROS_INFO("Goal moved (%d, %d, %d)",
               e[0], e[1], e[2]);
      float x, y, yaw;
      grid2Metric(e[0], e[1], e[2], x, y, yaw);
      goal_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      goal_.pose.position.x = x;
      goal_.pose.position.y = y;
    }
    if (cm_[s] == 100)
    {
      if (!searchAvailablePos(s, esc_range_, esc_angle_))
      {
        ROS_WARN("Oops! You are in Rock!");
        return;
      }
    }

    e[2] = 0;
    g[e] = -ec_rough_[0] * 0.5;  // Decrement to reduce calculation error
    open.push(Astar::PriorityVec(g[e], g[e], e));
    fillCostmap(open, g, s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache generated (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    g[e] = 0;

    if (goal_changed)
      cm_hyst_.clear(0);

    publishCostmap();

    goal_updated_ = true;
  }
  void publishCostmap()
  {
    sensor_msgs::PointCloud debug;
    debug.header = map_header_;
    debug.header.stamp = ros::Time::now();
    {
      Astar::Vec p;
      for (p[1] = 0; p[1] < cost_estim_cache_.size[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cost_estim_cache_.size[0]; p[0]++)
        {
          p[2] = 0;
          float x, y, yaw;
          grid2Metric(p[0], p[1], p[2], x, y, yaw);
          geometry_msgs::Point32 point;
          point.x = x;
          point.y = y;
          switch (debug_out_)
          {
            case DEBUG_HYSTERESIS:
              if (cost_estim_cache_[p] == FLT_MAX)
                continue;
              point.z = cm_hyst_[p] * 0.01;
              break;
            case DEBUG_HISTORY:
              if (cm_rough_base_[p] != 0)
                continue;
              point.z = cm_hist_[p] * 0.01;
              break;
            case DEBUG_COST_ESTIM:
              if (cost_estim_cache_[p] == FLT_MAX)
                continue;
              point.z = cost_estim_cache_[p] / 500;
              break;
          }
          debug.points.push_back(point);
        }
      }
    }
    pubDebug.publish(debug);
  }
  void cbMapUpdate(const costmap_cspace::CSpace3DUpdate::ConstPtr &msg)
  {
    if (!has_map_)
      return;
    ROS_DEBUG("Map updated");

    cm_ = cm_base_;
    cm_rough_ = cm_rough_base_;
    if (remember_updates_)
    {
      sensor_msgs::PointCloud pc;
      pc.header = map_header_;
      pc.header.stamp = ros::Time::now();

      Astar::Vec p;
      for (p[1] = 0; p[1] < cm_hist_.size[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cm_hist_.size[0]; p[0]++)
        {
          p[2] = 0;
          if (cm_hist_[p] > hist_cnt_thres_)
          {
            Astar::Vec p2;
            p2 = p;
            for (p2[2] = 0; p2[2] < static_cast<int>(map_info_.angle); p2[2]++)
            {
              if (cm_[p2] < hist_cost_)
                cm_[p2] = hist_cost_;
            }

            float x, y, yaw;
            grid2Metric(p[0], p[1], p[2], x, y, yaw);
            geometry_msgs::Point32 point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            pc.points.push_back(point);
          }
        }
      }
      pubHist.publish(pc);
    }

    {
      Astar::Vec p;
      Astar::Vec center;
      center[0] = msg->width / 2;
      center[1] = msg->height / 2;
      Astar::Vec gp;
      gp[0] = msg->x;
      gp[1] = msg->y;
      gp[2] = msg->yaw;
      Astar::Vec gp_rough = gp;
      gp_rough[2] = 0;
      const int hist_ignore_range_sq = hist_ignore_range_ * hist_ignore_range_;
      const int hist_ignore_range_max_sq =
          hist_ignore_range_max_ * hist_ignore_range_max_;
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
          cm_rough_[gp_rough + p] = cost_min;

          Astar::Vec pos = gp + p;
          pos[2] = 0;
          if (cost_min == 100)
          {
            Astar::Vec p2 = p - center;
            float sqlen = p2.sqlen();
            if (sqlen > hist_ignore_range_sq &&
                sqlen < hist_ignore_range_max_sq)
            {
              auto &ch = cm_hist_[pos];
              ch++;
              if (ch > hist_cnt_max_)
                ch = hist_cnt_max_;
            }
          }
          else if (cost_max == 0)
          {
            Astar::Vec p2 = p - center;
            float sqlen = p2.sqlen();
            if (sqlen < hist_ignore_range_max_sq)
            {
              auto &ch = cm_hist_[pos];
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
              c = unknown_cost_;
            cm_[gp + p] = c;
          }
        }
      }
    }
    if (!has_goal_ || !has_start_)
      return;

    if (!fast_map_update_)
    {
      updateGoal(false);
      return;
    }

    Astar::Vec s, e;
    metric2Grid(s[0], s[1], s[2],
                start_.pose.position.x, start_.pose.position.y,
                tf::getYaw(start_.pose.orientation));
    s.cycle_unsigned(s[2], map_info_.angle);
    metric2Grid(e[0], e[1], e[2],
                goal_.pose.position.x, goal_.pose.position.y,
                tf::getYaw(goal_.pose.orientation));
    e.cycle_unsigned(e[2], map_info_.angle);

    if (cm_[e] == 100)
    {
      updateGoal(false);
      return;
    }

    e[2] = 0;

    const auto ts = boost::chrono::high_resolution_clock::now();
    auto &g = cost_estim_cache_;

    Astar::Vec p, p_cost_min;
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

    reservable_priority_queue<Astar::PriorityVec> open;
    reservable_priority_queue<Astar::PriorityVec> erase;
    open.reserve(map_info_.width * map_info_.height / 2);
    erase.reserve(map_info_.width * map_info_.height / 2);

    if (cost_min != FLT_MAX)
      erase.push(Astar::PriorityVec(cost_min, cost_min, p_cost_min));
    while (true)
    {
      if (erase.size() < 1)
        break;
      const Astar::PriorityVec center = erase.top();
      const Astar::Vec p = center.v_;
      erase.pop();

      if (g[p] == FLT_MAX)
        continue;
      g[p] = FLT_MAX;

      Astar::Vec d;
      d[2] = 0;
      for (d[0] = -1; d[0] <= 1; d[0]++)
      {
        for (d[1] = -1; d[1] <= 1; d[1]++)
        {
          if (!((d[0] == 0) ^ (d[1] == 0)))
            continue;
          const Astar::Vec next = p + d;
          if ((unsigned int)next[0] >= (unsigned int)map_info_.width ||
              (unsigned int)next[1] >= (unsigned int)map_info_.height)
            continue;
          float &gn = g[next];
          if (gn == FLT_MAX)
            continue;
          if (gn < cost_min)
          {
            open.push(Astar::PriorityVec(gn, gn, next));
            continue;
          }
          erase.push(Astar::PriorityVec(gn, gn, next));
        }
      }
    }
    if (open.size() == 0)
    {
      open.push(Astar::PriorityVec(-ec_rough_[0] * 0.5, -ec_rough_[0] * 0.5, e));
    }
    {
      Astar::Vec p;
      auto &g = cost_estim_cache_;
      p[2] = 0;
      for (p[0] = 0; p[0] < static_cast<int>(map_info_.width); p[0]++)
      {
        for (p[1] = 0; p[1] < static_cast<int>(map_info_.height); p[1]++)
        {
          auto &gp = g[p];
          if (gp > rough_cost_max_)
          {
            open.push(Astar::PriorityVec(gp, gp, p));
          }
        }
      }
    }

    fillCostmap(open, g, s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache updated (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    publishCostmap();
  }
  void cbMap(const costmap_cspace::CSpace3D::ConstPtr &msg)
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
          1.0f / max_vel_,
          1.0f / max_vel_,
          1.0f * cc_.weight_ang_vel_ / max_ang_vel_
        };

    ec_ = Astar::Vecf(ec_val);
    ec_val[2] = 0;
    ec_rough_ = Astar::Vecf(ec_val);

    if (map_info_.linear_resolution != msg->info.linear_resolution ||
        map_info_.angular_resolution != msg->info.angular_resolution)
    {
      Astar::Vec d;
      range_ = static_cast<int>(search_range_ / msg->info.linear_resolution);

      search_list_.clear();
      for (d[0] = -range_; d[0] <= range_; d[0]++)
      {
        for (d[1] = -range_; d[1] <= range_; d[1]++)
        {
          if (d.sqlen() > range_ * range_)
            continue;
          for (d[2] = 0; d[2] < static_cast<int>(msg->info.angle); d[2]++)
          {
            search_list_.push_back(d);
          }
        }
      }
      search_list_rough_.clear();
      for (d[0] = -range_; d[0] <= range_; d[0]++)
      {
        for (d[1] = -range_; d[1] <= range_; d[1]++)
        {
          if (d.sqlen() > range_ * range_)
            continue;
          d[2] = 0;
          search_list_rough_.push_back(d);
        }
      }
      ROS_DEBUG("Search list updated (range: ang %d, lin %d) %d",
                msg->info.angle, range_, static_cast<int>(search_list_.size()));

      rotgm_.resize(msg->info.angle);
      for (int i = 0; i < static_cast<int>(msg->info.angle); i++)
      {
        const int size[3] =
            {
              range_ * 2 + 1,
              range_ * 2 + 1,
              static_cast<int>(msg->info.angle)
            };
        auto &r = rotgm_[i];
        r.reset(Astar::Vec(size));

        Astar::Vec d;

        for (d[0] = 0; d[0] <= range_ * 2; d[0]++)
        {
          for (d[1] = 0; d[1] <= range_ * 2; d[1]++)
          {
            for (d[2] = 0; d[2] < static_cast<int>(msg->info.angle); d[2]++)
            {
              const float val[3] =
                  {
                    (d[0] - range_) * msg->info.linear_resolution,
                    (d[1] - range_) * msg->info.linear_resolution,
                    d[2] * msg->info.angular_resolution
                  };
              auto v = Astar::Vecf(val);
              rotate(v, -i * msg->info.angular_resolution);
              r[d] = v;
            }
          }
        }
      }
      ROS_DEBUG("Rotation cache generated");
    }
    map_info_ = msg->info;
    map_header_ = msg->header;

    resolution_[0] = 1.0 / map_info_.linear_resolution;
    resolution_[1] = 1.0 / map_info_.linear_resolution;
    resolution_[2] = 1.0 / map_info_.angular_resolution;

    hist_ignore_range_ = lroundf(hist_ignore_range_f_ / map_info_.linear_resolution);
    hist_ignore_range_max_ = lroundf(hist_ignore_range_max_f_ / map_info_.linear_resolution);
    local_range_ = lroundf(local_range_f_ / map_info_.linear_resolution);
    longcut_range_ = lroundf(longcut_range_f_ / map_info_.linear_resolution);
    esc_range_ = lroundf(esc_range_f_ / map_info_.linear_resolution);
    esc_angle_ = map_info_.angle / 8;
    goal_tolerance_lin_ = lroundf(goal_tolerance_lin_f_ / map_info_.linear_resolution);
    goal_tolerance_ang_ = lroundf(goal_tolerance_ang_f_ / map_info_.angular_resolution);

    int size[3] =
        {
          static_cast<int>(map_info_.width),
          static_cast<int>(map_info_.height),
          static_cast<int>(map_info_.angle)
        };
    as_.reset(Astar::Vec(size));
    cm_.reset(Astar::Vec(size));
    size[2] = 1;
    cost_estim_cache_.reset(Astar::Vec(size));
    cm_rough_.reset(Astar::Vec(size));
    cm_hyst_.reset(Astar::Vec(size));
    cm_hist_.reset(Astar::Vec(size));

    Astar::Vec p;
    for (p[0] = 0; p[0] < static_cast<int>(map_info_.width); p[0]++)
    {
      for (p[1] = 0; p[1] < static_cast<int>(map_info_.height); p[1]++)
      {
        int cost_min = 100;
        for (p[2] = 0; p[2] < static_cast<int>(map_info_.angle); p[2]++)
        {
          const size_t addr = ((p[2] * size[1]) + p[1]) * size[0] + p[0];
          char c = msg->data[addr];
          if (c < 0)
            c = unknown_cost_;
          cm_[p] = c;
          if (c < cost_min)
            cost_min = c;
        }
        p[2] = 0;
        cm_rough_[p] = cost_min;
      }
    }
    ROS_DEBUG("Map copied");
    cm_hyst_.clear(0);

    has_map_ = true;

    cm_rough_base_ = cm_rough_;
    cm_base_ = cm_;
    cm_hist_.clear(0);

    updateGoal();
  }

public:
  Planner3d()
    : nh_("~")
  {
    subMap = nh_.subscribe("costmap", 1, &Planner3d::cbMap, this);
    subMapUpdate = nh_.subscribe("costmap_update", 1, &Planner3d::cbMapUpdate, this);
    subGoal = nh_.subscribe("goal", 1, &Planner3d::cbGoal, this);
    pubPath = nh_.advertise<nav_msgs::Path>("path", 1, true);
    pubDebug = nh_.advertise<sensor_msgs::PointCloud>("debug", 1, true);
    pubHist = nh_.advertise<sensor_msgs::PointCloud>("remembered", 1, true);
    pubStart = nh_.advertise<geometry_msgs::PoseStamped>("path_start", 1, true);
    pub_end_ = nh_.advertise<geometry_msgs::PoseStamped>("path_end", 1, true);
    pub_status_ = nh_.advertise<planner_cspace::PlannerStatus>("status", 1, true);
    srs_forget_ = nh_.advertiseService("forget", &Planner3d::cbForget, this);

    nh_.param_cast("freq", freq_, 4.0f);
    nh_.param_cast("freq_min", freq_min_, 2.0f);
    nh_.param_cast("search_range", search_range_, 0.4f);

    nh_.param_cast("max_vel", max_vel_, 0.3f);
    nh_.param_cast("max_ang_vel", max_ang_vel_, 0.6f);

    nh_.param_cast("weight_decel", cc_.weight_decel_, 50.0f);
    nh_.param_cast("weight_backward", cc_.weight_backward_, 0.9f);
    nh_.param_cast("weight_ang_vel", cc_.weight_ang_vel_, 1.0f);
    nh_.param_cast("weight_costmap", cc_.weight_costmap_, 50.0f);
    nh_.param_cast("cost_in_place_turn", cc_.in_place_turn_, 30.0f);
    nh_.param_cast("hysteresis_max_dist", cc_.hysteresis_max_dist_, 0.3f);
    nh_.param_cast("weight_hysteresis", cc_.weight_hysteresis_, 5.0f);

    nh_.param("goal_tolerance_lin", goal_tolerance_lin_f_, 0.05);
    nh_.param("goal_tolerance_ang", goal_tolerance_ang_f_, 0.1);
    nh_.param("goal_tolerance_ang_finish", goal_tolerance_ang_finish_, 0.05);

    nh_.param("unknown_cost", unknown_cost_, 100);
    nh_.param("hist_cnt_max", hist_cnt_max_, 20);
    nh_.param("hist_cnt_thres", hist_cnt_thres_, 19);
    nh_.param("hist_cost", hist_cost_, 90);
    nh_.param("hist_ignore_range", hist_ignore_range_f_, 0.6);
    nh_.param("hist_ignore_range_max", hist_ignore_range_max_f_, 1.25);
    nh_.param("remember_updates", remember_updates_, false);

    nh_.param("local_range", local_range_f_, 2.5);
    nh_.param("longcut_range", longcut_range_f_, 0.0);
    nh_.param("esc_range", esc_range_f_, 0.25);

    nh_.param_cast("sw_wait", sw_wait_, 2.0f);
    nh_.param("find_best", find_best_, true);

    nh_.param("pos_jump", pos_jump_, 1.0);
    nh_.param("yaw_jump", yaw_jump_, 1.5);

    nh_.param("force_goal_orientation", force_goal_orientation_, true);

    nh_.param("temporary_escape", temporary_escape_, true);

    nh_.param("fast_map_update", fast_map_update_, false);
    if (fast_map_update_)
    {
      ROS_WARN("Planner3d: Experimental fast_map_update is enabled. ");
    }
    std::string DebugMode;
    nh_.param("DebugMode", DebugMode, std::string("cost_estim"));
    if (DebugMode == "hyst")
      debug_out_ = DEBUG_HYSTERESIS;
    else if (DebugMode == "hist")
      debug_out_ = DEBUG_HISTORY;
    else if (DebugMode == "cost_estim")
      debug_out_ = DEBUG_COST_ESTIM;

    int queue_size_limit;
    nh_.param("queue_size_limit", queue_size_limit, 0);
    as_.setQueueSizeLimit(queue_size_limit);

    status_.status = planner_cspace::PlannerStatus::DONE;

    has_map_ = false;
    has_goal_ = false;
    has_start_ = false;
    goal_updated_ = false;

    escaping_ = false;
  }
  void spin()
  {
    ros::Rate wait(freq_);
    ROS_DEBUG("Initialized");

    geometry_msgs::PoseStamped start_prev;
    start_prev.pose.orientation.w = 1.0;

    while (ros::ok())
    {
      wait.sleep();
      ros::spinOnce();

      if (has_map_)
      {
        start_.header.frame_id = "base_link";
        start_.header.stamp = ros::Time(0);
        start_.pose.orientation.x = 0.0;
        start_.pose.orientation.y = 0.0;
        start_.pose.orientation.z = 0.0;
        start_.pose.orientation.w = 1.0;
        start_.pose.position.x = 0;
        start_.pose.position.y = 0;
        start_.pose.position.z = 0;
        try
        {
          tfl_.waitForTransform(map_header_.frame_id, "base_link",
                                map_header_.stamp, ros::Duration(0.1));
          tfl_.transformPose(map_header_.frame_id, start_, start_);
        }
        catch (tf::TransformException &e)
        {
          // ROS_INFO("planner_cspace: Transform failed %s", e.what());
          continue;
        }
        {
          float x_diff = start_.pose.position.x - start_prev.pose.position.x;
          float y_diff = start_.pose.position.y - start_prev.pose.position.y;
          float yaw_diff = tf::getYaw(start_.pose.orientation) - tf::getYaw(start_prev.pose.orientation);
          if (yaw_diff > M_PI)
            yaw_diff -= M_PI * 2;

          if (powf(x_diff, 2.0) + powf(y_diff, 2.0) > powf(pos_jump_, 2.0) ||
              fabs(yaw_diff) > yaw_jump_)
          {
            ROS_ERROR("Position jumped, history cleared");
            cm_hist_.clear(0);
          }
          start_prev = start_;
        }

        has_start_ = true;
        if (!goal_updated_ && has_goal_)
          updateGoal();
      }

      if (has_map_ && has_goal_ && has_start_)
      {
        if (status_.status == planner_cspace::PlannerStatus::FINISHING)
        {
          float yaw_s = tf::getYaw(start_.pose.orientation);
          float yaw_g;
          if (force_goal_orientation_)
            yaw_g = tf::getYaw(goal_raw_.pose.orientation);
          else
            yaw_g = tf::getYaw(goal_.pose.orientation);

          float yaw_diff = yaw_s - yaw_g;
          if (yaw_diff > M_PI)
            yaw_diff -= M_PI * 2.0;
          else if (yaw_diff < -M_PI)
            yaw_diff += M_PI * 2.0;
          if (fabs(yaw_diff) < goal_tolerance_ang_finish_)
          {
            status_.status = planner_cspace::PlannerStatus::DONE;
            has_goal_ = false;
            ROS_INFO("Path plan finished");
          }
        }
        else
        {
          if (escaping_)
            status_.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
          else
            status_.error = planner_cspace::PlannerStatus::GOING_WELL;

          nav_msgs::Path path;
          makePlan(start_.pose, goal_.pose, path, true);
          pubPath.publish(path);

          if (switchDetect(path))
          {
            ROS_INFO("Will have switch back");
            if (sw_wait_ > 0.0)
            {
              ros::Duration(sw_wait_).sleep();
            }
          }
        }
      }
      else if (!has_goal_)
      {
        nav_msgs::Path path;
        path.header = map_header_;
        path.header.stamp = ros::Time::now();
        pubPath.publish(path);
      }
      pub_status_.publish(status_);
    }
  }

private:
  void grid2Metric(
      const int x, const int y, const int yaw,
      float &gx, float &gy, float &gyaw)
  {
    gx = x * map_info_.linear_resolution + map_info_.origin.position.x;
    gy = y * map_info_.linear_resolution + map_info_.origin.position.y;
    gyaw = yaw * map_info_.angular_resolution;
  }
  void grid2Metric(const std::list<Astar::Vec> &path_grid,
                   nav_msgs::Path &path, const Astar::Vec &v_start)
  {
    path.header = map_header_;
    path.header.stamp = ros::Time::now();

    // static int cnt = 0;
    // cnt ++;
    float x_ = 0, y_ = 0, yaw_ = 0;
    Astar::Vec p_;
    bool init = false;
    for (auto &p : path_grid)
    {
      float x, y, yaw;
      grid2Metric(p[0], p[1], p[2], x, y, yaw);
      geometry_msgs::PoseStamped ps;
      ps.header = path.header;

      // printf("%d %d %d  %f\n", p[0], p[1], p[2], as_.g[p]);
      if (init)
      {
        auto ds = v_start - p;
        auto d = p - p_;
        float diff_val[3] =
            {
              d[0] * map_info_.linear_resolution,
              d[1] * map_info_.linear_resolution,
              p[2] * map_info_.angular_resolution
            };
        Astar::Vecf motion_(diff_val);
        rotate(motion_, -p_[2] * map_info_.angular_resolution);

        float inter = 0.1 / d.len();

        const Astar::Vecf motion = motion_;
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
                 ds.sqlen() > local_range_ * local_range_)
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
  void metric2Grid(
      int &x, int &y, int &yaw,
      const float gx, const float gy, const float gyaw)
  {
    x = lroundf((gx - map_info_.origin.position.x) / map_info_.linear_resolution);
    y = lroundf((gy - map_info_.origin.position.y) / map_info_.linear_resolution);
    yaw = lroundf(gyaw / map_info_.angular_resolution);
  }
  bool makePlan(const geometry_msgs::Pose &gs, const geometry_msgs::Pose &ge,
                nav_msgs::Path &path, bool hyst)
  {
    Astar::Vec s, e;
    metric2Grid(s[0], s[1], s[2],
                gs.position.x, gs.position.y, tf::getYaw(gs.orientation));
    s.cycle_unsigned(s[2], map_info_.angle);
    metric2Grid(e[0], e[1], e[2],
                ge.position.x, ge.position.y, tf::getYaw(ge.orientation));
    e.cycle_unsigned(e[2], map_info_.angle);

    geometry_msgs::PoseStamped p;
    p.header = map_header_;
    float x, y, yaw;
    grid2Metric(e[0], e[1], e[2], x, y, yaw);
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    p.pose.position.x = x;
    p.pose.position.y = y;
    pub_end_.publish(p);

    if (cm_[s] == 100)
    {
      if (!searchAvailablePos(s, esc_range_, esc_angle_))
      {
        ROS_WARN("Oops! You are in Rock!");
        status_.error = planner_cspace::PlannerStatus::IN_ROCK;
        return false;
      }
      ROS_INFO("Start moved");
    }
    auto s_rough = s;
    s_rough[2] = 0;

    if (cost_estim_cache_[s_rough] == FLT_MAX)
    {
      status_.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      ROS_WARN("Goal unreachable. History cleared.");
      cm_hist_.clear(0);
      if (!escaping_ && temporary_escape_)
      {
        e = s;
        if (searchAvailablePos(e, esc_range_, esc_angle_, 50, esc_range_ / 2))
        {
          escaping_ = true;
          ROS_INFO("Temporary goal (%d, %d, %d)",
                   e[0], e[1], e[2]);
          float x, y, yaw;
          grid2Metric(e[0], e[1], e[2], x, y, yaw);
          goal_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          goal_.pose.position.x = x;
          goal_.pose.position.y = y;

          updateGoal();
          return false;
        }
      }
      return false;
    }

    grid2Metric(s[0], s[1], s[2], x, y, yaw);
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    p.pose.position.x = x;
    p.pose.position.y = y;
    pubStart.publish(p);

    auto diff = s - e;
    diff.cycle(diff[2], map_info_.angle);
    if (diff.sqlen() <= goal_tolerance_lin_ * goal_tolerance_lin_ &&
        abs(diff[2]) <= goal_tolerance_ang_)
    {
      path.header = map_header_;
      path.header.stamp = ros::Time::now();
      path.poses.resize(1);
      path.poses[0].header = path.header;
      if (force_goal_orientation_)
        path.poses[0].pose = goal_raw_.pose;
      else
        path.poses[0].pose = ge;

      if (escaping_)
      {
        goal_ = goal_raw_;
        escaping_ = false;
        updateGoal();
        ROS_INFO("Escaped");
      }
      else
      {
        status_.status = planner_cspace::PlannerStatus::FINISHING;
        ROS_INFO("Path plan finishing");
      }
      return true;
    }

    auto range_limit = cost_estim_cache_[s_rough] - (local_range_ + range_) * ec_[0];
    angle_resolution_aspect_ = 1.0 / tanf(map_info_.angular_resolution);

    // ROS_INFO("Planning from (%d, %d, %d) to (%d, %d, %d)",
    //   s[0], s[1], s[2], e[0], e[1], e[2]);
    std::list<Astar::Vec> path_grid;
    // const auto ts = boost::chrono::high_resolution_clock::now();
    if (!as_.search(s, e, path_grid,
                    std::bind(&Planner3d::cbCost,
                              this, std::placeholders::_1, std::placeholders::_2,
                              std::placeholders::_3, std::placeholders::_4, hyst),
                    std::bind(&Planner3d::cbCostEstim,
                              this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&Planner3d::cbSearch,
                              this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3),
                    std::bind(&Planner3d::cbProgress,
                              this, std::placeholders::_1),
                    range_limit,
                    1.0f / freq_min_,
                    true))
    {
      ROS_WARN("Path plan failed (goal unreachable)");
      status_.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      if (!find_best_)
        return false;
    }
    // const auto tnow = boost::chrono::high_resolution_clock::now();
    // ROS_INFO("Path found (%0.3f sec.)",
    //   boost::chrono::duration<float>(tnow - ts).count());

    grid2Metric(path_grid, path, s);

    if (hyst)
    {
      std::unordered_map<Astar::Vec, bool, Astar::Vec> path_points;
      float max_dist = cc_.hysteresis_max_dist_ / map_info_.linear_resolution;
      int path_range = range_ + max_dist + 1;
      for (auto &p : path_grid)
      {
        Astar::Vec d;
        for (d[0] = -path_range; d[0] <= path_range; d[0]++)
        {
          for (d[1] = -path_range; d[1] <= path_range; d[1]++)
          {
            Astar::Vec point = p + d;
            point[2] = 0;
            if ((unsigned int)point[0] >= (unsigned int)map_info_.width ||
                (unsigned int)point[1] >= (unsigned int)map_info_.height)
              continue;
            path_points[point] = true;
          }
        }
      }

      cm_hyst_.clear(100);
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
        cm_hyst_[p] = d_min * 100.0 / max_dist;
      }
      // const auto tnow = boost::chrono::high_resolution_clock::now();
      // ROS_INFO("Hysteresis map generated (%0.3f sec.)",
      //   boost::chrono::duration<float>(tnow - ts).count());
      publishCostmap();
    }

    return true;
  }
  std::vector<Astar::Vec> &cbSearch(
      const Astar::Vec &p,
      const Astar::Vec &s, const Astar::Vec &e)
  {
    const auto ds = s - p;
    rot_cache_ = &rotgm_[p[2]];

    if (ds.sqlen() < local_range_ * local_range_)
    {
      rough_ = false;
      euclid_cost_coef_ = ec_;
      return search_list_;
    }
    rough_ = true;
    euclid_cost_coef_ = ec_rough_;
    return search_list_rough_;
  }
  bool cbProgress(const std::list<Astar::Vec> &path_grid)
  {
    nav_msgs::Path path;
    path.header = map_header_;
    path.header.stamp = ros::Time::now();
    // grid2Metric(path_grid, path);
    pubPath.publish(path);
    ROS_WARN("Search timed out");
    return true;
  }
  void rotate(Astar::Vecf &v, const float &ang)
  {
    const Astar::Vecf tmp = v;
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
  float cbCostEstim(const Astar::Vec &s, const Astar::Vec &e)
  {
    auto s2 = s;
    s2[2] = 0;
    auto cost = cost_estim_cache_[s2];
    if (cost == FLT_MAX)
      return FLT_MAX;
    if (!rough_)
    {
      if (s2[2] > static_cast<int>(map_info_.angle) / 2)
        s2[2] -= map_info_.angle;
      cost += ec_rough_[2] * fabs(s[2]);
    }
    return cost;
  }
  bool switchDetect(const nav_msgs::Path &path)
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
  float cbCost(const Astar::Vec &s, Astar::Vec &e,
               const Astar::Vec &v_goal,
               const Astar::Vec &v_start,
               const bool hyst)
  {
    const Astar::Vec d = e - s;
    float cost = euclidCost(d);

    if (d[0] == 0 && d[1] == 0)
    {
      // In-place turn
      int sum = 0;
      int dir = 1;
      if (d[2] > static_cast<int>(map_info_.angle) / 2)
        dir = -1;
      Astar::Vec pos = s;
      for (int i = 0; i < abs(d[2]); i++)
      {
        pos[2] += dir;
        if (pos[2] < 0)
          pos[2] += map_info_.angle;
        else if (pos[2] >= static_cast<int>(map_info_.angle))
          pos[2] -= map_info_.angle;
        const auto c = cm_[pos];
        if (c > 99)
          return -1;
        sum += c;
      }

      float cost = sum * map_info_.angular_resolution * ec_[2] / ec_[0];
      return cc_.in_place_turn_ + cost;
    }

    Astar::Vec d2;
    d2[0] = d[0] + range_;
    d2[1] = d[1] + range_;
    d2[2] = e[2];
    const Astar::Vecf motion = (*rot_cache_)[d2];

    const Astar::Vecf motion_grid = motion * resolution_;
    // motion_grid[0] /= map_info_.linear_resolution;
    // motion_grid[1] /= map_info_.linear_resolution;
    // motion_grid[2] /= map_info_.angular_resolution;

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
      cost *= 1.0 + cc_.weight_backward_;
    }

    if (lroundf(motion_grid[2]) == 0)
    {
      float distf = d.len();

      if (motion_grid[0] == 0)
        return -1;  // side slip
      float aspect = motion[0] / motion[1];
      if (fabs(aspect) < angle_resolution_aspect_ * 2.0)
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
      Astar::Vec pos(v);
      for (int i = 0; i < dist; i++)
      {
        pos[0] = lroundf(v[0]);
        pos[1] = lroundf(v[1]);
        const auto c = cm_[pos];
        if (c > 99)
          return -1;
        if (hyst)
        {
          auto pos_rough = pos;
          pos_rough[2] = 0;
          sum_hyst += cm_hyst_[pos_rough];
        }
        sum += c;
        v[0] += dp[0];
        v[1] += dp[1];
      }
      cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / 100.0;
      cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / 100.0;
    }
    else
    {
      // Curve
      if (motion[0] * motion[1] * motion[2] < 0)
        return -1;
      if (d.sqlen() < 4 * 4)
        return -1;
      if (fabs(motion[1]) <= map_info_.linear_resolution * 0.5)
        return -1;

      const float r1 = motion[1] + motion[0] * cos_v / sin_v;
      float r2 = sqrtf(powf(motion[0], 2.0) + powf(motion[0] * cos_v / sin_v, 2.0));
      if (motion[0] * sin_v < 0)
        r2 = -r2;

      // curveture at the start_ pose and the end pose must be same
      if (fabs(r1 - r2) >= map_info_.linear_resolution * 1.5)
      {
        // Drifted
        return -1;
      }

      const float curv_radius = (r1 + r2) / 2;

      float vel = max_vel_;
      float ang_vel = cos_v * vel / (cos_v * motion[0] + sin_v * motion[1]);
      if (fabs(ang_vel) > max_ang_vel_)
      {
        ang_vel = signf(ang_vel) * max_ang_vel_;
        vel = fabs(curv_radius) * max_ang_vel_;

        // Curve deceleration penalty
        cost += dist * fabs(vel / max_vel_) * cc_.weight_decel_;
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
        if (dp[2] < -map_info_.angle / 2)
          dp[2] += map_info_.angle;
        else if (dp[2] >= map_info_.angle / 2)
          dp[2] -= map_info_.angle;
        dp[2] /= dist;
        Astar::Vec pos(v);
        for (int i = 0; i < dist; i++)
        {
          pos[0] = lroundf(v[0]);
          pos[1] = lroundf(v[1]);
          pos[2] = lroundf(v[2]);
          if (pos[2] < 0)
            pos[2] += map_info_.angle;
          else if (pos[2] >= static_cast<int>(map_info_.angle))
            pos[2] -= map_info_.angle;
          const auto c = cm_[pos];
          if (c > 99)
            return -1;
          sum += c;
          if (hyst)
          {
            auto pos_rough = pos;
            pos_rough[2] = 0;
            sum_hyst += cm_hyst_[pos_rough];
          }
          v[0] += dp[0];
          v[1] += dp[1];
          v[2] += dp[2];
        }
        cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / 100.0;
        cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / 100.0;
      }
    }

    return cost;
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "planner_3d");

  Planner3d jy;
  jy.spin();

  return 0;
}
