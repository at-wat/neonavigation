/*
 * Copyright (c) 2014-2019, the neonavigation authors
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
#include <limits>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/Empty.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/converter.h>

#include <ros/console.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <costmap_cspace/node_handle_float.h>
#include <neonavigation_common/compatibility.h>

#include <planner_cspace/bbf.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/planner_3d/grid_metric_converter.h>
#include <planner_cspace/planner_3d/jump_detector.h>
#include <planner_cspace/planner_3d/motion_cache.h>
#include <planner_cspace/planner_3d/rotation_cache.h>

#include <omp.h>

class Planner3dNode
{
public:
  using Astar = GridAstar<3, 2>;

protected:
  using Planner3DActionServer = actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;

  ros::NodeHandle nh_;
  ros::NodeHandle_f pnh_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_update_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_velocity_;
  ros::Publisher pub_path_poses_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_hist_;
  ros::Publisher pub_start_;
  ros::Publisher pub_end_;
  ros::Publisher pub_status_;
  ros::ServiceServer srs_forget_;
  ros::ServiceServer srs_make_plan_;

  std::shared_ptr<Planner3DActionServer> act_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;
  Astar::Gridmap<bbf::BinaryBayesFilter, 0x20> cm_hist_bbf_;
  Astar::Gridmap<char, 0x40> cm_hist_;
  Astar::Gridmap<char, 0x80> cm_rough_;
  Astar::Gridmap<char, 0x40> cm_base_;
  Astar::Gridmap<char, 0x80> cm_rough_base_;
  Astar::Gridmap<char, 0x80> cm_hyst_;
  Astar::Gridmap<float> cost_estim_cache_;

  Astar::Vecf euclid_cost_coef_;

  float euclidCost(const Astar::Vec& v, const Astar::Vecf coef) const
  {
    Astar::Vec vc = v;
    float cost = 0;
    for (int i = 0; i < as_.getNoncyclic(); i++)
    {
      cost += powf(coef[i] * vc[i], 2.0);
    }
    cost = sqrtf(cost);
    vc.cycle(cm_.size());
    for (int i = as_.getNoncyclic(); i < as_.getDim(); i++)
    {
      cost += fabs(coef[i] * vc[i]);
    }
    return cost;
  }
  float euclidCost(const Astar::Vec& v) const
  {
    return euclidCost(v, euclid_cost_coef_);
  }

  RotationCache rot_cache_;

  costmap_cspace_msgs::MapMetaData3D map_info_;
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
  int tolerance_range_;
  int tolerance_angle_;
  double tolerance_range_f_;
  double tolerance_angle_f_;
  int unknown_cost_;
  bool overwrite_cost_;
  bool has_map_;
  bool has_goal_;
  bool has_start_;
  bool goal_updated_;
  bool remember_updates_;
  bool fast_map_update_;
  std::vector<Astar::Vec> search_list_;
  std::vector<Astar::Vec> search_list_rough_;
  double hist_ignore_range_f_;
  int hist_ignore_range_;
  double hist_ignore_range_max_f_;
  int hist_ignore_range_max_;
  bool temporary_escape_;
  float remember_hit_odds_;
  float remember_miss_odds_;
  bool use_path_with_velocity_;
  float min_curve_raduis_;

  JumpDetector jump_;
  std::string robot_frame_;

  int max_retry_num_;

  int num_task_;

  // Cost weights
  class CostCoeff
  {
  public:
    float weight_decel_;
    float weight_backward_;
    float weight_ang_vel_;
    float weight_costmap_;
    float weight_costmap_turn_;
    float weight_remembered_;
    float weight_hysteresis_;
    float in_place_turn_;
    float hysteresis_max_dist_;
    float hysteresis_expand_;
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

  planner_cspace_msgs::PlannerStatus status_;

  bool find_best_;
  float sw_wait_;

  float rough_cost_max_;
  bool rough_;

  bool force_goal_orientation_;

  bool escaping_;

  int cnt_stuck_;

  MotionCache motion_cache_;
  MotionCache motion_cache_linear_;

  diagnostic_updater::Updater diag_updater_;
  ros::Duration costmap_watchdog_;
  ros::Time last_costmap_;

  bool cbForget(std_srvs::EmptyRequest& req,
                std_srvs::EmptyResponse& res)
  {
    ROS_WARN("Forgetting remembered costmap.");
    if (has_map_)
      cm_hist_bbf_.clear(bbf::BinaryBayesFilter(bbf::MIN_ODDS));

    return true;
  }
  bool cbMakePlan(nav_msgs::GetPlan::Request& req,
                  nav_msgs::GetPlan::Response& res)
  {
    if (!has_map_)
      return false;

    Astar::Vec s, e;
    grid_metric_converter::metric2Grid(
        map_info_, s[0], s[1], s[2],
        req.start.pose.position.x, req.start.pose.position.y, tf2::getYaw(req.start.pose.orientation));
    s[2] = 0;
    grid_metric_converter::metric2Grid(
        map_info_, e[0], e[1], e[2],
        req.goal.pose.position.x, req.goal.pose.position.y, tf2::getYaw(req.goal.pose.orientation));
    e[2] = 0;

    if (!(cm_rough_.validate(s, range_) && cm_rough_.validate(e, range_)))
    {
      ROS_ERROR("Given start or goal is not on the map.");
      return false;
    }
    else if (cm_rough_[s] == 100 || cm_rough_[e] == 100)
    {
      ROS_ERROR("Given start or goal is in Rock.");
      return false;
    }

    const Astar::Vecf euclid_cost_coef = ec_rough_;

    const auto cb_cost = [this, &euclid_cost_coef](
        const Astar::Vec& s, Astar::Vec& e,
        const Astar::Vec& v_goal, const Astar::Vec& v_start,
        const bool hyst) -> float
    {
      const Astar::Vec d = e - s;
      float cost = euclidCost(d, euclid_cost_coef);

      int sum = 0;
      const auto cache_page = motion_cache_linear_.find(0, d);
      if (cache_page == motion_cache_linear_.end(0))
        return -1;
      const int num = cache_page->second.getMotion().size();
      for (const auto& pos_diff : cache_page->second.getMotion())
      {
        const Astar::Vec pos(s[0] + pos_diff[0], s[1] + pos_diff[1], 0);
        const auto c = cm_rough_[pos];
        if (c > 99)
          return -1;
        sum += c;
      }
      const float distf = cache_page->second.getDistance();
      cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / (100.0 * num);

      return cost;
    };
    const auto cb_cost_estim = [this, &euclid_cost_coef](
        const Astar::Vec& s, const Astar::Vec& e)
    {
      const Astar::Vec d = e - s;
      const float cost = euclidCost(d, euclid_cost_coef);

      return cost;
    };
    const auto cb_search = [this](
        const Astar::Vec& p,
        const Astar::Vec& s, const Astar::Vec& e) -> std::vector<Astar::Vec>&
    {
      return search_list_rough_;
    };
    const auto cb_progress = [](const std::list<Astar::Vec>& path_grid)
    {
      return true;
    };

    const auto ts = boost::chrono::high_resolution_clock::now();
    // ROS_INFO("Planning from (%d, %d, %d) to (%d, %d, %d)",
    //   s[0], s[1], s[2], e[0], e[1], e[2]);
    std::list<Astar::Vec> path_grid;
    if (!as_.search(s, e, path_grid,
                    std::bind(cb_cost,
                              std::placeholders::_1, std::placeholders::_2,
                              std::placeholders::_3, std::placeholders::_4, false),
                    cb_cost_estim, cb_search, cb_progress,
                    0,
                    1.0f / freq_min_,
                    find_best_))
    {
      ROS_WARN("Path plan failed (goal unreachable)");
      return false;
    }
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_INFO("Path found (%0.4f sec.)",
             boost::chrono::duration<float>(tnow - ts).count());

    nav_msgs::Path path;
    path.header = map_header_;
    path.header.stamp = ros::Time::now();

    grid_metric_converter::grid2MetricPath(map_info_, 0.0, path_grid, path, s);

    res.plan.header = map_header_;
    res.plan.poses.resize(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      res.plan.poses[i] = path.poses[i];
    }

    return true;
  }

  void cbGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    if (act_->isActive())
    {
      ROS_ERROR("Setting new goal is ignored since planner_3d is proceeding the action.");
      return;
    }
    setGoal(*msg);
  }
  void cbPreempt()
  {
    ROS_WARN("Preempting the current goal.");
    act_->setPreempted(move_base_msgs::MoveBaseResult(), "Preempted.");
    has_goal_ = false;
    status_.status = planner_cspace_msgs::PlannerStatus::DONE;
  }
  bool setGoal(const geometry_msgs::PoseStamped& msg)
  {
    goal_raw_ = goal_ = msg;

    const double len2 =
        goal_.pose.orientation.x * goal_.pose.orientation.x +
        goal_.pose.orientation.y * goal_.pose.orientation.y +
        goal_.pose.orientation.z * goal_.pose.orientation.z +
        goal_.pose.orientation.w * goal_.pose.orientation.w;
    if (fabs(len2 - 1.0) < 0.1)
    {
      escaping_ = false;
      has_goal_ = true;
      cnt_stuck_ = 0;
      if (!updateGoal())
      {
        has_goal_ = false;
        return false;
      }
      status_.status = planner_cspace_msgs::PlannerStatus::DOING;
      pub_status_.publish(status_);
      diag_updater_.update();
    }
    else
    {
      has_goal_ = false;
      act_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal cleared.");
    }
    return true;
  }
  void fillCostmap(
      reservable_priority_queue<Astar::PriorityVec>& open,
      Astar::Gridmap<float>& g,
      const Astar::Vec& s, const Astar::Vec& e)
  {
    const Astar::Vec s_rough(s[0], s[1], 0);

    while (true)
    {
      if (open.size() < 1)
        break;

      std::vector<Astar::PriorityVec> centers;
      for (size_t i = 0; i < static_cast<size_t>(num_task_); ++i)
      {
        if (open.size() < 1)
          break;
        const auto center = open.top();
        open.pop();
        if (center.p_raw_ > g[center.v_])
          continue;
        if (center.p_raw_ - ec_rough_[0] * (range_ + local_range_ + longcut_range_) > g[s_rough])
          continue;
        centers.push_back(center);
      }
#pragma omp parallel for schedule(static)
      for (auto it = centers.begin(); it < centers.end(); ++it)
      {
        const Astar::Vec p = it->v_;
        const float c = it->p_raw_;

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
            auto& gnext = g[next];
            if (gnext < 0)
              continue;

            float cost = 0;

            {
              float sum = 0, sum_hist = 0;
              const float grid_to_len = d.gridToLenFactor();
              const int dist = d.len();
              const float dpx = static_cast<float>(d[0]) / dist;
              const float dpy = static_cast<float>(d[1]) / dist;
              Astar::Vec pos(static_cast<int>(p[0]), static_cast<int>(p[1]), 0);
              int i = 0;
              for (; i < dist; i++)
              {
                const char c = cm_rough_[pos];
                if (c > 99)
                  break;
                sum += c;

                sum_hist += cm_hist_[pos];
                pos[0] += dpx;
                pos[1] += dpy;
              }
              if (i != dist)
                continue;
              cost +=
                  (map_info_.linear_resolution * grid_to_len / 100.0) *
                  (sum * cc_.weight_costmap_ + sum_hist * cc_.weight_remembered_);

              if (cost < 0)
              {
                cost = 0;
                ROS_WARN_THROTTLE(1.0, "Negative cost value is detected. Limited to zero.");
              }
            }
            cost += euclidCost(d, ec_rough_);

            const auto gp = c + cost;
            if (gnext > gp)
            {
              gnext = gp;
#pragma omp critical
              {
                open.push(Astar::PriorityVec(gp, gp, next));
              }
            }
          }
        }
      }
    }
    rough_cost_max_ = g[s_rough] + ec_rough_[0] * (range_ + local_range_);
  }
  bool searchAvailablePos(Astar::Vec& s, const int xy_range, const int angle_range,
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
          s2.cycleUnsigned(map_info_.angle);
          if (cm_[s2] >= cost_acceptable)
            continue;
          const auto cost = euclidCost(d, ec_);
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
    s.cycleUnsigned(map_info_.angle);
    ROS_DEBUG("    (%d,%d,%d)", s[0], s[1], s[2]);
    return true;
  }
  bool updateGoal(const bool goal_changed = true)
  {
    if (!has_map_ || !has_goal_ || !has_start_)
    {
      ROS_ERROR("Goal received, however map/goal/start are not ready. (%d/%d/%d)",
                static_cast<int>(has_map_), static_cast<int>(has_goal_), static_cast<int>(has_start_));
      return true;
    }

    Astar::Vec s, e;
    grid_metric_converter::metric2Grid(
        map_info_, s[0], s[1], s[2],
        start_.pose.position.x, start_.pose.position.y,
        tf2::getYaw(start_.pose.orientation));
    s.cycleUnsigned(map_info_.angle);
    grid_metric_converter::metric2Grid(
        map_info_, e[0], e[1], e[2],
        goal_.pose.position.x, goal_.pose.position.y,
        tf2::getYaw(goal_.pose.orientation));
    e.cycleUnsigned(map_info_.angle);
    if (goal_changed)
    {
      ROS_INFO(
          "New goal received (%d, %d, %d)",
          e[0], e[1], e[2]);
    }

    if (!cm_.validate(e, range_))
    {
      ROS_ERROR("Given goal is not on the map.");
      return false;
    }
    if (!cm_.validate(s, range_))
    {
      ROS_ERROR("You are on the edge of the world.");
      return false;
    }

    const auto ts = boost::chrono::high_resolution_clock::now();
    reservable_priority_queue<Astar::PriorityVec> open;

    cost_estim_cache_.clear(FLT_MAX);
    if (cm_[e] == 100)
    {
      if (!searchAvailablePos(e, tolerance_range_, tolerance_angle_))
      {
        ROS_WARN("Oops! Goal is in Rock!");
        ++cnt_stuck_;
        return true;
      }
      ROS_INFO("Goal moved (%d, %d, %d)",
               e[0], e[1], e[2]);
      float x, y, yaw;
      grid_metric_converter::grid2Metric(map_info_, e[0], e[1], e[2], x, y, yaw);
      goal_.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
      goal_.pose.position.x = x;
      goal_.pose.position.y = y;
    }
    if (cm_[s] == 100)
    {
      if (!searchAvailablePos(s, tolerance_range_, tolerance_angle_))
      {
        ROS_WARN("Oops! You are in Rock!");
        return true;
      }
    }

    e[2] = 0;
    cost_estim_cache_[e] = -ec_rough_[0] * 0.5;  // Decrement to reduce calculation error
    open.push(Astar::PriorityVec(cost_estim_cache_[e], cost_estim_cache_[e], e));
    fillCostmap(open, cost_estim_cache_, s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache generated (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    cost_estim_cache_[e] = 0;

    if (goal_changed)
      cm_hyst_.clear(0);

    publishCostmap();

    goal_updated_ = true;

    return true;
  }
  void publishCostmap()
  {
    if (pub_debug_.getNumSubscribers() == 0)
      return;

    sensor_msgs::PointCloud debug;
    debug.header = map_header_;
    debug.header.stamp = ros::Time::now();
    {
      Astar::Vec p;
      for (p[1] = 0; p[1] < cost_estim_cache_.size()[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cost_estim_cache_.size()[0]; p[0]++)
        {
          p[2] = 0;
          float x, y, yaw;
          grid_metric_converter::grid2Metric(map_info_, p[0], p[1], p[2], x, y, yaw);
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
              point.z = cm_hist_bbf_[p].getProbability();
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
    pub_debug_.publish(debug);
  }
  void publishEmptyPath()
  {
    nav_msgs::Path path;
    path.header.frame_id = robot_frame_;
    path.header.stamp = ros::Time::now();
    if (use_path_with_velocity_)
    {
      pub_path_velocity_.publish(
          trajectory_tracker_msgs::toPathWithVelocity(
              path, std::numeric_limits<double>::quiet_NaN()));
    }
    else
    {
      pub_path_.publish(path);
    }
  }

  void cbMapUpdate(const costmap_cspace_msgs::CSpace3DUpdate::ConstPtr& msg)
  {
    if (!has_map_)
      return;
    ROS_DEBUG("Map updated");

    const ros::Time now = ros::Time::now();
    last_costmap_ = now;

    cm_ = cm_base_;
    cm_rough_ = cm_rough_base_;
    if (remember_updates_)
    {
      if (pub_hist_.getNumSubscribers() > 0)
      {
        sensor_msgs::PointCloud pc;
        pc.header = map_header_;
        pc.header.stamp = now;
        Astar::Vec p;
        for (p[1] = 0; p[1] < cm_hist_bbf_.size()[1]; p[1]++)
        {
          for (p[0] = 0; p[0] < cm_hist_bbf_.size()[0]; p[0]++)
          {
            if (cm_hist_bbf_[p].get() > bbf::probabilityToOdds(0.1))
            {
              float x, y, yaw;
              grid_metric_converter::grid2Metric(map_info_, p[0], p[1], p[2], x, y, yaw);
              geometry_msgs::Point32 point;
              point.x = x;
              point.y = y;
              point.z = cm_hist_bbf_[p].getProbability();
              pc.points.push_back(point);
            }
          }
        }
        pub_hist_.publish(pc);
      }
      Astar::Vec p;
      cm_hist_.clear(0);
      for (p[1] = 0; p[1] < cm_hist_bbf_.size()[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cm_hist_bbf_.size()[0]; p[0]++)
        {
          p[2] = 0;
          cm_hist_[p] = lroundf(cm_hist_bbf_[p].getNormalizedProbability() * 100.0);
        }
      }
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
      const Astar::Vec gp_rough(gp[0], gp[1], 0);
      const int hist_ignore_range_sq = hist_ignore_range_ * hist_ignore_range_;
      const int hist_ignore_range_max_sq =
          hist_ignore_range_max_ * hist_ignore_range_max_;
      for (p[0] = 0; p[0] < static_cast<int>(msg->width); p[0]++)
      {
        for (p[1] = 0; p[1] < static_cast<int>(msg->height); p[1]++)
        {
          int cost_min = 100;
          for (p[2] = 0; p[2] < static_cast<int>(msg->angle); p[2]++)
          {
            const size_t addr = ((p[2] * msg->height) + p[1]) * msg->width + p[0];
            const char c = msg->data[addr];
            if (c < cost_min)
              cost_min = c;
          }
          p[2] = 0;
          if (cost_min > cm_rough_[gp_rough + p])
          {
            cm_rough_[gp_rough + p] = cost_min;
          }

          Astar::Vec pos = gp + p;
          pos[2] = 0;
          if (cost_min == 100)
          {
            const Astar::Vec p2 = p - center;
            const float sqlen = p2.sqlen();
            if (sqlen > hist_ignore_range_sq &&
                sqlen < hist_ignore_range_max_sq)
            {
              cm_hist_bbf_[pos].update(remember_hit_odds_);
            }
          }
          else if (cost_min >= 0)
          {
            const Astar::Vec p2 = p - center;
            const float sqlen = p2.sqlen();
            if (sqlen < hist_ignore_range_max_sq)
            {
              cm_hist_bbf_[pos].update(remember_miss_odds_);
            }
          }

          for (p[2] = 0; p[2] < static_cast<int>(msg->angle); p[2]++)
          {
            const size_t addr = ((p[2] * msg->height) + p[1]) * msg->width + p[0];
            const char c = msg->data[addr];
            if (overwrite_cost_)
            {
              if (c >= 0)
                cm_[gp + p] = c;
            }
            else
            {
              if (cm_[gp + p] < c)
                cm_[gp + p] = c;
            }
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
    grid_metric_converter::metric2Grid(
        map_info_, s[0], s[1], s[2],
        start_.pose.position.x, start_.pose.position.y,
        tf2::getYaw(start_.pose.orientation));
    s.cycleUnsigned(map_info_.angle);
    grid_metric_converter::metric2Grid(
        map_info_, e[0], e[1], e[2],
        goal_.pose.position.x, goal_.pose.position.y,
        tf2::getYaw(goal_.pose.orientation));
    e.cycleUnsigned(map_info_.angle);

    if (cm_[e] == 100)
    {
      updateGoal(false);
      return;
    }

    e[2] = 0;

    const auto ts = boost::chrono::high_resolution_clock::now();

    Astar::Vec p, p_cost_min;
    p[2] = 0;
    float cost_min = FLT_MAX;
    for (p[1] = static_cast<int>(msg->y); p[1] < static_cast<int>(msg->y + msg->height); p[1]++)
    {
      for (p[0] = static_cast<int>(msg->x); p[0] < static_cast<int>(msg->x + msg->width); p[0]++)
      {
        if (cost_min > cost_estim_cache_[p])
        {
          p_cost_min = p;
          cost_min = cost_estim_cache_[p];
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

      if (cost_estim_cache_[p] == FLT_MAX)
        continue;
      cost_estim_cache_[p] = FLT_MAX;

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
          const float& gn = cost_estim_cache_[next];
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
      p[2] = 0;
      for (p[0] = 0; p[0] < static_cast<int>(map_info_.width); p[0]++)
      {
        for (p[1] = 0; p[1] < static_cast<int>(map_info_.height); p[1]++)
        {
          const auto& gp = cost_estim_cache_[p];
          if (gp > rough_cost_max_)
          {
            open.push(Astar::PriorityVec(gp, gp, p));
          }
        }
      }
    }

    fillCostmap(open, cost_estim_cache_, s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache updated (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    publishCostmap();
  }
  void cbMap(const costmap_cspace_msgs::CSpace3D::ConstPtr& msg)
  {
    ROS_INFO("Map received");
    ROS_INFO(" linear_resolution %0.2f x (%dx%d) px", msg->info.linear_resolution,
             msg->info.width, msg->info.height);
    ROS_INFO(" angular_resolution %0.2f x %d px", msg->info.angular_resolution,
             msg->info.angle);
    ROS_INFO(" origin %0.3f m, %0.3f m, %0.3f rad",
             msg->info.origin.position.x,
             msg->info.origin.position.y,
             tf2::getYaw(msg->info.origin.orientation));

    // Stop robot motion until next planning step
    publishEmptyPath();

    const float ec_val[3] =
        {
          1.0f / max_vel_,
          1.0f / max_vel_,
          1.0f * cc_.weight_ang_vel_ / max_ang_vel_
        };
    ec_ = Astar::Vecf(ec_val[0], ec_val[1], ec_val[2]);
    ec_rough_ = Astar::Vecf(ec_val[0], ec_val[1], 0);

    if (map_info_.linear_resolution != msg->info.linear_resolution ||
        map_info_.angular_resolution != msg->info.angular_resolution)
    {
      map_info_ = msg->info;
      Astar::Vec d;
      range_ = static_cast<int>(search_range_ / map_info_.linear_resolution);

      costmap_cspace_msgs::MapMetaData3D map_info_linear(map_info_);
      map_info_linear.angle = 1;
      motion_cache_linear_.reset(
          map_info_linear.linear_resolution,
          map_info_linear.angular_resolution,
          range_,
          cm_rough_.getAddressor());
      motion_cache_.reset(
          map_info_.linear_resolution,
          map_info_.angular_resolution,
          range_,
          cm_.getAddressor());

      search_list_.clear();
      for (d[0] = -range_; d[0] <= range_; d[0]++)
      {
        for (d[1] = -range_; d[1] <= range_; d[1]++)
        {
          if (d.sqlen() > range_ * range_)
            continue;
          for (d[2] = 0; d[2] < static_cast<int>(map_info_.angle); d[2]++)
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
                map_info_.angle, range_, static_cast<int>(search_list_.size()));

      rot_cache_.reset(map_info_.linear_resolution, map_info_.angular_resolution, range_);
      ROS_DEBUG("Rotation cache generated");
    }
    else
    {
      map_info_ = msg->info;
    }
    map_header_ = msg->header;
    jump_.setMapFrame(map_header_.frame_id);

    resolution_[0] = 1.0 / map_info_.linear_resolution;
    resolution_[1] = 1.0 / map_info_.linear_resolution;
    resolution_[2] = 1.0 / map_info_.angular_resolution;

    hist_ignore_range_ = lroundf(hist_ignore_range_f_ / map_info_.linear_resolution);
    hist_ignore_range_max_ = lroundf(hist_ignore_range_max_f_ / map_info_.linear_resolution);
    local_range_ = lroundf(local_range_f_ / map_info_.linear_resolution);
    longcut_range_ = lroundf(longcut_range_f_ / map_info_.linear_resolution);
    esc_range_ = lroundf(esc_range_f_ / map_info_.linear_resolution);
    esc_angle_ = map_info_.angle / 8;
    tolerance_range_ = lroundf(tolerance_range_f_ / map_info_.linear_resolution);
    tolerance_angle_ = lroundf(tolerance_angle_f_ / map_info_.angular_resolution);
    goal_tolerance_lin_ = lroundf(goal_tolerance_lin_f_ / map_info_.linear_resolution);
    goal_tolerance_ang_ = lroundf(goal_tolerance_ang_f_ / map_info_.angular_resolution);

    const int size[3] =
        {
          static_cast<int>(map_info_.width),
          static_cast<int>(map_info_.height),
          static_cast<int>(map_info_.angle)
        };
    as_.reset(Astar::Vec(size[0], size[1], size[2]));
    cm_.reset(Astar::Vec(size[0], size[1], size[2]));
    cost_estim_cache_.reset(Astar::Vec(size[0], size[1], 1));
    cm_rough_.reset(Astar::Vec(size[0], size[1], 1));
    cm_hyst_.reset(Astar::Vec(size[0], size[1], 1));
    cm_hist_.reset(Astar::Vec(size[0], size[1], 1));
    cm_hist_bbf_.reset(Astar::Vec(size[0], size[1], 1));

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
    cm_hist_bbf_.clear(bbf::BinaryBayesFilter(bbf::MIN_ODDS));
    cm_hist_.clear(0);

    updateGoal();
  }
  void cbAction()
  {
    move_base_msgs::MoveBaseGoalConstPtr goal = act_->acceptNewGoal();
    if (!setGoal(goal->target_pose))
    {
      act_->setAborted(move_base_msgs::MoveBaseResult(), "Given goal is invalid.");
    }
  }

  void updateStart()
  {
    geometry_msgs::PoseStamped start;
    start.header.frame_id = robot_frame_;
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
      geometry_msgs::TransformStamped trans =
          tfbuf_.lookupTransform(map_header_.frame_id, robot_frame_, ros::Time(), ros::Duration(0.1));
      tf2::doTransform(start, start, trans);
    }
    catch (tf2::TransformException& e)
    {
      has_start_ = false;
      return;
    }
    start_ = start;
    has_start_ = true;
  }

public:
  Planner3dNode()
    : nh_()
    , pnh_("~")
    , tfl_(tfbuf_)
    , jump_(tfbuf_)
  {
    neonavigation_common::compat::checkCompatMode();
    sub_map_ = neonavigation_common::compat::subscribe(
        nh_, "costmap",
        pnh_, "costmap", 1, &Planner3dNode::cbMap, this);
    sub_map_update_ = neonavigation_common::compat::subscribe(
        nh_, "costmap_update",
        pnh_, "costmap_update", 1, &Planner3dNode::cbMapUpdate, this);
    sub_goal_ = neonavigation_common::compat::subscribe(
        nh_, "move_base_simple/goal",
        pnh_, "goal", 1, &Planner3dNode::cbGoal, this);
    pub_debug_ = pnh_.advertise<sensor_msgs::PointCloud>("debug", 1, true);
    pub_hist_ = pnh_.advertise<sensor_msgs::PointCloud>("remembered", 1, true);
    pub_start_ = pnh_.advertise<geometry_msgs::PoseStamped>("path_start", 1, true);
    pub_end_ = pnh_.advertise<geometry_msgs::PoseStamped>("path_end", 1, true);
    pub_status_ = pnh_.advertise<planner_cspace_msgs::PlannerStatus>("status", 1, true);
    srs_forget_ = neonavigation_common::compat::advertiseService(
        nh_, "forget_planning_cost",
        pnh_, "forget", &Planner3dNode::cbForget, this);
    srs_make_plan_ = pnh_.advertiseService("make_plan", &Planner3dNode::cbMakePlan, this);

    act_.reset(new Planner3DActionServer(ros::NodeHandle(), "move_base", false));
    act_->registerGoalCallback(boost::bind(&Planner3dNode::cbAction, this));
    act_->registerPreemptCallback(boost::bind(&Planner3dNode::cbPreempt, this));

    pnh_.param("use_path_with_velocity", use_path_with_velocity_, false);
    if (use_path_with_velocity_)
    {
      pub_path_velocity_ = nh_.advertise<trajectory_tracker_msgs::PathWithVelocity>(
          "path_velocity", 1, true);
    }
    else
    {
      pub_path_ = neonavigation_common::compat::advertise<nav_msgs::Path>(
          nh_, "path",
          pnh_, "path", 1, true);
    }
    pub_path_poses_ = pnh_.advertise<geometry_msgs::PoseArray>(
        "path_poses", 1, true);

    pnh_.param_cast("freq", freq_, 4.0f);
    pnh_.param_cast("freq_min", freq_min_, 2.0f);
    pnh_.param_cast("search_range", search_range_, 0.4f);

    double costmap_watchdog;
    pnh_.param("costmap_watchdog", costmap_watchdog, 0.0);
    costmap_watchdog_ = ros::Duration(costmap_watchdog);

    pnh_.param_cast("max_vel", max_vel_, 0.3f);
    pnh_.param_cast("max_ang_vel", max_ang_vel_, 0.6f);
    pnh_.param_cast("min_curve_raduis", min_curve_raduis_, 0.1f);

    pnh_.param_cast("weight_decel", cc_.weight_decel_, 50.0f);
    pnh_.param_cast("weight_backward", cc_.weight_backward_, 0.9f);
    pnh_.param_cast("weight_ang_vel", cc_.weight_ang_vel_, 1.0f);
    pnh_.param_cast("weight_costmap", cc_.weight_costmap_, 50.0f);
    pnh_.param_cast("weight_costmap_turn", cc_.weight_costmap_turn_, 0.0f);
    pnh_.param_cast("weight_remembered", cc_.weight_remembered_, 1000.0f);
    pnh_.param_cast("cost_in_place_turn", cc_.in_place_turn_, 30.0f);
    pnh_.param_cast("hysteresis_max_dist", cc_.hysteresis_max_dist_, 0.3f);
    pnh_.param_cast("hysteresis_expand", cc_.hysteresis_expand_, 0.1f);
    pnh_.param_cast("weight_hysteresis", cc_.weight_hysteresis_, 5.0f);

    pnh_.param("goal_tolerance_lin", goal_tolerance_lin_f_, 0.05);
    pnh_.param("goal_tolerance_ang", goal_tolerance_ang_f_, 0.1);
    pnh_.param("goal_tolerance_ang_finish", goal_tolerance_ang_finish_, 0.05);

    pnh_.param("unknown_cost", unknown_cost_, 100);
    pnh_.param("overwrite_cost", overwrite_cost_, false);

    pnh_.param("hist_ignore_range", hist_ignore_range_f_, 0.6);
    pnh_.param("hist_ignore_range_max", hist_ignore_range_max_f_, 1.25);
    pnh_.param("remember_updates", remember_updates_, false);
    double remember_hit_prob, remember_miss_prob;
    pnh_.param("remember_hit_prob", remember_hit_prob, 0.6);
    pnh_.param("remember_miss_prob", remember_miss_prob, 0.3);
    remember_hit_odds_ = bbf::probabilityToOdds(remember_hit_prob);
    remember_miss_odds_ = bbf::probabilityToOdds(remember_miss_prob);

    pnh_.param("local_range", local_range_f_, 2.5);
    pnh_.param("longcut_range", longcut_range_f_, 0.0);
    pnh_.param("esc_range", esc_range_f_, 0.25);
    pnh_.param("tolerance_range", tolerance_range_f_, 0.25);
    pnh_.param("tolerance_angle", tolerance_angle_f_, 0.0);

    pnh_.param_cast("sw_wait", sw_wait_, 2.0f);
    pnh_.param("find_best", find_best_, true);

    pnh_.param("robot_frame", robot_frame_, std::string("base_link"));

    double pos_jump, yaw_jump;
    std::string jump_detect_frame;
    pnh_.param("pos_jump", pos_jump, 1.0);
    pnh_.param("yaw_jump", yaw_jump, 1.5);
    pnh_.param("jump_detect_frame", jump_detect_frame, std::string("base_link"));
    jump_.setBaseFrame(jump_detect_frame);
    jump_.setThresholds(pos_jump, yaw_jump);

    pnh_.param("force_goal_orientation", force_goal_orientation_, true);

    pnh_.param("temporary_escape", temporary_escape_, true);

    pnh_.param("fast_map_update", fast_map_update_, false);
    if (fast_map_update_)
    {
      ROS_WARN("planner_3d: Experimental fast_map_update is enabled. ");
    }
    std::string debug_mode;
    pnh_.param("debug_mode", debug_mode, std::string("cost_estim"));
    if (debug_mode == "hyst")
      debug_out_ = DEBUG_HYSTERESIS;
    else if (debug_mode == "hist")
      debug_out_ = DEBUG_HISTORY;
    else if (debug_mode == "cost_estim")
      debug_out_ = DEBUG_COST_ESTIM;

    bool print_planning_duration;
    pnh_.param("print_planning_duration", print_planning_duration, false);
    if (print_planning_duration)
    {
      if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      {
        ros::console::notifyLoggerLevelsChanged();
      }
    }

    pnh_.param("max_retry_num", max_retry_num_, -1);

    int queue_size_limit;
    pnh_.param("queue_size_limit", queue_size_limit, 0);
    as_.setQueueSizeLimit(queue_size_limit);

    int num_threads;
    pnh_.param("num_threads", num_threads, 1);
    omp_set_num_threads(num_threads);

    pnh_.param("num_search_task", num_task_, num_threads * 16);
    as_.setSearchTaskNum(num_task_);

    status_.status = planner_cspace_msgs::PlannerStatus::DONE;

    has_map_ = false;
    has_goal_ = false;
    has_start_ = false;
    goal_updated_ = false;

    escaping_ = false;
    cnt_stuck_ = 0;

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Path Planner Status", this, &Planner3dNode::diagnoseStatus);

    act_->start();
  }
  void spin()
  {
    ros::Rate wait(freq_);
    ROS_DEBUG("Initialized");

    while (ros::ok())
    {
      wait.sleep();
      ros::spinOnce();

      const ros::Time now = ros::Time::now();

      if (has_map_)
      {
        updateStart();
        if (jump_.detectJump())
        {
          cm_hist_bbf_.clear(bbf::BinaryBayesFilter(bbf::MIN_ODDS));
        }

        if (!goal_updated_ && has_goal_)
          updateGoal();
      }

      bool has_costmap(false);
      if (costmap_watchdog_ > ros::Duration(0))
      {
        if (last_costmap_ + costmap_watchdog_ < now)
        {
          ROS_WARN(
              "Navigation is stopping since the costmap is too old (costmap: %0.3f)",
              last_costmap_.toSec());
          status_.error = planner_cspace_msgs::PlannerStatus::DATA_MISSING;
          publishEmptyPath();
        }
        else
        {
          has_costmap = true;
        }
      }
      else
      {
        has_costmap = true;
      }

      if (has_map_ && has_goal_ && has_start_ && has_costmap)
      {
        if (act_->isActive())
        {
          move_base_msgs::MoveBaseFeedback feedback;
          feedback.base_position = start_;
          act_->publishFeedback(feedback);
        }

        if (status_.status == planner_cspace_msgs::PlannerStatus::FINISHING)
        {
          const float yaw_s = tf2::getYaw(start_.pose.orientation);
          float yaw_g;
          if (force_goal_orientation_)
            yaw_g = tf2::getYaw(goal_raw_.pose.orientation);
          else
            yaw_g = tf2::getYaw(goal_.pose.orientation);

          float yaw_diff = yaw_s - yaw_g;
          if (yaw_diff > M_PI)
            yaw_diff -= M_PI * 2.0;
          else if (yaw_diff < -M_PI)
            yaw_diff += M_PI * 2.0;
          if (fabs(yaw_diff) < goal_tolerance_ang_finish_)
          {
            status_.status = planner_cspace_msgs::PlannerStatus::DONE;
            has_goal_ = false;
            ROS_INFO("Path plan finished");
            act_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          }
        }
        else
        {
          if (escaping_)
          {
            status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
          }
          else if (max_retry_num_ != -1 && cnt_stuck_ > max_retry_num_)
          {
            status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
            status_.status = planner_cspace_msgs::PlannerStatus::DONE;
            has_goal_ = false;
            act_->setAborted(move_base_msgs::MoveBaseResult(),
                             "Goal is in Rock");
            ROS_ERROR("Exceeded max_retry_num:%d", max_retry_num_);
            continue;
          }
          else
          {
            status_.error = planner_cspace_msgs::PlannerStatus::GOING_WELL;
          }
          nav_msgs::Path path;
          path.header = map_header_;
          path.header.stamp = now;
          makePlan(start_.pose, goal_.pose, path, true);
          if (use_path_with_velocity_)
          {
            // NaN velocity means that don't care the velocity
            pub_path_velocity_.publish(
                trajectory_tracker_msgs::toPathWithVelocity(path, std::numeric_limits<double>::quiet_NaN()));
          }
          else
          {
            pub_path_.publish(path);
          }

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
        publishEmptyPath();
      }
      pub_status_.publish(status_);
      diag_updater_.force_update();
    }
  }

protected:
  bool makePlan(const geometry_msgs::Pose& gs, const geometry_msgs::Pose& ge,
                nav_msgs::Path& path, bool hyst)
  {
    Astar::Vec s, e;
    grid_metric_converter::metric2Grid(
        map_info_, s[0], s[1], s[2],
        gs.position.x, gs.position.y, tf2::getYaw(gs.orientation));
    s.cycleUnsigned(map_info_.angle);
    grid_metric_converter::metric2Grid(
        map_info_, e[0], e[1], e[2],
        ge.position.x, ge.position.y, tf2::getYaw(ge.orientation));
    e.cycleUnsigned(map_info_.angle);

    geometry_msgs::PoseStamped p;
    p.header = map_header_;
    float x, y, yaw;
    grid_metric_converter::grid2Metric(map_info_, e[0], e[1], e[2], x, y, yaw);
    p.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    p.pose.position.x = x;
    p.pose.position.y = y;
    pub_end_.publish(p);

    if (cm_[s] == 100)
    {
      if (!searchAvailablePos(s, tolerance_range_, tolerance_angle_))
      {
        ROS_WARN("Oops! You are in Rock!");
        status_.error = planner_cspace_msgs::PlannerStatus::IN_ROCK;
        return false;
      }
      ROS_INFO("Start moved");
    }
    const Astar::Vec s_rough(s[0], s[1], 0);

    if (cost_estim_cache_[s_rough] == FLT_MAX)
    {
      status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
      ROS_WARN("Goal unreachable.");
      if (!escaping_ && temporary_escape_)
      {
        e = s;
        if (searchAvailablePos(e, esc_range_, esc_angle_, 50, esc_range_ / 2))
        {
          escaping_ = true;
          ROS_INFO("Temporary goal (%d, %d, %d)",
                   e[0], e[1], e[2]);
          float x, y, yaw;
          grid_metric_converter::grid2Metric(map_info_, e[0], e[1], e[2], x, y, yaw);
          goal_.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
          goal_.pose.position.x = x;
          goal_.pose.position.y = y;

          updateGoal();
          return false;
        }
      }
      return false;
    }

    grid_metric_converter::grid2Metric(map_info_, s[0], s[1], s[2], x, y, yaw);
    p.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    p.pose.position.x = x;
    p.pose.position.y = y;
    pub_start_.publish(p);

    auto diff = s - e;
    diff.cycle(map_info_.angle);
    if (diff.sqlen() <= goal_tolerance_lin_ * goal_tolerance_lin_ &&
        abs(diff[2]) <= goal_tolerance_ang_)
    {
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
        status_.status = planner_cspace_msgs::PlannerStatus::FINISHING;
        ROS_INFO("Path plan finishing");
      }
      return true;
    }

    auto range_limit = cost_estim_cache_[s_rough] - (local_range_ + range_) * ec_[0];
    angle_resolution_aspect_ = 2.0 / tanf(map_info_.angular_resolution);

    const auto ts = boost::chrono::high_resolution_clock::now();
    // ROS_INFO("Planning from (%d, %d, %d) to (%d, %d, %d)",
    //   s[0], s[1], s[2], e[0], e[1], e[2]);
    std::list<Astar::Vec> path_grid;
    if (!as_.search(s, e, path_grid,
                    std::bind(&Planner3dNode::cbCost,
                              this, std::placeholders::_1, std::placeholders::_2,
                              std::placeholders::_3, std::placeholders::_4, hyst),
                    std::bind(&Planner3dNode::cbCostEstim,
                              this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&Planner3dNode::cbSearch,
                              this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3),
                    std::bind(&Planner3dNode::cbProgress,
                              this, std::placeholders::_1),
                    range_limit,
                    1.0f / freq_min_,
                    true))
    {
      ROS_WARN("Path plan failed (goal unreachable)");
      status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
      if (!find_best_)
        return false;
    }
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Path found (%0.4f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());

    geometry_msgs::PoseArray poses;
    poses.header = path.header;
    for (const auto& p : path_grid)
    {
      geometry_msgs::Pose pose;
      float x, y, yaw;
      grid_metric_converter::grid2Metric(map_info_, p[0], p[1], p[2], x, y, yaw);
      pose.position.x = x;
      pose.position.y = y;
      pose.orientation =
          tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
      poses.poses.push_back(pose);
    }
    pub_path_poses_.publish(poses);

    grid_metric_converter::grid2MetricPath(map_info_, local_range_, path_grid, path, s);

    if (hyst)
    {
      std::unordered_map<Astar::Vec, bool, Astar::Vec> path_points;
      const float max_dist = cc_.hysteresis_max_dist_ / map_info_.linear_resolution;
      const float expand_dist = cc_.hysteresis_expand_ / map_info_.linear_resolution;
      const int path_range = range_ + max_dist + expand_dist + 1;
      for (auto& p : path_grid)
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
      for (auto& ps : path_points)
      {
        auto& p = ps.first;
        float d_min = FLT_MAX;
        auto it_prev = path_grid.begin();
        for (auto it = path_grid.begin(); it != path_grid.end(); it++)
        {
          if (it != it_prev)
          {
            auto d = p.distLinestrip2d(*it_prev, *it);
            if (d < d_min)
              d_min = d;
          }
          it_prev = it;
        }
        if (d_min < expand_dist)
          d_min = 0;
        else if (d_min > max_dist + expand_dist)
          d_min = max_dist;
        else
          d_min -= expand_dist;
        cm_hyst_[p] = (d_min - expand_dist) * 100.0 / max_dist;
      }
      // const auto tnow = boost::chrono::high_resolution_clock::now();
      // ROS_INFO("Hysteresis map generated (%0.3f sec.)",
      //   boost::chrono::duration<float>(tnow - ts).count());
      publishCostmap();
    }

    return true;
  }
  std::vector<Astar::Vec>& cbSearch(
      const Astar::Vec& p,
      const Astar::Vec& s, const Astar::Vec& e)
  {
    const auto ds = s - p;

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
  bool cbProgress(const std::list<Astar::Vec>& path_grid)
  {
    publishEmptyPath();
    ROS_WARN("Search timed out");
    return true;
  }

  float cbCostEstim(const Astar::Vec& s, const Astar::Vec& e)
  {
    Astar::Vec s2(s[0], s[1], 0);
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
  bool switchDetect(const nav_msgs::Path& path)
  {
    geometry_msgs::Pose p_prev;
    bool first(true);
    bool dir_set(false);
    bool dir_prev(false);
    for (const auto& p : path.poses)
    {
      if (!first)
      {
        const float len = hypotf(
            p.pose.position.y - p_prev.position.y,
            p.pose.position.x - p_prev.position.x);
        if (len > 0.001)
        {
          const float yaw = tf2::getYaw(p.pose.orientation);
          const float vel_yaw = atan2f(
              p.pose.position.y - p_prev.position.y,
              p.pose.position.x - p_prev.position.x);
          const bool dir = (cosf(yaw) * cosf(vel_yaw) + sinf(yaw) * sinf(vel_yaw) < 0);

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
  float cbCost(const Astar::Vec& s, Astar::Vec& e,
               const Astar::Vec& v_goal,
               const Astar::Vec& v_start,
               const bool hyst)
  {
    Astar::Vec d_raw = e - s;
    d_raw.cycle(map_info_.angle);
    const Astar::Vec d = d_raw;
    float cost = euclidCost(d);

    if (d[0] == 0 && d[1] == 0)
    {
      // In-place turn
      int sum = 0;
      const int dir = d[2] < 0 ? -1 : 1;
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

      const float cost =
          sum * map_info_.angular_resolution * ec_[2] / ec_[0] +
          sum * map_info_.angular_resolution * cc_.weight_costmap_turn_ / 100.0;
      // simplified from sum * map_info_.angular_resolution * abs(d[2]) * cc_.weight_costmap_turn_ / (100.0 * abs(d[2]))
      return cc_.in_place_turn_ + cost;
    }

    Astar::Vec d2;
    d2[0] = d[0] + range_;
    d2[1] = d[1] + range_;
    d2[2] = e[2];

    const Astar::Vecf motion = rot_cache_.getMotion(s[2], d2);
    const Astar::Vecf motion_grid = motion * resolution_;

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

    if (motion[0] < 0)
    {
      // Going backward
      cost *= 1.0 + cc_.weight_backward_;
    }

    if (d[2] == 0)
    {
      if (lroundf(motion_grid[0]) == 0)
        return -1;  // side slip
      const float aspect = motion[0] / motion[1];
      if (fabs(aspect) < angle_resolution_aspect_)
        return -1;  // large y offset

      // Go-straight
      int sum = 0, sum_hyst = 0;
      Astar::Vec d_index(d[0], d[1], e[2]);
      d_index.cycleUnsigned(map_info_.angle);

      const auto cache_page = motion_cache_.find(s[2], d_index);
      if (cache_page == motion_cache_.end(s[2]))
        return -1;
      const int num = cache_page->second.getMotion().size();
      for (const auto& pos_diff : cache_page->second.getMotion())
      {
        const Astar::Vec pos(
            s[0] + pos_diff[0], s[1] + pos_diff[1], pos_diff[2]);
        const auto c = cm_[pos];
        if (c > 99)
          return -1;
        if (hyst)
        {
          const Astar::Vec pos_rough(pos[0], pos[1], 0);
          sum_hyst += cm_hyst_[pos_rough];
        }
        sum += c;
      }
      const float distf = cache_page->second.getDistance();
      cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / (100.0 * num);
      cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / (100.0 * num);
    }
    else
    {
      // Curve
      if (motion[0] * motion[1] * motion[2] < 0)
        return -1;

      if (d.sqlen() < 3 * 3)
        return -1;

      const std::pair<float, float>& radiuses = rot_cache_.getRadiuses(s[2], d2);
      const float r1 = radiuses.first;
      const float r2 = radiuses.second;

      // curveture at the start_ pose and the end pose must be same
      if (fabs(r1 - r2) >= map_info_.linear_resolution * 1.5)
      {
        // Drifted
        return -1;
      }

      const float curv_radius = (r1 + r2) / 2;
      if (std::abs(curv_radius) < min_curve_raduis_)
        return -1;

      if (fabs(max_vel_ / r1) > max_ang_vel_)
      {
        const float vel = fabs(curv_radius) * max_ang_vel_;

        // Curve deceleration penalty
        cost += dist * fabs(vel / max_vel_) * cc_.weight_decel_;
      }

      {
        int sum = 0, sum_hyst = 0;
        Astar::Vec d_index(d[0], d[1], e[2]);
        d_index.cycleUnsigned(map_info_.angle);

        const auto cache_page = motion_cache_.find(s[2], d_index);
        if (cache_page == motion_cache_.end(s[2]))
          return -1;
        const int num = cache_page->second.getMotion().size();
        for (const auto& pos_diff : cache_page->second.getMotion())
        {
          const Astar::Vec pos(
              s[0] + pos_diff[0], s[1] + pos_diff[1], pos_diff[2]);
          const auto c = cm_[pos];
          if (c > 99)
            return -1;
          sum += c;
          if (hyst)
          {
            const Astar::Vec pos_rough(pos[0], pos[1], 0);
            sum_hyst += cm_hyst_[pos_rough];
          }
        }
        const float distf = cache_page->second.getDistance();
        cost += sum * map_info_.linear_resolution * distf * cc_.weight_costmap_ / (100.0 * num);
        cost += sum * map_info_.angular_resolution * abs(d[2]) * cc_.weight_costmap_turn_ / (100.0 * num);
        cost += sum_hyst * map_info_.linear_resolution * distf * cc_.weight_hysteresis_ / (100.0 * num);
      }
    }

    return cost;
  }

  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    switch (status_.error)
    {
      case planner_cspace_msgs::PlannerStatus::GOING_WELL:
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Going well.");
        break;
      case planner_cspace_msgs::PlannerStatus::IN_ROCK:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "The robot is in rock.");
        break;
      case planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Path not found.");
        break;
      case planner_cspace_msgs::PlannerStatus::DATA_MISSING:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Required data is missing.");
        break;
      case planner_cspace_msgs::PlannerStatus::INTERNAL_ERROR:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Planner internal error.");
        break;
      default:
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown error.");
        break;
    }
    stat.addf("status", "%u", status_.status);
    stat.addf("error", "%u", status_.error);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "planner_3d");

  Planner3dNode jy;
  jy.spin();

  return 0;
}
