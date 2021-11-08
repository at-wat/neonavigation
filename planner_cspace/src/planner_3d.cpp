/*
 * Copyright (c) 2014-2020, the neonavigation authors
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
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <omp.h>

#include <ros/ros.h>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <planner_cspace_msgs/PlannerStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/Empty.h>
#include <trajectory_tracker_msgs/PathWithVelocity.h>
#include <trajectory_tracker_msgs/converter.h>

#include <ros/console.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planner_cspace_msgs/MoveWithToleranceAction.h>

#include <neonavigation_common/compatibility.h>

#include <planner_cspace/bbf.h>
#include <planner_cspace/grid_astar.h>
#include <planner_cspace/jump_detector.h>
#include <planner_cspace/planner_3d/costmap_bbf.h>
#include <planner_cspace/planner_3d/grid_astar_model.h>
#include <planner_cspace/planner_3d/grid_metric_converter.h>
#include <planner_cspace/planner_3d/motion_cache.h>
#include <planner_cspace/planner_3d/path_interpolator.h>
#include <planner_cspace/planner_3d/rotation_cache.h>
#include <planner_cspace/planner_3d/distance_map.h>

namespace planner_cspace
{
namespace planner_3d
{
class Planner3dNode
{
public:
  using Astar = GridAstar<3, 2>;

protected:
  using Planner3DActionServer = actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>;
  using Planner3DTolerantActionServer = actionlib::SimpleActionServer<planner_cspace_msgs::MoveWithToleranceAction>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_update_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_velocity_;
  ros::Publisher pub_path_poses_;
  ros::Publisher pub_hysteresis_map_;
  ros::Publisher pub_distance_map_;
  ros::Publisher pub_remembered_map_;
  ros::Publisher pub_start_;
  ros::Publisher pub_end_;
  ros::Publisher pub_status_;
  ros::ServiceServer srs_forget_;
  ros::ServiceServer srs_make_plan_;

  std::shared_ptr<Planner3DActionServer> act_;
  std::shared_ptr<Planner3DTolerantActionServer> act_tolerant_;
  planner_cspace_msgs::MoveWithToleranceGoalConstPtr goal_tolerant_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;
  Astar::Gridmap<char, 0x80> cm_rough_;
  Astar::Gridmap<char, 0x40> cm_base_;
  Astar::Gridmap<char, 0x80> cm_rough_base_;
  Astar::Gridmap<char, 0x80> cm_hyst_;
  Astar::Gridmap<char, 0x80> cm_updates_;
  CostmapBBF bbf_costmap_;
  DistanceMap cost_estim_cache_;

  GridAstarModel3D::Ptr model_;

  costmap_cspace_msgs::MapMetaData3D map_info_;
  std_msgs::Header map_header_;
  float freq_;
  float freq_min_;
  float search_timeout_abort_;
  float search_range_;
  bool antialias_start_;
  int range_;
  int local_range_;
  double local_range_f_;
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
  bool has_hysteresis_map_;
  std::vector<Astar::Vec> hyst_updated_cells_;
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
  bool retain_last_error_status_;

  JumpDetector jump_;
  std::string robot_frame_;

  int max_retry_num_;

  int num_task_;

  // Cost weights
  CostCoeff cc_;

  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped goal_raw_;
  Astar::Vecf ec_;
  double goal_tolerance_lin_f_;
  double goal_tolerance_ang_f_;
  double goal_tolerance_ang_finish_;
  int goal_tolerance_lin_;
  int goal_tolerance_ang_;

  planner_cspace_msgs::PlannerStatus status_;

  bool find_best_;
  float sw_wait_;
  geometry_msgs::PoseStamped sw_pos_;
  bool is_path_switchback_;

  bool rough_;

  bool force_goal_orientation_;

  bool escaping_;

  int cnt_stuck_;

  diagnostic_updater::Updater diag_updater_;
  ros::Duration costmap_watchdog_;
  ros::Time last_costmap_;

  int prev_map_update_x_min_;
  int prev_map_update_x_max_;
  int prev_map_update_y_min_;
  int prev_map_update_y_max_;

  bool cbForget(std_srvs::EmptyRequest& req,
                std_srvs::EmptyResponse& res)
  {
    ROS_WARN("Forgetting remembered costmap.");
    if (has_map_)
      bbf_costmap_.clear();

    return true;
  }
  enum class DiscretePoseStatus
  {
    OK,
    RELOCATED,
    IN_ROCK,
    OUT_OF_MAP,
  };
  template <class T>
  DiscretePoseStatus relocateDiscretePoseIfNeededImpl(const T& cm,
                                                      const int tolerance_range,
                                                      const int tolerance_angle,
                                                      Astar::Vec& pose_discrete) const
  {
    if (!cm.validate(pose_discrete, range_))
    {
      return DiscretePoseStatus::OUT_OF_MAP;
    }
    if (cm[pose_discrete] == 100)
    {
      if (searchAvailablePos(cm, pose_discrete, tolerance_range, tolerance_angle))
        return DiscretePoseStatus::RELOCATED;
      else
        return DiscretePoseStatus::IN_ROCK;
    }
    return DiscretePoseStatus::OK;
  }
  DiscretePoseStatus relocateDiscretePoseIfNeeded(Astar::Vec& pose_discrete,
                                                  const int tolerance_range,
                                                  const int tolerance_angle,
                                                  bool use_cm_rough = false) const
  {
    if (use_cm_rough)
    {
      pose_discrete[2] = 0;
      return relocateDiscretePoseIfNeededImpl(cm_rough_, tolerance_range, tolerance_angle, pose_discrete);
    }
    else
    {
      return relocateDiscretePoseIfNeededImpl(cm_, tolerance_range, tolerance_angle, pose_discrete);
    }
  }

  bool cbMakePlan(nav_msgs::GetPlan::Request& req,
                  nav_msgs::GetPlan::Response& res)
  {
    if (!has_map_)
    {
      ROS_ERROR("make_plan service is called without map.");
      return false;
    }

    if (req.start.header.frame_id != map_header_.frame_id ||
        req.goal.header.frame_id != map_header_.frame_id)
    {
      ROS_ERROR("Start [%s] and Goal [%s] poses must be in the map frame [%s].",
                req.start.header.frame_id.c_str(),
                req.goal.header.frame_id.c_str(),
                map_header_.frame_id.c_str());
      return false;
    }

    Astar::Vec s, e;
    grid_metric_converter::metric2Grid(
        map_info_, s[0], s[1], s[2],
        req.start.pose.position.x, req.start.pose.position.y, tf2::getYaw(req.start.pose.orientation));
    grid_metric_converter::metric2Grid(
        map_info_, e[0], e[1], e[2],
        req.goal.pose.position.x, req.goal.pose.position.y, tf2::getYaw(req.goal.pose.orientation));
    ROS_INFO("Path plan from (%d, %d) to (%d, %d)", s[0], s[1], e[0], e[1]);

    const int tolerance_range = std::lround(req.tolerance / map_info_.linear_resolution);
    const DiscretePoseStatus start_status = relocateDiscretePoseIfNeeded(s, tolerance_range, tolerance_angle_, true);
    const DiscretePoseStatus goal_status = relocateDiscretePoseIfNeeded(e, tolerance_range, tolerance_angle_, true);
    switch (start_status)
    {
      case DiscretePoseStatus::OUT_OF_MAP:
        ROS_ERROR("Given start is not on the map.");
        return false;
      case DiscretePoseStatus::IN_ROCK:
        ROS_ERROR("Given start is in Rock.");
        return false;
      case DiscretePoseStatus::RELOCATED:
        ROS_INFO("Given start is moved (%d, %d)", s[0], s[1]);
        break;
      default:
        break;
    }
    switch (goal_status)
    {
      case DiscretePoseStatus::OUT_OF_MAP:
        ROS_ERROR("Given goal is not on the map.");
        return false;
      case DiscretePoseStatus::IN_ROCK:
        ROS_ERROR("Given goal is in Rock.");
        return false;
      case DiscretePoseStatus::RELOCATED:
        ROS_INFO("Given goal is moved (%d, %d)", e[0], e[1]);
        break;
      default:
        break;
    }

    const auto cb_progress = [](const std::list<Astar::Vec>& /* path_grid */, const SearchStats& /* stats */)
    {
      return true;
    };

    const auto ts = boost::chrono::high_resolution_clock::now();

    GridAstarModel2D::Ptr model_2d(new GridAstarModel2D(model_));

    std::list<Astar::Vec> path_grid;
    std::vector<GridAstarModel3D::VecWithCost> starts;
    starts.emplace_back(s);
    if (!as_.search(
            starts, e, path_grid,
            model_2d,
            cb_progress,
            0, 1.0f / freq_min_, find_best_))
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

    const std::list<Astar::Vecf> path_interpolated =
        model_->path_interpolator_.interpolate(path_grid, 0.5, 0.0);
    grid_metric_converter::grid2MetricPath(map_info_, path_interpolated, path);

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
    if (act_->isActive() || act_tolerant_->isActive())
    {
      ROS_ERROR("Setting new goal is ignored since planner_3d is proceeding the action.");
      return;
    }
    setGoal(*msg);
  }
  void cbPreempt()
  {
    ROS_WARN("Preempting the current goal.");
    if (act_->isActive())
      act_->setPreempted(move_base_msgs::MoveBaseResult(), "Preempted.");

    if (act_tolerant_->isActive())
      act_tolerant_->setPreempted(planner_cspace_msgs::MoveWithToleranceResult(), "Preempted.");

    has_goal_ = false;
    status_.status = planner_cspace_msgs::PlannerStatus::DONE;
  }

  bool setGoal(const geometry_msgs::PoseStamped& msg)
  {
    if (msg.header.frame_id != map_header_.frame_id)
    {
      ROS_ERROR("Goal [%s] pose must be in the map frame [%s].",
                msg.header.frame_id.c_str(), map_header_.frame_id.c_str());
      return false;
    }

    goal_raw_ = goal_ = msg;

    const double len2 =
        goal_.pose.orientation.x * goal_.pose.orientation.x +
        goal_.pose.orientation.y * goal_.pose.orientation.y +
        goal_.pose.orientation.z * goal_.pose.orientation.z +
        goal_.pose.orientation.w * goal_.pose.orientation.w;
    if (std::abs(len2 - 1.0) < 0.1)
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
      if (act_->isActive())
        act_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal cleared.");
      if (act_tolerant_->isActive())
        act_tolerant_->setSucceeded(planner_cspace_msgs::MoveWithToleranceResult(), "Goal cleared.");
    }
    return true;
  }

  template <class T>
  bool searchAvailablePos(const T& cm, Astar::Vec& s, const int xy_range, const int angle_range,
                          const int cost_acceptable = 50, const int min_xy_range = 0) const
  {
    ROS_DEBUG("%d, %d  (%d,%d,%d)", xy_range, angle_range, s[0], s[1], s[2]);

    float range_min = std::numeric_limits<float>::max();
    Astar::Vec s_out;
    Astar::Vec d;
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
          if (!cm_.validate(s2, range_))
            continue;

          if (cm_[s2] >= cost_acceptable)
            continue;
          const auto cost = model_->euclidCost(d);
          if (cost < range_min)
          {
            range_min = cost;
            s_out = s2;
          }
        }
      }
    }

    if (range_min == std::numeric_limits<float>::max())
    {
      if (cost_acceptable != 100)
      {
        return searchAvailablePos(cm, s, xy_range, angle_range, 100);
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
    if (!has_goal_)
      return true;

    if (!has_map_ || !has_start_)
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
    const DiscretePoseStatus start_pose_status = relocateDiscretePoseIfNeeded(s, tolerance_range_, tolerance_angle_);
    const DiscretePoseStatus goal_pose_status = relocateDiscretePoseIfNeeded(e, tolerance_range_, tolerance_angle_);
    switch (start_pose_status)
    {
      case DiscretePoseStatus::OUT_OF_MAP:
        ROS_ERROR("You are on the edge of the world.");
        return false;
      case DiscretePoseStatus::IN_ROCK:
        ROS_WARN("Oops! You are in Rock!");
        return true;
      default:
        break;
    }
    switch (goal_pose_status)
    {
      case DiscretePoseStatus::OUT_OF_MAP:
        ROS_ERROR("Given goal is not on the map.");
        return false;
      case DiscretePoseStatus::IN_ROCK:
        ROS_WARN("Oops! Goal is in Rock!");
        ++cnt_stuck_;
        return true;
      case DiscretePoseStatus::RELOCATED:
        ROS_INFO("Goal moved (%d, %d, %d)", e[0], e[1], e[2]);
        float x, y, yaw;
        grid_metric_converter::grid2Metric(map_info_, e[0], e[1], e[2], x, y, yaw);
        goal_.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
        goal_.pose.position.x = x;
        goal_.pose.position.y = y;
        break;
      default:
        break;
    }
    const auto ts = boost::chrono::high_resolution_clock::now();
    cost_estim_cache_.create(s, e);
    const auto tnow = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Cost estimation cache generated (%0.4f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());

    if (goal_changed)
    {
      clearHysteresis();
      has_hysteresis_map_ = false;
    }

    publishDebug();

    goal_updated_ = true;

    return true;
  }
  void clearHysteresis()
  {
    for (const Astar::Vec& p : hyst_updated_cells_)
    {
      cm_hyst_[p] = 100;
    }
    hyst_updated_cells_.clear();
  }
  void publishDebug()
  {
    if (pub_distance_map_.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud distance_map;
      distance_map.header = map_header_;
      distance_map.header.stamp = ros::Time::now();
      distance_map.channels.resize(1);
      distance_map.channels[0].name = "distance";
      distance_map.points.reserve(1024);
      distance_map.channels[0].values.reserve(1024);
      const float k_dist = map_info_.linear_resolution * cc_.max_vel_;
      for (Astar::Vec p(0, 0, 0); p[1] < cost_estim_cache_.size()[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cost_estim_cache_.size()[0]; p[0]++)
        {
          p[2] = 0;
          float x, y, yaw;
          grid_metric_converter::grid2Metric(map_info_, p[0], p[1], p[2], x, y, yaw);
          geometry_msgs::Point32 point;
          point.x = x;
          point.y = y;
          if (cost_estim_cache_[p] == std::numeric_limits<float>::max())
            continue;
          point.z = cost_estim_cache_[p] / 500;
          distance_map.points.push_back(point);
          distance_map.channels[0].values.push_back(cost_estim_cache_[p] * k_dist);
        }
      }
      pub_distance_map_.publish(distance_map);
    }

    if (pub_hysteresis_map_.getNumSubscribers() > 0)
    {
      nav_msgs::OccupancyGrid hysteresis_map;
      hysteresis_map.header.frame_id = map_header_.frame_id;
      hysteresis_map.info.resolution = map_info_.linear_resolution;
      hysteresis_map.info.width = map_info_.width;
      hysteresis_map.info.height = map_info_.height;
      hysteresis_map.info.origin = map_info_.origin;
      hysteresis_map.data.resize(map_info_.width * map_info_.height, 100);

      for (Astar::Vec p(0, 0, 0); p[1] < cost_estim_cache_.size()[1]; p[1]++)
      {
        for (p[0] = 0; p[0] < cost_estim_cache_.size()[0]; p[0]++)
        {
          if (cost_estim_cache_[p] == std::numeric_limits<float>::max())
            continue;

          char cost = 100;
          for (Astar::Vec p2 = p; p2[2] < static_cast<int>(map_info_.angle); ++p2[2])
          {
            cost = std::min(cm_hyst_[p2], cost);
          }
          hysteresis_map.data[p[0] + p[1] * map_info_.width] = cost;
        }
      }
      pub_hysteresis_map_.publish(hysteresis_map);
    }
  }
  void publishRememberedMap()
  {
    if (pub_remembered_map_.getNumSubscribers() > 0 && remember_updates_)
    {
      nav_msgs::OccupancyGrid remembered_map;
      remembered_map.header.frame_id = map_header_.frame_id;
      remembered_map.info.resolution = map_info_.linear_resolution;
      remembered_map.info.width = map_info_.width;
      remembered_map.info.height = map_info_.height;
      remembered_map.info.origin = map_info_.origin;
      remembered_map.data.resize(map_info_.width * map_info_.height);

      const auto generate_pointcloud = [this, &remembered_map](const Astar::Vec& p, bbf::BinaryBayesFilter& bbf)
      {
        remembered_map.data[p[0] + p[1] * map_info_.width] =
            (bbf.getProbability() - bbf::MIN_PROBABILITY) * 100 / (bbf::MAX_PROBABILITY - bbf::MIN_PROBABILITY);
      };
      bbf_costmap_.forEach(generate_pointcloud);
      pub_remembered_map_.publish(remembered_map);
    }
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
  void publishFinishPath()
  {
    nav_msgs::Path path;
    path.header.frame_id = map_header_.frame_id;
    path.header.stamp = ros::Time::now();
    // Specify single pose to control only orientation
    path.poses.resize(1);
    path.poses[0].header = path.header;
    if (force_goal_orientation_)
      path.poses[0].pose = goal_raw_.pose;
    else
      path.poses[0].pose = goal_.pose;

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

    const auto ts_cm_init_start = boost::chrono::high_resolution_clock::now();
    const ros::Time now = ros::Time::now();
    last_costmap_ = now;

    cm_.copy_partially(cm_base_, Astar::Vec(prev_map_update_x_min_, prev_map_update_y_min_, 0),
                       Astar::Vec(prev_map_update_x_max_, prev_map_update_y_max_, static_cast<int>(map_info_.angle)));
    cm_rough_.copy_partially(cm_rough_base_, Astar::Vec(prev_map_update_x_min_, prev_map_update_y_min_, 0),
                             Astar::Vec(prev_map_update_x_max_, prev_map_update_y_max_, 1));
    cm_updates_.clear_partially(-1, Astar::Vec(prev_map_update_x_min_, prev_map_update_y_min_, 0),
                                Astar::Vec(prev_map_update_x_max_, prev_map_update_y_max_, 1));

    const int map_update_x_min = static_cast<int>(msg->x);
    const int map_update_x_max = static_cast<int>(msg->x + msg->width);
    const int map_update_y_min = static_cast<int>(msg->y);
    const int map_update_y_max = static_cast<int>(msg->y + msg->height);

    // Should search 1px around the region to
    // update the costmap even if the edge of the local map is obstacle
    const int search_range_x_min = std::max(0, std::min(prev_map_update_x_min_, map_update_x_min) - 1);
    const int search_range_x_max = std::min(static_cast<int>(map_info_.width),
                                            std::max(prev_map_update_x_max_, map_update_x_max) + 1);
    const int search_range_y_min = std::max(0, std::min(prev_map_update_y_min_, map_update_y_min) - 1);
    const int search_range_y_max = std::min(static_cast<int>(map_info_.height),
                                            std::max(prev_map_update_y_max_, map_update_y_max) + 1);
    prev_map_update_x_min_ = map_update_x_min;
    prev_map_update_x_max_ = map_update_x_max;
    prev_map_update_y_min_ = map_update_y_min;
    prev_map_update_y_max_ = map_update_y_max;

    bool clear_hysteresis(false);

    {
      const Astar::Vec gp(
          static_cast<int>(msg->x), static_cast<int>(msg->y), static_cast<int>(msg->yaw));
      const Astar::Vec gp_rough(gp[0], gp[1], 0);
      for (Astar::Vec p(0, 0, 0); p[0] < static_cast<int>(msg->width); p[0]++)
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
            if (c == 100 && !clear_hysteresis && cm_hyst_[gp + p] == 0)
              clear_hysteresis = true;
          }
          p[2] = 0;
          cm_updates_[gp_rough + p] = cost_min;
          if (cost_min > cm_rough_[gp_rough + p])
            cm_rough_[gp_rough + p] = cost_min;

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
    const auto ts_cm_init_end = boost::chrono::high_resolution_clock::now();
    ROS_DEBUG("Costmaps updated (%.4f)", boost::chrono::duration<float>(ts_cm_init_end - ts_cm_init_start).count());

    if (clear_hysteresis && has_hysteresis_map_)
    {
      ROS_INFO("The previous path collides to the obstacle. Clearing hysteresis map.");
      clearHysteresis();
      has_hysteresis_map_ = false;
    }

    if (!has_start_)
      return;

    Astar::Vec s;
    grid_metric_converter::metric2Grid(
        map_info_, s[0], s[1], s[2],
        start_.pose.position.x, start_.pose.position.y,
        tf2::getYaw(start_.pose.orientation));
    s.cycleUnsigned(map_info_.angle);

    if (remember_updates_)
    {
      bbf_costmap_.remember(
          &cm_updates_, s,
          remember_hit_odds_, remember_miss_odds_,
          hist_ignore_range_, hist_ignore_range_max_);
      publishRememberedMap();
      bbf_costmap_.updateCostmap();
    }
    if (!has_goal_)
      return;

    if (!fast_map_update_)
    {
      updateGoal(false);
      return;
    }

    Astar::Vec e;
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

    const auto ts = boost::chrono::high_resolution_clock::now();
    cost_estim_cache_.update(
        s, e,
        DistanceMap::Rect(
            Astar::Vec(search_range_x_min, search_range_y_min, 0),
            Astar::Vec(search_range_x_max, search_range_y_max, 0)));
    const auto tnow = boost::chrono::high_resolution_clock::now();
    const DistanceMap::DebugData dm_debug = cost_estim_cache_.getDebugData();
    if (dm_debug.has_negative_cost)
    {
      ROS_WARN("Negative cost value is detected. Limited to zero.");
    }
    ROS_DEBUG("Cost estimation cache search queue initial size: %lu, capacity: %lu",
              dm_debug.search_queue_size, dm_debug.search_queue_cap);
    ROS_DEBUG("Cost estimation cache updated (%0.4f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    publishDebug();
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

    ec_ = Astar::Vecf(
        1.0f / cc_.max_vel_,
        1.0f / cc_.max_vel_,
        1.0f * cc_.weight_ang_vel_ / cc_.max_ang_vel_);

    if (map_info_.linear_resolution != msg->info.linear_resolution ||
        map_info_.angular_resolution != msg->info.angular_resolution)
    {
      map_info_ = msg->info;

      range_ = static_cast<int>(search_range_ / map_info_.linear_resolution);
      hist_ignore_range_ = std::lround(hist_ignore_range_f_ / map_info_.linear_resolution);
      hist_ignore_range_max_ = std::lround(hist_ignore_range_max_f_ / map_info_.linear_resolution);
      local_range_ = std::lround(local_range_f_ / map_info_.linear_resolution);
      esc_range_ = std::lround(esc_range_f_ / map_info_.linear_resolution);
      esc_angle_ = map_info_.angle / 8;
      tolerance_range_ = std::lround(tolerance_range_f_ / map_info_.linear_resolution);
      tolerance_angle_ = std::lround(tolerance_angle_f_ / map_info_.angular_resolution);
      goal_tolerance_lin_ = std::lround(goal_tolerance_lin_f_ / map_info_.linear_resolution);
      goal_tolerance_ang_ = std::lround(goal_tolerance_ang_f_ / map_info_.angular_resolution);
      cc_.angle_resolution_aspect_ = 2.0 / tanf(map_info_.angular_resolution);

      model_.reset(
          new GridAstarModel3D(
              map_info_,
              ec_,
              local_range_,
              cost_estim_cache_.gridmap(), cm_, cm_hyst_, cm_rough_,
              cc_, range_));

      ROS_DEBUG("Search model updated");
    }
    else
    {
      map_info_ = msg->info;
    }
    map_header_ = msg->header;
    jump_.setMapFrame(map_header_.frame_id);

    const int size[3] =
        {
            static_cast<int>(map_info_.width),
            static_cast<int>(map_info_.height),
            static_cast<int>(map_info_.angle),
        };

    const Astar::Vec size3d(size[0], size[1], size[2]);
    const Astar::Vec size2d(size[0], size[1], 1);

    as_.reset(size3d);
    cm_.reset(size3d);
    cm_hyst_.reset(size3d);

    const DistanceMap::Params dmp =
        {
            .euclid_cost = ec_,
            .range = range_,
            .local_range = local_range_,
            .longcut_range = static_cast<int>(std::lround(longcut_range_f_ / map_info_.linear_resolution)),
            .size = size2d,
            .resolution = map_info_.linear_resolution,
        };
    cost_estim_cache_.init(model_, dmp);
    cm_rough_.reset(size2d);
    cm_updates_.reset(size2d);
    bbf_costmap_.reset(size2d);

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

    cm_hyst_.clear(100);
    hyst_updated_cells_.clear();
    has_hysteresis_map_ = false;

    has_map_ = true;

    cm_rough_base_ = cm_rough_;
    cm_base_ = cm_;
    bbf_costmap_.clear();

    prev_map_update_x_min_ = std::numeric_limits<int>::max();
    prev_map_update_x_max_ = std::numeric_limits<int>::lowest();
    prev_map_update_y_min_ = std::numeric_limits<int>::max();
    prev_map_update_y_max_ = std::numeric_limits<int>::lowest();

    updateGoal();
  }
  void cbAction()
  {
    if (act_tolerant_->isActive())
    {
      ROS_ERROR("Setting new goal is ignored since planner_3d is proceeding by tolerant_move action.");
      return;
    }

    move_base_msgs::MoveBaseGoalConstPtr goal = act_->acceptNewGoal();
    if (!setGoal(goal->target_pose))
      act_->setAborted(move_base_msgs::MoveBaseResult(), "Given goal is invalid.");
  }

  void cbTolerantAction()
  {
    if (act_->isActive())
    {
      ROS_ERROR("Setting new goal is ignored since planner_3d is proceeding by move_base action.");
      return;
    }

    goal_tolerant_ = act_tolerant_->acceptNewGoal();
    if (!setGoal(goal_tolerant_->target_pose))
      act_tolerant_->setAborted(planner_cspace_msgs::MoveWithToleranceResult(), "Given goal is invalid.");
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
    , cost_estim_cache_(cm_rough_, bbf_costmap_)
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
    pub_start_ = pnh_.advertise<geometry_msgs::PoseStamped>("path_start", 1, true);
    pub_end_ = pnh_.advertise<geometry_msgs::PoseStamped>("path_end", 1, true);
    pub_status_ = pnh_.advertise<planner_cspace_msgs::PlannerStatus>("status", 1, true);
    srs_forget_ = neonavigation_common::compat::advertiseService(
        nh_, "forget_planning_cost",
        pnh_, "forget", &Planner3dNode::cbForget, this);
    srs_make_plan_ = pnh_.advertiseService("make_plan", &Planner3dNode::cbMakePlan, this);

    // Debug outputs
    pub_distance_map_ = pnh_.advertise<sensor_msgs::PointCloud>("distance_map", 1, true);
    pub_hysteresis_map_ = pnh_.advertise<nav_msgs::OccupancyGrid>("hysteresis_map", 1, true);
    pub_remembered_map_ = pnh_.advertise<nav_msgs::OccupancyGrid>("remembered_map", 1, true);

    act_.reset(new Planner3DActionServer(ros::NodeHandle(), "move_base", false));
    act_->registerGoalCallback(boost::bind(&Planner3dNode::cbAction, this));
    act_->registerPreemptCallback(boost::bind(&Planner3dNode::cbPreempt, this));

    act_tolerant_.reset(new Planner3DTolerantActionServer(ros::NodeHandle(), "tolerant_move", false));
    act_tolerant_->registerGoalCallback(boost::bind(&Planner3dNode::cbTolerantAction, this));
    act_tolerant_->registerPreemptCallback(boost::bind(&Planner3dNode::cbPreempt, this));
    goal_tolerant_ = nullptr;

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

    pnh_.param("freq", freq_, 4.0f);
    pnh_.param("freq_min", freq_min_, 2.0f);
    pnh_.param("search_timeout_abort", search_timeout_abort_, 30.0f);
    pnh_.param("search_range", search_range_, 0.4f);
    pnh_.param("antialias_start", antialias_start_, false);

    double costmap_watchdog;
    pnh_.param("costmap_watchdog", costmap_watchdog, 0.0);
    costmap_watchdog_ = ros::Duration(costmap_watchdog);

    pnh_.param("max_vel", cc_.max_vel_, 0.3f);
    pnh_.param("max_ang_vel", cc_.max_ang_vel_, 0.6f);
    pnh_.param("min_curve_radius", cc_.min_curve_radius_, 0.1f);

    pnh_.param("weight_decel", cc_.weight_decel_, 50.0f);
    pnh_.param("weight_backward", cc_.weight_backward_, 0.9f);
    pnh_.param("weight_ang_vel", cc_.weight_ang_vel_, 1.0f);
    pnh_.param("weight_costmap", cc_.weight_costmap_, 50.0f);
    pnh_.param("weight_costmap_turn", cc_.weight_costmap_turn_, 0.0f);
    pnh_.param("weight_remembered", cc_.weight_remembered_, 1000.0f);
    pnh_.param("cost_in_place_turn", cc_.in_place_turn_, 30.0f);
    pnh_.param("hysteresis_max_dist", cc_.hysteresis_max_dist_, 0.1f);
    pnh_.param("hysteresis_expand", cc_.hysteresis_expand_, 0.1f);
    pnh_.param("weight_hysteresis", cc_.weight_hysteresis_, 5.0f);

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

    pnh_.param("sw_wait", sw_wait_, 2.0f);
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
    if (pnh_.hasParam("debug_mode"))
    {
      ROS_ERROR(
          "planner_3d: ~/debug_mode parameter and ~/debug topic are deprecated. "
          "Use ~/distance_map, ~/hysteresis_map, and ~/remembered_map topics instead.");
    }

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

    int num_task;
    pnh_.param("num_search_task", num_task, num_threads * 16);
    as_.setSearchTaskNum(num_task);
    int num_cost_estim_task;
    pnh_.param("num_cost_estim_task", num_cost_estim_task, num_threads * 16);
    cost_estim_cache_.setParams(cc_, num_cost_estim_task);

    pnh_.param("retain_last_error_status", retain_last_error_status_, true);
    status_.status = planner_cspace_msgs::PlannerStatus::DONE;

    has_map_ = false;
    has_goal_ = false;
    has_start_ = false;
    goal_updated_ = false;

    escaping_ = false;
    cnt_stuck_ = 0;
    is_path_switchback_ = false;

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Path Planner Status", this, &Planner3dNode::diagnoseStatus);

    act_->start();
    act_tolerant_->start();
  }

  GridAstarModel3D::Vec pathPose2Grid(const geometry_msgs::PoseStamped& pose) const
  {
    GridAstarModel3D::Vec grid_vec;
    grid_metric_converter::metric2Grid(map_info_, grid_vec[0], grid_vec[1], grid_vec[2],
                                       pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
    grid_vec.cycleUnsigned(map_info_.angle);
    return grid_vec;
  }

  void waitUntil(const ros::Time& next_replan_time, const nav_msgs::Path& previous_path)
  {
    while (ros::ok())
    {
      const ros::Time prev_map_update_stamp = last_costmap_;
      ros::spinOnce();
      const bool costmap_udpated = last_costmap_ != prev_map_update_stamp;

      if (has_map_)
      {
        updateStart();

        if (jump_.detectJump())
        {
          bbf_costmap_.clear();
          // Robot pose jumped.
          return;
        }

        if (costmap_udpated && previous_path.poses.size() > 1)
        {
          for (const auto& path_pose : previous_path.poses)
          {
            if (cm_[pathPose2Grid(path_pose)] == 100)
            {
              // Obstacle on the path.
              return;
            }
          }
        }

        if (is_path_switchback_)
        {
          const float len = std::hypot(
              start_.pose.position.y - sw_pos_.pose.position.y,
              start_.pose.position.x - sw_pos_.pose.position.x);
          const float yaw = tf2::getYaw(start_.pose.orientation);
          const float sw_yaw = tf2::getYaw(sw_pos_.pose.orientation);
          float yaw_diff = yaw - sw_yaw;
          yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
          if (len < goal_tolerance_lin_f_ && std::fabs(yaw_diff) < goal_tolerance_ang_f_)
          {
            // robot has arrived at the switchback point
            is_path_switchback_ = false;
          }
        }
      }
      if (ros::Time::now() > next_replan_time)
      {
        return;
      }
      ros::Duration(0.01).sleep();
    }
  }

  void spin()
  {
    ROS_DEBUG("Initialized");

    ros::Time next_replan_time = ros::Time::now();
    nav_msgs::Path previous_path;

    while (ros::ok())
    {
      waitUntil(next_replan_time, previous_path);

      const ros::Time now = ros::Time::now();
      next_replan_time = now;

      if (has_map_ && !goal_updated_ && has_goal_)
      {
        updateGoal();
      }
      bool has_costmap(false);
      if (costmap_watchdog_ > ros::Duration(0))
      {
        if (last_costmap_ + costmap_watchdog_ < now)
        {
          ROS_WARN_THROTTLE(1.0,
                            "Navigation is stopping since the costmap is too old (costmap: %0.3f)",
                            last_costmap_.toSec());
          status_.error = planner_cspace_msgs::PlannerStatus::DATA_MISSING;
          publishEmptyPath();
          previous_path.poses.clear();
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

      bool is_path_switchback = false;
      if (has_map_ && has_goal_ && has_start_ && has_costmap)
      {
        if (act_->isActive())
        {
          move_base_msgs::MoveBaseFeedback feedback;
          feedback.base_position = start_;
          act_->publishFeedback(feedback);
        }

        if (act_tolerant_->isActive())
        {
          planner_cspace_msgs::MoveWithToleranceFeedback feedback;
          feedback.base_position = start_;
          act_tolerant_->publishFeedback(feedback);
        }

        if (status_.status == planner_cspace_msgs::PlannerStatus::FINISHING)
        {
          const float yaw_s = tf2::getYaw(start_.pose.orientation);
          float yaw_g = tf2::getYaw(goal_.pose.orientation);
          if (force_goal_orientation_)
            yaw_g = tf2::getYaw(goal_raw_.pose.orientation);

          float yaw_diff = yaw_s - yaw_g;
          if (yaw_diff > M_PI)
            yaw_diff -= M_PI * 2.0;
          else if (yaw_diff < -M_PI)
            yaw_diff += M_PI * 2.0;
          if (std::abs(yaw_diff) <
              (act_tolerant_->isActive() ? goal_tolerant_->goal_tolerance_ang_finish : goal_tolerance_ang_finish_))
          {
            status_.status = planner_cspace_msgs::PlannerStatus::DONE;
            has_goal_ = false;
            // Don't publish empty path here in order a path follower
            // to minimize the error to the desired final pose
            ROS_INFO("Path plan finished");

            if (act_->isActive())
              act_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
            if (act_tolerant_->isActive())
              act_tolerant_->setSucceeded(planner_cspace_msgs::MoveWithToleranceResult(), "Goal reached.");
          }
          else
          {
            publishFinishPath();
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

            publishEmptyPath();
            previous_path.poses.clear();
            next_replan_time += ros::Duration(1.0 / freq_);
            ROS_ERROR("Exceeded max_retry_num:%d", max_retry_num_);

            if (act_->isActive())
              act_->setAborted(
                  move_base_msgs::MoveBaseResult(), "Goal is in Rock");
            if (act_tolerant_->isActive())
              act_tolerant_->setAborted(
                  planner_cspace_msgs::MoveWithToleranceResult(), "Goal is in Rock");

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
          previous_path = path;

          if (sw_wait_ > 0.0)
          {
            const int sw_index = getSwitchIndex(path);
            is_path_switchback = (sw_index >= 0);
            if (is_path_switchback)
              sw_pos_ = path.poses[sw_index];
          }
        }
      }
      else if (!has_goal_)
      {
        if (!retain_last_error_status_)
          status_.error = planner_cspace_msgs::PlannerStatus::GOING_WELL;
        publishEmptyPath();
        previous_path.poses.clear();
      }
      pub_status_.publish(status_);
      diag_updater_.force_update();

      if (is_path_switchback)
      {
        next_replan_time += ros::Duration(sw_wait_);
        ROS_INFO("Planned path has switchback. Planner will stop until: %f at the latest.", next_replan_time.toSec());
      }
      else
      {
        next_replan_time += ros::Duration(1.0 / freq_);
      }
      is_path_switchback_ = is_path_switchback;
    }
  }

protected:
  bool makePlan(const geometry_msgs::Pose& gs, const geometry_msgs::Pose& ge,
                nav_msgs::Path& path, bool hyst)
  {
    Astar::Vec e;
    grid_metric_converter::metric2Grid(
        map_info_, e[0], e[1], e[2],
        ge.position.x, ge.position.y, tf2::getYaw(ge.orientation));
    e.cycleUnsigned(map_info_.angle);

    Astar::Vecf sf;
    grid_metric_converter::metric2Grid(
        map_info_, sf[0], sf[1], sf[2],
        gs.position.x, gs.position.y, tf2::getYaw(gs.orientation));
    Astar::Vec s(static_cast<int>(std::floor(sf[0])), static_cast<int>(std::floor(sf[1])), std::lround(sf[2]));
    s.cycleUnsigned(map_info_.angle);
    if (!cm_.validate(s, range_))
    {
      ROS_ERROR("You are on the edge of the world.");
      return false;
    }

    std::vector<Astar::VecWithCost> starts;
    if (antialias_start_)
    {
      const int x_cand[] = {0, ((sf[0] - s[0]) < 0.5 ? -1 : 1)};
      const int y_cand[] = {0, ((sf[1] - s[1]) < 0.5 ? -1 : 1)};
      for (const int x : x_cand)
      {
        for (const int y : y_cand)
        {
          const Astar::Vec p = s + Astar::Vec(x, y, 0);
          if (!cm_.validate(p, range_))
            continue;

          const Astar::Vecf subpx = sf - Astar::Vecf(p[0] + 0.5f, p[1] + 0.5f, 0.0f);
          if (subpx.sqlen() > 1.0)
            continue;
          if (cm_[p] > 99)
            continue;

          starts.push_back(Astar::VecWithCost(p));
        }
      }
    }
    else
    {
      if (cm_[s] < 100)
      {
        starts.push_back(Astar::VecWithCost(s));
      }
    }
    for (Astar::VecWithCost& s : starts)
    {
      const Astar::Vecf diff =
          Astar::Vecf(s.v_[0] + 0.5f, s.v_[1] + 0.5f, 0.0f) - sf;
      s.c_ = std::hypot(diff[0] * ec_[0], diff[1] * ec_[0]);
      s.c_ += cm_[s.v_] * cc_.weight_costmap_ / 100.0;

      // Check if arrived to the goal
      Astar::Vec remain = s.v_ - e;
      remain.cycle(map_info_.angle);

      int g_tolerance_lin, g_tolerance_ang;
      if (act_tolerant_->isActive())
      {
        g_tolerance_lin = std::lround(goal_tolerant_->goal_tolerance_lin / map_info_.linear_resolution);
        g_tolerance_ang = std::lround(goal_tolerant_->goal_tolerance_ang / map_info_.angular_resolution);
      }
      else
      {
        g_tolerance_lin = goal_tolerance_lin_;
        g_tolerance_ang = goal_tolerance_ang_;
      }

      if (remain.sqlen() <= g_tolerance_lin * g_tolerance_lin &&
          std::abs(remain[2]) <= g_tolerance_ang)
      {
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
          publishFinishPath();
          ROS_INFO("Path plan finishing");
        }
        return true;
      }
    }

    geometry_msgs::PoseStamped p;
    p.header = map_header_;
    float x, y, yaw;
    grid_metric_converter::grid2Metric(map_info_, e[0], e[1], e[2], x, y, yaw);
    p.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
    p.pose.position.x = x;
    p.pose.position.y = y;
    pub_end_.publish(p);

    if (starts.size() == 0)
    {
      if (!searchAvailablePos(cm_, s, tolerance_range_, tolerance_angle_))
      {
        ROS_WARN("Oops! You are in Rock!");
        status_.error = planner_cspace_msgs::PlannerStatus::IN_ROCK;
        return false;
      }
      ROS_INFO("Start moved");
      starts.push_back(Astar::VecWithCost(s));
    }
    const Astar::Vec s_rough(s[0], s[1], 0);

    // If goal gets occupied, cost_estim_cache_ is not updated to reduce
    // computational cost for clearing huge map. In this case, cm_[e] is 100.
    if (cost_estim_cache_[s_rough] == std::numeric_limits<float>::max() || cm_[e] >= 100)
    {
      status_.error = planner_cspace_msgs::PlannerStatus::PATH_NOT_FOUND;
      ROS_WARN("Goal unreachable.");
      if (!escaping_ && temporary_escape_)
      {
        e = s;
        if (searchAvailablePos(cm_, e, esc_range_, esc_angle_, 50, esc_range_ / 2))
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

    const float range_limit = cost_estim_cache_[s_rough] - (local_range_ + range_) * ec_[0];

    const auto ts = boost::chrono::high_resolution_clock::now();
    // ROS_INFO("Planning from (%d, %d, %d) to (%d, %d, %d)",
    //   s[0], s[1], s[2], e[0], e[1], e[2]);

    const auto cb_progress = [this, ts, s, e](const std::list<Astar::Vec>& path_grid, const SearchStats& stats) -> bool
    {
      const auto tnow = boost::chrono::high_resolution_clock::now();
      const auto tdiff = boost::chrono::duration<float>(tnow - ts).count();
      publishEmptyPath();
      if (tdiff > search_timeout_abort_)
      {
        ROS_ERROR(
            "Search aborted due to timeout. "
            "search_timeout_abort may be too small or planner_3d may have a bug: "
            "s=(%d, %d, %d), g=(%d, %d, %d), tdiff=%0.4f, "
            "num_loop=%lu, "
            "num_search_queue=%lu, "
            "num_prev_updates=%lu, "
            "num_total_updates=%lu, ",
            s[0], s[1], s[2], e[0], e[1], e[2], tdiff,
            stats.num_loop, stats.num_search_queue, stats.num_prev_updates, stats.num_total_updates);
        return false;
      }
      ROS_WARN("Search timed out (%0.4f sec.)", tdiff);
      return true;
    };

    model_->enableHysteresis(hyst && has_hysteresis_map_);
    std::list<Astar::Vec> path_grid;
    if (!as_.search(starts, e, path_grid,
                    model_,
                    cb_progress,
                    range_limit,
                    1.0f / freq_min_,
                    find_best_))
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

    const std::list<Astar::Vecf> path_interpolated =
        model_->path_interpolator_.interpolate(path_grid, 0.5, local_range_);
    grid_metric_converter::grid2MetricPath(map_info_, path_interpolated, path);

    if (hyst)
    {
      const auto ts = boost::chrono::high_resolution_clock::now();
      std::unordered_map<Astar::Vec, bool, Astar::Vec> path_points;
      const float max_dist = cc_.hysteresis_max_dist_ / map_info_.linear_resolution;
      const float expand_dist = cc_.hysteresis_expand_ / map_info_.linear_resolution;
      const int path_range = range_ + max_dist + expand_dist + 5;
      for (const Astar::Vecf& p : path_interpolated)
      {
        Astar::Vec d;
        for (d[0] = -path_range; d[0] <= path_range; d[0]++)
        {
          for (d[1] = -path_range; d[1] <= path_range; d[1]++)
          {
            Astar::Vec point = p + d;
            point.cycleUnsigned(map_info_.angle);
            if ((unsigned int)point[0] >= (unsigned int)map_info_.width ||
                (unsigned int)point[1] >= (unsigned int)map_info_.height)
              continue;
            path_points[point] = true;
          }
        }
      }

      clearHysteresis();
      for (auto& ps : path_points)
      {
        const Astar::Vec& p = ps.first;
        float d_min = std::numeric_limits<float>::max();
        auto it_prev = path_interpolated.cbegin();
        for (auto it = path_interpolated.cbegin(); it != path_interpolated.cend(); it++)
        {
          if (it != it_prev)
          {
            int yaw = std::lround((*it)[2]) % map_info_.angle;
            int yaw_prev = std::lround((*it_prev)[2]) % map_info_.angle;
            if (yaw < 0)
              yaw += map_info_.angle;
            if (yaw_prev < 0)
              yaw_prev += map_info_.angle;
            if (yaw == p[2] || yaw_prev == p[2])
            {
              const float d =
                  CyclicVecFloat<3, 2>(p).distLinestrip2d(*it_prev, *it);
              if (d < d_min)
                d_min = d;
            }
          }
          it_prev = it;
        }
        d_min = std::max(expand_dist, std::min(expand_dist + max_dist, d_min));
        cm_hyst_[p] = std::lround((d_min - expand_dist) * 100.0 / max_dist);
        hyst_updated_cells_.push_back(p);
      }
      has_hysteresis_map_ = true;
      const auto tnow = boost::chrono::high_resolution_clock::now();
      ROS_DEBUG("Hysteresis map generated (%0.4f sec.)",
                boost::chrono::duration<float>(tnow - ts).count());
      publishDebug();
    }

    return true;
  }

  int getSwitchIndex(const nav_msgs::Path& path) const
  {
    geometry_msgs::Pose p_prev;
    bool first(true);
    bool dir_set(false);
    bool dir_prev(false);
    for (auto it = path.poses.begin(); it != path.poses.end(); ++it)
    {
      const auto& p = *it;
      if (!first)
      {
        const float x_diff = p.pose.position.x - p_prev.position.x;
        const float y_diff = p.pose.position.y - p_prev.position.y;
        const float len_sq = std::pow(y_diff, 2) + std::pow(x_diff, 2);
        if (len_sq > std::pow(0.001f, 2))
        {
          const float yaw = tf2::getYaw(p.pose.orientation);
          const bool dir = (std::cos(yaw) * x_diff + std::sin(yaw) * y_diff < 0);

          if (dir_set && (dir_prev ^ dir))
          {
            return std::distance(path.poses.begin(), it);
          }
          dir_prev = dir;
          dir_set = true;
        }
      }
      first = false;
      p_prev = p.pose;
    }
    // -1 means no switchback in the path
    return -1;
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
}  // namespace planner_3d
}  // namespace planner_cspace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "planner_3d");

  planner_cspace::planner_3d::Planner3dNode node;
  node.spin();

  return 0;
}
