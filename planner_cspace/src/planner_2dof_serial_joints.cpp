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
#include <planner_cspace/PlannerStatus.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <utility>
#include <algorithm>
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

class planner2dofSerialJoints
{
public:
  using Astar = GridAstar<2, 2>;

private:
  ros::Publisher pub_status_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_joint_;

  tf::TransformListener tfl_;

  Astar as_;
  Astar::Gridmap<char, 0x40> cm_;

  Astar::Vecf euclid_cost_coef_;

  float euclid_cost(const Astar::Vec &v, const Astar::Vecf coef)
  {
    auto vc = v;
    float cost = 0;
    for (int i = 0; i < as_.getDim(); i++)
    {
      // vc.cycle(vc[i], cm_.size[i]);
      cost += fabs(coef[i] * vc[i]);
    }
    return cost;
  }
  float euclid_cost(const Astar::Vec &v)
  {
    return euclid_cost(v, euclid_cost_coef_);
  }

  float freq_;
  float freq_min_;
  bool has_goal_;
  bool has_start_;
  std::vector<Astar::Vec> search_list_;
  int resolution_;
  float weight_cost_;
  float expand_;
  float avg_vel_;
  enum point_vel_mode
  {
    VEL_PREV,
    VEL_NEXT,
    VEL_AVG
  };
  point_vel_mode point_vel_;

  std::string group_;

  bool debug_aa_;

  class link_body
  {
  protected:
    class vec3dof
    {
    public:
      float x_;
      float y_;
      float th_;

      float dist(const vec3dof &b)
      {
        return hypotf(b.x_ - x_, b.y_ - y_);
      }
    };

  public:
    float radius_[2];
    float vmax_;
    float length_;
    std::string name_;
    vec3dof origin_;
    vec3dof gain_;
    float current_th_;

    link_body()
    {
      gain_.x_ = 1.0;
      gain_.y_ = 1.0;
      gain_.th_ = 1.0;
    }
    vec3dof end(const float th) const
    {
      vec3dof e = origin_;
      e.x_ += cosf(e.th_ + th * gain_.th_) * length_;
      e.y_ += sinf(e.th_ + th * gain_.th_) * length_;
      e.th_ += th;
      return e;
    }
    bool isCollide(const link_body b, const float th0, const float th1)
    {
      auto end0 = end(th0);
      auto end1 = b.end(th1);
      auto &end0r = radius_[1];
      auto &end1r = b.radius_[1];
      auto &origin0 = origin_;
      auto &origin1 = b.origin_;
      auto &origin0r = radius_[0];
      auto &origin1r = b.radius_[0];

      if (end0.dist(end1) < end0r + end1r)
        return true;
      if (end0.dist(origin1) < end0r + origin1r)
        return true;
      if (end1.dist(origin0) < end1r + origin0r)
        return true;

      // add side collision

      return false;
    }
  };
  link_body links_[2];

  planner_cspace::PlannerStatus status_;
  sensor_msgs::JointState joint_;
  ros::Time replan_prev_;
  ros::Duration replan_interval_;

  void cb_joint(const sensor_msgs::JointState::ConstPtr &msg)
  {
    int id[2] = { -1, -1 };
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      if (msg->name[i].compare(links_[0].name_) == 0)
        id[0] = i;
      else if (msg->name[i].compare(links_[1].name_) == 0)
        id[1] = i;
    }
    if (id[0] == -1 || id[1] == -1)
    {
      ROS_ERROR("joint_state does not contain link group_ %s.", group_.c_str());
      return;
    }
    links_[0].current_th_ = msg->position[id[0]];
    links_[1].current_th_ = msg->position[id[1]];

    if (replan_prev_ + replan_interval_ < ros::Time::now() &&
        replan_interval_ > ros::Duration(0) &&
        replan_prev_ != ros::Time(0))
    {
      replan();
    }
  }
  std::pair<ros::Duration, std::pair<float, float>> cmd_prev;
  trajectory_msgs::JointTrajectory traj_prev;
  int id[2];
  void cb_trajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
  {
    id[0] = -1;
    id[1] = -1;
    for (size_t i = 0; i < msg->joint_names.size(); i++)
    {
      if (msg->joint_names[i].compare(links_[0].name_) == 0)
        id[0] = i;
      else if (msg->joint_names[i].compare(links_[1].name_) == 0)
        id[1] = i;
    }
    if (id[0] == -1 || id[1] == -1)
    {
      ROS_ERROR("joint_trajectory does not contains link group_ %s.", group_.c_str());
      return;
    }
    if (msg->points.size() != 1)
    {
      ROS_ERROR("single trajectory point required.");
    }
    decltype(cmd_prev) cmd;
    cmd.first = msg->points[0].time_from_start;
    cmd.second.first = msg->points[0].positions[id[0]];
    cmd.second.second = msg->points[0].positions[id[1]];
    if (cmd_prev == cmd)
      return;
    cmd_prev = cmd;
    traj_prev = *msg;
    avg_vel_ = -1.0;

    replan();
  }
  void replan()
  {
    replan_prev_ = ros::Time::now();
    if (id[0] == -1 || id[1] == -1)
      return;

    float st[2] =
        {
          links_[0].current_th_,
          links_[1].current_th_
        };
    float en[2] =
        {
          static_cast<float>(traj_prev.points[0].positions[id[0]]),
          static_cast<float>(traj_prev.points[0].positions[id[1]])
        };
    Astar::Vecf start(st);
    Astar::Vecf end(en);

    ROS_INFO("link %s: %0.3f, %0.3f", group_.c_str(),
             traj_prev.points[0].positions[id[0]],
             traj_prev.points[0].positions[id[1]]);

    ROS_INFO("Start searching");
    std::list<Astar::Vecf> path;
    if (make_plan(start, end, path))
    {
      ROS_INFO("Trajectory found");

      if (avg_vel_ < 0)
      {
        float pos_sum = 0;
        for (auto it = path.begin(); it != path.end(); it++)
        {
          auto it_next = it;
          it_next++;
          if (it_next != path.end())
          {
            float diff[2], diff_max;
            diff[0] = fabs((*it_next)[0] - (*it)[0]);
            diff[1] = fabs((*it_next)[1] - (*it)[1]);
            diff_max = std::max(diff[0], diff[1]);
            pos_sum += diff_max;
          }
        }
        if (traj_prev.points[0].time_from_start <= ros::Duration(0))
        {
          avg_vel_ = std::min(links_[0].vmax_, links_[1].vmax_);
        }
        else
        {
          avg_vel_ = pos_sum / traj_prev.points[0].time_from_start.toSec();
          if (avg_vel_ > links_[0].vmax_)
            avg_vel_ = links_[0].vmax_;
          if (avg_vel_ > links_[1].vmax_)
            avg_vel_ = links_[1].vmax_;
        }
      }

      trajectory_msgs::JointTrajectory out;
      out.header = traj_prev.header;
      out.header.stamp = ros::Time(0);
      out.joint_names.resize(2);
      out.joint_names[0] = links_[0].name_;
      out.joint_names[1] = links_[1].name_;
      float pos_sum = 0.0;
      for (auto it = path.begin(); it != path.end(); it++)
      {
        if (it == path.begin())
          continue;

        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.resize(2);
        p.velocities.resize(2);

        auto it_prev = it;
        it_prev--;
        auto it_next = it;
        it_next++;

        float diff[2], diff_max;
        diff[0] = fabs((*it)[0] - (*it_prev)[0]);
        diff[1] = fabs((*it)[1] - (*it_prev)[1]);
        diff_max = std::max(diff[0], diff[1]);
        pos_sum += diff_max;

        if (it_next == path.end())
        {
          p.velocities[0] = 0.0;
          p.velocities[1] = 0.0;
        }
        else
        {
          float dir[2], dir_max;
          switch (point_vel_)
          {
            default:
            case VEL_PREV:
              dir[0] = ((*it)[0] - (*it_prev)[0]);
              dir[1] = ((*it)[1] - (*it_prev)[1]);
              break;
            case VEL_NEXT:
              dir[0] = ((*it_next)[0] - (*it)[0]);
              dir[1] = ((*it_next)[1] - (*it)[1]);
              break;
            case VEL_AVG:
              dir[0] = ((*it_next)[0] - (*it_prev)[0]);
              dir[1] = ((*it_next)[1] - (*it_prev)[1]);
              break;
          }
          dir_max = std::max(fabs(dir[0]), fabs(dir[1]));
          float t = dir_max / avg_vel_;

          p.velocities[0] = dir[0] / t;
          p.velocities[1] = dir[1] / t;
        }
        p.time_from_start = ros::Duration(pos_sum / avg_vel_);
        p.positions[0] = (*it)[0];
        p.positions[1] = (*it)[1];
        out.points.push_back(p);
      }
      pub_trajectory_.publish(out);
    }
    else
    {
      trajectory_msgs::JointTrajectory out;
      out.header = traj_prev.header;
      out.header.stamp = ros::Time(0);
      out.joint_names.resize(2);
      out.joint_names[0] = links_[0].name_;
      out.joint_names[1] = links_[1].name_;
      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.resize(2);
      p.positions[0] = links_[0].current_th_;
      p.positions[1] = links_[1].current_th_;
      p.velocities.resize(2);
      out.points.push_back(p);
      pub_trajectory_.publish(out);

      ROS_WARN("Trajectory not found");
    }

    pub_status_.publish(status_);
  }

public:
  explicit planner2dofSerialJoints(const std::string group_name)
  {
    group_ = group_name;
    ros::NodeHandle_f nh("~/" + group_);
    ros::NodeHandle_f nh_home("~");

    pub_trajectory_ = nh_home.advertise<trajectory_msgs::JointTrajectory>("trajectory_out", 1, true);
    sub_trajectory_ = nh_home.subscribe("trajectory_in", 1, &planner2dofSerialJoints::cb_trajectory, this);
    sub_joint_ = nh_home.subscribe("joint_", 1, &planner2dofSerialJoints::cb_joint, this);

    pub_status_ = nh.advertise<planner_cspace::PlannerStatus>("status_", 1, true);

    nh.param("resolution_", resolution_, 128);
    nh_home.param("debug_aa_", debug_aa_, false);

    double interval;
    nh_home.param("replan_interval_", interval, 0.2);
    replan_interval_ = ros::Duration(interval);
    replan_prev_ = ros::Time(0);

    int queue_size_limit;
    nh.param("queue_size_limit", queue_size_limit, 0);
    as_.setQueueSizeLimit(queue_size_limit);

    status_.status = planner_cspace::PlannerStatus::DONE;

    has_goal_ = false;
    has_start_ = false;

    int size[2] = { resolution_ * 2, resolution_ * 2 };
    cm_.reset(Astar::Vec(size));
    as_.reset(Astar::Vec(size));
    cm_.clear(0);

    nh.param("link0_name", links_[0].name_, std::string("link0"));
    nh.param_cast("link0_joint_radius", links_[0].radius_[0], 0.07f);
    nh.param_cast("link0_end_radius", links_[0].radius_[1], 0.07f);
    nh.param_cast("link0_length", links_[0].length_, 0.135);
    nh.param_cast("link0_x", links_[0].origin_.x_, 0.22f);
    nh.param_cast("link0_y", links_[0].origin_.y_, 0.0f);
    nh.param_cast("link0_th", links_[0].origin_.th_, 0.0f);
    nh.param_cast("link0_gain_th", links_[0].gain_.th_, -1.0f);
    nh.param_cast("link0_vmax", links_[0].vmax_, 0.5f);
    nh.param("link1_name", links_[1].name_, std::string("link1"));
    nh.param_cast("link1_joint_radius", links_[1].radius_[0], 0.07f);
    nh.param_cast("link1_end_radius", links_[1].radius_[1], 0.07f);
    nh.param_cast("link1_length", links_[1].length_, 0.27f);
    nh.param_cast("link1_x", links_[1].origin_.x_, -0.22f);
    nh.param_cast("link1_y", links_[1].origin_.y_, 0.0f);
    nh.param_cast("link1_th", links_[1].origin_.th_, 0.0f);
    nh.param_cast("link1_gain_th", links_[1].gain_.th_, 1.0f);
    nh.param_cast("link1_vmax", links_[1].vmax_, 0.5f);

    links_[0].current_th_ = 0.0;
    links_[1].current_th_ = 0.0;

    ROS_INFO("link group_: %s", group_.c_str());
    ROS_INFO(" - link0: %s", links_[0].name_.c_str());
    ROS_INFO(" - link1: %s", links_[1].name_.c_str());

    nh.param_cast("link0_coef", euclid_cost_coef_[0], 1.0f);
    nh.param_cast("link1_coef", euclid_cost_coef_[1], 1.5f);

    nh.param_cast("weight_cost_", weight_cost_, 4.0f);
    nh.param_cast("expand_", expand_, 0.1);

    std::string point_vel_mode;
    nh.param("point_vel_mode", point_vel_mode, std::string("prev"));
    std::transform(point_vel_mode.begin(), point_vel_mode.end(), point_vel_mode.begin(), ::tolower);
    if (point_vel_mode.compare("prev") == 0)
      point_vel_ = VEL_PREV;
    else if (point_vel_mode.compare("next") == 0)
      point_vel_ = VEL_NEXT;
    else if (point_vel_mode.compare("avg") == 0)
      point_vel_ = VEL_AVG;
    else
      ROS_ERROR("point_vel_mode must be prev/next/avg");

    ROS_INFO("Resolution: %d", resolution_);
    Astar::Vec p;
    for (p[0] = 0; p[0] < resolution_ * 2; p[0]++)
    {
      for (p[1] = 0; p[1] < resolution_ * 2; p[1]++)
      {
        Astar::Vecf pf;
        grid2metric(p, pf);

        if (links_[0].isCollide(links_[1], pf[0], pf[1]))
          cm_[p] = 100;
        // else if(pf[0] > M_PI || pf[1] > M_PI)
        //   cm_[p] = 50;
        else
          cm_[p] = 0;
      }
    }
    for (p[0] = 0; p[0] < resolution_ * 2; p[0]++)
    {
      for (p[1] = 0; p[1] < resolution_ * 2; p[1]++)
      {
        if (cm_[p] != 100)
          continue;

        Astar::Vec d;
        int range = lroundf(expand_ * resolution_ / (2.0 * M_PI));
        for (d[0] = -range; d[0] <= range; d[0]++)
        {
          for (d[1] = -range; d[1] <= range; d[1]++)
          {
            Astar::Vec p2 = p + d;
            if ((unsigned int)p2[0] >= (unsigned int)resolution_ * 2 ||
                (unsigned int)p2[1] >= (unsigned int)resolution_ * 2)
              continue;
            int dist = std::max(abs(d[0]), abs(d[1]));
            int c = floorf(100.0 * (range - dist) / range);
            if (cm_[p2] < c)
              cm_[p2] = c;
          }
        }
      }
    }

    int range;
    nh.param("range", range, 8);
    for (p[0] = -range; p[0] <= range; p[0]++)
    {
      for (p[1] = -range; p[1] <= range; p[1]++)
      {
        search_list_.push_back(p);
      }
    }
    has_start_ = has_goal_ = true;
  }

private:
  void grid2metric(
      const int t0, const int t1,
      float &gt0, float &gt1)
  {
    gt0 = (t0 - resolution_) * 2.0 * M_PI / static_cast<float>(resolution_);
    gt1 = (t1 - resolution_) * 2.0 * M_PI / static_cast<float>(resolution_);
  }
  void metric2grid(
      int &t0, int &t1,
      const float gt0, const float gt1)
  {
    t0 = lroundf(gt0 * resolution_ / (2.0 * M_PI)) + resolution_;
    t1 = lroundf(gt1 * resolution_ / (2.0 * M_PI)) + resolution_;
  }
  void grid2metric(
      const Astar::Vec t,
      Astar::Vecf &gt)
  {
    grid2metric(t[0], t[1], gt[0], gt[1]);
  }
  void metric2grid(
      Astar::Vec &t,
      const Astar::Vecf gt)
  {
    metric2grid(t[0], t[1], gt[0], gt[1]);
  }
  bool make_plan(const Astar::Vecf sg, const Astar::Vecf eg, std::list<Astar::Vecf> &path)
  {
    Astar::Vec s, e;
    metric2grid(s, sg);
    metric2grid(e, eg);
    ROS_INFO("Planning from (%d, %d) to (%d, %d)",
             s[0], s[1], e[0], e[1]);

    if (cm_[s] == 100)
    {
      ROS_WARN("Path plan failed (current status_ is in collision)");
      status_.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      return false;
    }
    if (cm_[e] == 100)
    {
      ROS_WARN("Path plan failed (goal status_ is in collision)");
      status_.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      return false;
    }
    Astar::Vec d = e - s;
    d.cycle(d[0], resolution_);
    d.cycle(d[1], resolution_);

    if (cb_cost(s, e, e, s) >= euclid_cost(d))
    {
      path.push_back(sg);
      path.push_back(eg);
      if (s == e)
      {
        replan_prev_ = ros::Time(0);
      }
      return true;
    }
    std::list<Astar::Vec> path_grid;
    // const auto ts = std::chrono::high_resolution_clock::now();
    float cancel = FLT_MAX;
    if (replan_interval_ >= ros::Duration(0))
      cancel = replan_interval_.toSec();
    if (!as_.search(s, e, path_grid,
                    std::bind(&planner2dofSerialJoints::cb_cost,
                              this, std::placeholders::_1, std::placeholders::_2,
                              std::placeholders::_3, std::placeholders::_4),
                    std::bind(&planner2dofSerialJoints::cb_cost_estim,
                              this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&planner2dofSerialJoints::cb_search,
                              this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3),
                    std::bind(&planner2dofSerialJoints::cb_progress,
                              this, std::placeholders::_1),
                    0,
                    cancel,
                    true))
    {
      ROS_WARN("Path plan failed (goal unreachable)");
      status_.error = planner_cspace::PlannerStatus::PATH_NOT_FOUND;
      return false;
    }
    // const auto tnow = std::chrono::high_resolution_clock::now();
    // ROS_INFO("Path found (%0.3f sec.)",
    //   std::chrono::duration<float>(tnow - ts).count());

    bool first = false;
    Astar::Vec n_prev = s;
    path.push_back(sg);
    int i = 0;
    for (auto &n : path_grid)
    {
      if (!first)
      {
        first = true;
        continue;
      }
      if (i == 0)
        ROS_INFO("  next: %d, %d", n[0], n[1]);
      Astar::Vec n_diff = n - n_prev;
      n_diff.cycle(n_diff[0], resolution_);
      n_diff.cycle(n_diff[1], resolution_);
      Astar::Vec n2 = n_prev + n_diff;
      n_prev = n2;

      Astar::Vecf p;
      grid2metric(n2, p);
      path.push_back(p);
      i++;
    }
    float prec = 2.0 * M_PI / static_cast<float>(resolution_);
    Astar::Vecf egp = eg;
    if (egp[0] < 0)
      egp[0] += ceilf(-egp[0] / M_PI * 2.0) * M_PI * 2.0;
    if (egp[1] < 0)
      egp[1] += ceilf(-egp[1] / M_PI * 2.0) * M_PI * 2.0;
    path.back()[0] += fmod(egp[0] + prec / 2.0, prec) - prec / 2.0;
    path.back()[1] += fmod(egp[1] + prec / 2.0, prec) - prec / 2.0;

    if (debug_aa_)
    {
      Astar::Vec p;
      for (p[0] = resolution_ / 2; p[0] < resolution_ * 3 / 2; p[0]++)
      {
        for (p[1] = resolution_ / 2; p[1] < resolution_ * 3 / 2; p[1]++)
        {
          bool found = false;
          for (auto &g : path_grid)
          {
            if (g == p)
              found = true;
          }
          if (p == s)
            printf("\033[31ms\033[0m");
          else if (p == e)
            printf("\033[31me\033[0m");
          else if (found)
            printf("\033[34m*\033[0m");
          else
            printf("%d", cm_[p] / 11);
        }
        printf("\n");
      }
      printf("\n");
    }

    return true;
  }
  std::vector<Astar::Vec> &cb_search(
      const Astar::Vec &p,
      const Astar::Vec &s, const Astar::Vec &e)
  {
    return search_list_;
  }
  bool cb_progress(const std::list<Astar::Vec> &path_grid)
  {
    return false;
  }
  float cb_cost_estim(const Astar::Vec &s, const Astar::Vec &e)
  {
    const Astar::Vec d = e - s;
    return euclid_cost(d);
  }
  float cb_cost(const Astar::Vec &s, Astar::Vec &e,
                const Astar::Vec &v_goal,
                const Astar::Vec &v_start)
  {
    if ((unsigned int)e[0] >= (unsigned int)resolution_ * 2 ||
        (unsigned int)e[1] >= (unsigned int)resolution_ * 2)
      return -1;
    Astar::Vec d = e - s;
    d.cycle(d[0], resolution_);
    d.cycle(d[1], resolution_);

    float cost = euclid_cost(d);

    float distf = hypotf(static_cast<float>(d[0]), static_cast<float>(d[1]));
    float v[2], dp[2];
    int sum = 0;
    const int dist = distf;
    distf /= dist;
    v[0] = s[0];
    v[1] = s[1];
    dp[0] = static_cast<float>(d[0]) / dist;
    dp[1] = static_cast<float>(d[1]) / dist;
    Astar::Vec pos;
    for (int i = 0; i < dist; i++)
    {
      pos[0] = lroundf(v[0]);
      pos[1] = lroundf(v[1]);
      pos.cycle_unsigned(pos[0], resolution_);
      pos.cycle_unsigned(pos[1], resolution_);
      const auto c = cm_[pos];
      if (c > 99)
        return -1;
      sum += c;
      v[0] += dp[0];
      v[1] += dp[1];
    }
    cost += sum * weight_cost_ / 100.0;
    return cost;
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "planner2dofSerialJoints");
  ros::NodeHandle_f nh("~");

  std::vector<std::shared_ptr<planner2dofSerialJoints>> jys;
  int n;
  nh.param("num_groups", n, 1);
  for (int i = 0; i < n; i++)
  {
    std::string name;
    nh.param("group_" + std::to_string(i) + "_name",
             name, std::string("group_") + std::to_string(i));
    std::shared_ptr<planner2dofSerialJoints> jy;

    jy.reset(new planner2dofSerialJoints(name));
    jys.push_back(jy);
  }

  ros::spin();

  return 0;
}
