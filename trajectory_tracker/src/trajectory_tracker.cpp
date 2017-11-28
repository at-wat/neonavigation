/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
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

/*
   * This research was supported by a contract with the Ministry of Internal 
   Affairs and Communications entitled, 'Novel and innovative R&D making use 
   of brain structures'

   This software was implemented to accomplish the above research.
   Original idea of the implemented control scheme was published on:  
   S. Iida, S. Yuta, "Vehicle command system and trajectory control for 
   autonomous mobile robots," in Proceedings of the 1991 IEEE/RSJ 
   International Workshop on Intelligent Robots and Systems (IROS), 
   1991, pp. 212-217.
 */

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <trajectory_tracker/TrajectoryTrackerStatus.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <string>

class TrackerNode
{
public:
  TrackerNode();
  ~TrackerNode();
  void spin();

private:
  std::string topic_path_;
  std::string topic_cmd_vel_;
  std::string topic_odom_;
  std::string frame_robot_;
  double hz_;
  double look_forward_;
  double curv_forward_;
  double k_[3];
  double d_lim_;
  double d_stop_;
  double vel_[2];
  double acc_[2];
  double w_;
  double v_;
  double dec_;
  double rotate_ang_;
  double ang_factor_;
  double sw_dist_;
  double goal_tolerance_dist_;
  double goal_tolerance_ang_;
  double stop_tolerance_dist_;
  double stop_tolerance_ang_;
  double no_pos_cntl_dist_;
  double min_track_path_;
  int path_step_;
  int path_step_done_;
  bool out_of_line_strip_;
  bool allow_backward_;
  bool limit_vel_by_avel_;
  bool check_old_path_;

  int error_cnt_;

  ros::Subscriber sub_path_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_vel_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_status_;
  ros::Publisher pub_tracking_;
  ros::NodeHandle nh_;
  tf::TransformListener tfl_;

  nav_msgs::Path path_;
  nav_msgs::Odometry odom_;

  void cbPath(const nav_msgs::Path::ConstPtr &msg);
  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg);
  void cbSpeed(const std_msgs::Float32::ConstPtr &msg);
  void control();
};

template <typename T>
class average
{
public:
  average()
    : sum()
  {
    num = 0;
  };
  void operator+=(const T &val)
  {
    sum += val;
    num++;
  };
  operator T()
  {
    if (num == 0)
      return 0;
    return sum / num;
  };

private:
  T sum;
  int num;
};

TrackerNode::TrackerNode()
  : w_(0.0)
  , v_(0.0)
  , nh_("~")
{
  nh_.param("frame_robot", frame_robot_, std::string("base_link"));
  nh_.param("path", topic_path_, std::string("path"));
  nh_.param("odom", topic_odom_, std::string("odom"));
  nh_.param("cmd_vel", topic_cmd_vel_, std::string("cmd_vel"));
  nh_.param("hz", hz_, 50.0);
  nh_.param("look_forward", look_forward_, 0.5);
  nh_.param("curv_forward", curv_forward_, 0.5);
  nh_.param("k_dist", k_[0], 1.0);
  nh_.param("k_ang", k_[1], 1.0);
  nh_.param("k_avel", k_[2], 1.0);
  nh_.param("k_dcel", dec_, 0.2);
  nh_.param("dist_lim", d_lim_, 0.5);
  nh_.param("dist_stop", d_stop_, 2.0);
  nh_.param("rotate_ang", rotate_ang_, M_PI / 4);
  nh_.param("max_vel", vel_[0], 0.5);
  nh_.param("max_angvel", vel_[1], 1.0);
  nh_.param("max_acc", acc_[0], 1.0);
  nh_.param("max_angacc", acc_[1], 2.0);
  nh_.param("path_step", path_step_, 1);
  nh_.param("distance_angle_factor", ang_factor_, 0.0);
  nh_.param("switchback_dist", sw_dist_, 0.3);
  nh_.param("goal_tolerance_dist", goal_tolerance_dist_, 0.2);
  nh_.param("goal_tolerance_ang", goal_tolerance_ang_, 0.1);
  nh_.param("stop_tolerance_dist", stop_tolerance_dist_, 0.1);
  nh_.param("stop_tolerance_ang", stop_tolerance_ang_, 0.05);
  nh_.param("no_position_control_dist", no_pos_cntl_dist_, 0.0);
  nh_.param("min_tracking_path", min_track_path_, no_pos_cntl_dist_);
  nh_.param("allow_backward", allow_backward_, true);
  nh_.param("limit_vel_by_avel", limit_vel_by_avel_, false);
  nh_.param("check_old_path", check_old_path_, false);

  sub_path_ = nh_.subscribe(topic_path_, 2, &TrackerNode::cbPath, this);
  sub_odom_ = nh_.subscribe(topic_odom_, 20, &TrackerNode::cbOdom, this);
  sub_vel_ = nh_.subscribe("speed", 20, &TrackerNode::cbSpeed, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>(topic_cmd_vel_, 10);
  pub_status_ = nh_.advertise<trajectory_tracker::TrajectoryTrackerStatus>("status", 10);
  pub_tracking_ = nh_.advertise<geometry_msgs::PoseStamped>("tracking", 10);
}
TrackerNode::~TrackerNode()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  pub_vel_.publish(cmd_vel);
}

float dist2d(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
}
float len2d(const geometry_msgs::Point a)
{
  return sqrtf(powf(a.x, 2) + powf(a.y, 2));
}
float curv3p(const geometry_msgs::Point &a,
             const geometry_msgs::Point &b,
             const geometry_msgs::Point &c)
{
  float ret;
  ret = 2 * (a.x * b.y + b.x * c.y + c.x * a.y - a.x * c.y - b.x * a.y - c.x * b.y);
  ret /= sqrtf((powf(b.x - a.x, 2) +
                powf(b.y - a.y, 2)) *
               (powf(b.x - c.x, 2) +
                powf(b.y - c.y, 2)) *
               (powf(c.x - a.x, 2) +
                powf(c.y - a.y, 2)));

  return ret;
}
float cross2d(const geometry_msgs::Point a, const geometry_msgs::Point b)
{
  return a.x * b.y - a.y * b.x;
}
float dot2d(const geometry_msgs::Point a, const geometry_msgs::Point b)
{
  return a.x * b.x + a.y * b.y;
}
geometry_msgs::Point point2d(const float x, const float y)
{
  geometry_msgs::Point ret;
  ret.x = x;
  ret.y = y;
  return ret;
}
geometry_msgs::Point sub2d(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  geometry_msgs::Point ret;
  ret.x = a.x - b.x;
  ret.y = a.y - b.y;
  return ret;
}
float sign(const float a)
{
  if (a < 0)
    return -1;
  return 1;
}
float dist2d_line(const geometry_msgs::Point &a,
                  const geometry_msgs::Point &b,
                  const geometry_msgs::Point &c)
{
  return (cross2d(sub2d(b, a), sub2d(c, a)) / dist2d(b, a));
}
float dist2d_linestrip(const geometry_msgs::Point &a,
                       const geometry_msgs::Point &b,
                       const geometry_msgs::Point &c)
{
  if (dot2d(sub2d(b, a), sub2d(c, a)) <= 0)
    return dist2d(c, a);
  if (dot2d(sub2d(a, b), sub2d(c, b)) <= 0)
    return -dist2d(c, b) - 0.005;
  return fabs(dist2d_line(a, b, c));
}
geometry_msgs::Point projection2d(const geometry_msgs::Point &a,
                                  const geometry_msgs::Point &b,
                                  const geometry_msgs::Point &c)
{
  float r = dot2d(sub2d(b, a), sub2d(c, a)) / pow(len2d(sub2d(b, a)), 2);
  geometry_msgs::Point ret;
  ret.x = b.x * r + a.x * (1 - r);
  ret.y = b.y * r + a.y * (1 - r);
  return ret;
}

void TrackerNode::cbSpeed(const std_msgs::Float32::ConstPtr &msg)
{
  vel_[0] = msg->data;
}

void TrackerNode::cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom_ = *msg;
}

void TrackerNode::cbPath(const nav_msgs::Path::ConstPtr &msg)
{
  path_ = *msg;
  auto i = path_.poses.begin();
  for (auto j = path_.poses.begin(); j != path_.poses.end();)
  {
    if (j + 1 == path_.poses.end())
      break;
    if (i != j && dist2d((*i).pose.position, (*j).pose.position) < 0.01)
    {
      j = path_.poses.erase(j);
      continue;
    }
    i = j;
    j++;
  }
  path_step_done_ = 0;

  while (path_.poses.size() < 3 && path_.poses.size() > 0)
  {
    float yaw = tf::getYaw(path_.poses.back().pose.orientation);
    auto next = path_.poses.back();
    next.pose.position.x += 0.001 * cos(yaw);
    next.pose.position.y += 0.001 * sin(yaw);
    path_.poses.push_back(next);
  }
}

float timeoptimal_control(const float angle_orig, const float acc_, const float dt)
{
  float angle = angle_orig;

  float angvel = -sign(angle) * sqrtf(fabs(2 * angle * acc_ * 0.85));
  angle += angvel * dt * 1.5;
  if (angle * angle_orig < 0)
    angle = 0;

  angvel = -sign(angle) * sqrtf(fabs(2 * angle * acc_ * 0.85));
  return angvel;
}

void TrackerNode::spin()
{
  ros::Rate loop_rate(hz_);

  while (ros::ok())
  {
    control();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void TrackerNode::control()
{
  trajectory_tracker::TrajectoryTrackerStatus status;
  status.header.stamp = ros::Time::now();
  status.header.seq = path_.header.seq;
  status.distance_remains = 0.0;
  status.angle_remains = 0.0;

  if (path_.header.frame_id.size() == 0)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_vel_.publish(cmd_vel);
    status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }
  // Transform
  nav_msgs::Path lpath;
  lpath.header = path_.header;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time(0);
    tfl_.waitForTransform(frame_robot_, path_.header.frame_id, now, ros::Duration(0.05));
    tfl_.lookupTransform(frame_robot_, path_.header.frame_id, now, transform);
    if (fabs((ros::Time::now() - transform.stamp_).toSec()) > 0.1)
    {
      if (error_cnt_ % 16 == 0 && check_old_path_)
        ROS_ERROR("Timestamp of the transform is too old %f %f", ros::Time::now().toSec(), transform.stamp_.toSec());
      error_cnt_++;
    }
    else
    {
      error_cnt_ = 0;
    }

    for (size_t i = 0; i < path_.poses.size(); i += path_step_)
    {
      geometry_msgs::PoseStamped pose;
      tfl_.transformPose(frame_robot_, transform.stamp_, path_.poses[i], path_.header.frame_id, pose);
      lpath.poses.push_back(pose);
    }
  }
  catch (tf::TransformException &e)
  {
    ROS_WARN("TF exception: %s", e.what());
    status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }

  float min_dist_ = 10000.0;
  int iclose = -1;
  geometry_msgs::Point origin;
  origin.x = cos(w_ * look_forward_ / 2.0) * v_ * look_forward_;
  origin.y = sin(w_ * look_forward_ / 2.0) * v_ * look_forward_;
  // Find nearest line strip
  out_of_line_strip_ = false;
  float distance_path_ = 0;
  for (size_t i = 1; i < lpath.poses.size(); i++)
  {
    distance_path_ += dist2d(lpath.poses[i - 1].pose.position, lpath.poses[i].pose.position);
  }
  float distance_path_search_ = 0;
  float sign_vel_prev_ = 0;
  for (size_t i = path_step_done_; i < lpath.poses.size(); i++)
  {
    if (i < 1)
      continue;
    distance_path_search_ += dist2d(lpath.poses[i - 1].pose.position, lpath.poses[i].pose.position);
    if (dist2d(origin, lpath.poses[i].pose.position) < 0.05 &&
        i < lpath.poses.size() - 1)
      continue;
    float d = dist2d_linestrip(lpath.poses[i - 1].pose.position, lpath.poses[i].pose.position, origin);
    if (fabs(d) <= fabs(min_dist_))
    {
      min_dist_ = d;
      iclose = i;
    }
    if (path_step_done_ > 0 && distance_path_search_ > 1.0)
      break;

    geometry_msgs::Point vec = sub2d(lpath.poses[i].pose.position,
                                     lpath.poses[i - 1].pose.position);
    float angle = atan2(vec.y, vec.x);
    float angle_pose_;
    if (allow_backward_)
      angle_pose_ = tf::getYaw(lpath.poses[i].pose.orientation);
    else
      angle_pose_ = angle;
    float signVel_req = cos(angle) * cos(angle_pose_) + sin(angle) * sin(angle_pose_);
    if (sign_vel_prev_ * signVel_req < 0)
    {
      // Stop read forward if the path_ switched back
      break;
    }
    sign_vel_prev_ = signVel_req;
  }
  if (iclose < 0)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_vel_.publish(cmd_vel);
    // ROS_WARN("failed to find nearest node");
    status.status = trajectory_tracker::TrajectoryTrackerStatus::NO_PATH;
    pub_status_.publish(status);
    return;
  }
  // Signed distance error
  float dist = dist2d_line(lpath.poses[iclose - 1].pose.position, lpath.poses[iclose].pose.position, origin);
  float _dist = dist;
  if (iclose == 0)
  {
    _dist = -dist2d(lpath.poses[iclose].pose.position, origin);
  }
  if (iclose + 1 >= static_cast<int>(path_.poses.size()))
  {
    _dist = -dist2d(lpath.poses[iclose].pose.position, origin);
  }

  // Angular error
  geometry_msgs::Point vec = sub2d(lpath.poses[iclose].pose.position, lpath.poses[iclose - 1].pose.position);
  float angle = -atan2(vec.y, vec.x);
  float angle_pose_;
  if (allow_backward_)
    angle_pose_ = tf::getYaw(lpath.poses[iclose].pose.orientation);
  else
    angle_pose_ = -angle;
  float sign_vel_ = 1.0;
  if (cos(-angle) * cos(angle_pose_) + sin(-angle) * sin(angle_pose_) < 0)
  {
    sign_vel_ = -1.0;
    angle = angle + M_PI;
    if (angle > M_PI)
      angle -= 2.0 * M_PI;
  }
  // Curvature
  average<float> curv;
  geometry_msgs::Point pos_line_ =
      projection2d(lpath.poses[iclose - 1].pose.position, lpath.poses[iclose].pose.position, origin);
  int local_goal = lpath.poses.size() - 1;
  float remain_local_ = 0;
  remain_local_ = dist2d(pos_line_, lpath.poses[iclose].pose.position);
  for (int i = iclose - 1; i < static_cast<int>(lpath.poses.size()) - 1; i++)
  {
    if (i > 2)
    {
      geometry_msgs::Point vec = sub2d(lpath.poses[i].pose.position,
                                       lpath.poses[i - 1].pose.position);
      float angle = atan2(vec.y, vec.x);
      float angle_pose_;
      if (allow_backward_)
        angle_pose_ = tf::getYaw(lpath.poses[i + 1].pose.orientation);
      else
        angle_pose_ = angle;
      float signVel_req = cos(angle) * cos(angle_pose_) + sin(angle) * sin(angle_pose_);
      if (sign_vel_ * signVel_req < 0)
      {
        // Stop read forward if the path_ switched back
        local_goal = i;
        break;
      }
      if (i > iclose)
        remain_local_ += dist2d(lpath.poses[i - 1].pose.position, lpath.poses[i].pose.position);
    }
  }
  for (int i = iclose - 1; i < local_goal; i++)
  {
    if (i > 2)
    {
      curv += curv3p(lpath.poses[i - 2].pose.position, lpath.poses[i - 1].pose.position, lpath.poses[i].pose.position);
    }
    if (dist2d(lpath.poses[i].pose.position,
               lpath.poses[local_goal].pose.position) < 0.05)
      break;
    if (dist2d(lpath.poses[i].pose.position, pos_line_) > curv_forward_)
      break;
  }
  float remain;
  remain = dist2d(origin, lpath.poses.back().pose.position);
  if (min_dist_ < 0 && iclose == local_goal)
    out_of_line_strip_ = true;
  if (out_of_line_strip_)
  {
    remain = -remain;
    remain_local_ = -remain_local_;
  }
  if (distance_path_ < no_pos_cntl_dist_)
    remain = remain_local_ = 0;
  // fprintf(stderr,"%d %d   %0.3f  %+0.3f %+0.3f  %f  %f  sv %f\n",
  //   out_of_line_strip_, iclose, distance_path_, remain, remain_local_, min_dist_, angle, sign_vel_);
  // printf("d=%.2f, th=%.2f, curv=%.2f\n", dist, angle, (float)curv);
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  while (angle > M_PI)
    angle -= 2.0 * M_PI;

  status.distance_remains = remain;
  status.angle_remains = angle;

  float dt = 1.0 / hz_;
  float _v = v_;
  float _w = w_;
  // Stop and rotate
  if ((fabs(rotate_ang_) < M_PI && cos(rotate_ang_) > cos(angle)) ||
      fabs(remain_local_) < stop_tolerance_dist_ ||
      distance_path_ < min_track_path_)
  {
    // int m = 0;
    if (distance_path_ < min_track_path_ || fabs(remain_local_) < stop_tolerance_dist_)
    {
      angle = -tf::getYaw(lpath.poses.back().pose.orientation);
      status.angle_remains = angle;
      // m = 1;
    }
    w_ = timeoptimal_control(angle, acc_[1], dt);

    // float wo = w_;
    v_ = 0;
    if (v_ > vel_[0])
      v_ = vel_[0];
    else if (v_ < -vel_[0])
      v_ = -vel_[0];
    if (v_ > _v + dt * acc_[0])
      v_ = _v + dt * acc_[0];
    else if (v_ < _v - dt * acc_[0])
      v_ = _v - dt * acc_[0];
    if (w_ > vel_[1])
      w_ = vel_[1];
    else if (w_ < -vel_[1])
      w_ = -vel_[1];
    if (w_ > _w + dt * acc_[1])
      w_ = _w + dt * acc_[1];
    else if (w_ < _w - dt * acc_[1])
      w_ = _w - dt * acc_[1];
    // fprintf(stderr, "%0.3f %0.3f %0.3f %d\n", angle, w_, wo, m);

    if (distance_path_ < stop_tolerance_dist_)
    {
      status.distance_remains = remain = remain_local_ = 0.0;
    }
  }
  else
  {
    // Control
    if (dist < -d_lim_)
      dist = -d_lim_;
    else if (dist > d_lim_)
      dist = d_lim_;

    v_ = timeoptimal_control(-remain_local_ * sign_vel_, acc_[0], dt);

    if (v_ > vel_[0])
      v_ = vel_[0];
    else if (v_ < -vel_[0])
      v_ = -vel_[0];
    if (v_ > _v + dt * acc_[0])
      v_ = _v + dt * acc_[0];
    else if (v_ < _v - dt * acc_[0])
      v_ = _v - dt * acc_[0];

    float wref = fabs(v_) * curv;

    if (limit_vel_by_avel_)
    {
      if (fabs(wref) > vel_[1])
      {
        v_ = sign(v_) * fabs(vel_[1] / curv);
        if (v_ > vel_[0])
          v_ = vel_[0];
        else if (v_ < -vel_[0])
          v_ = -vel_[0];
        if (v_ > _v + dt * acc_[0])
          v_ = _v + dt * acc_[0];
        else if (v_ < _v - dt * acc_[0])
          v_ = _v - dt * acc_[0];
      }
    }

    w_ += dt * (-dist * k_[0] - angle * k_[1] - (w_ - wref) * k_[2]);

    if (w_ > vel_[1])
      w_ = vel_[1];
    else if (w_ < -vel_[1])
      w_ = -vel_[1];
    if (w_ > _w + dt * acc_[1])
      w_ = _w + dt * acc_[1];
    else if (w_ < _w - dt * acc_[1])
      w_ = _w - dt * acc_[1];

    if (!std::isfinite(v_))
      v_ = 0;
    if (!std::isfinite(w_))
      w_ = 0;

    // Too far from given path
    if (fabs(_dist) > d_stop_)
    {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      pub_vel_.publish(cmd_vel);
      // ROS_WARN("Far from given path");
      status.status = trajectory_tracker::TrajectoryTrackerStatus::FAR_FROM_PATH;
      pub_status_.publish(status);
      return;
    }
  }

  geometry_msgs::Twist cmd_vel;
  if (fabs(status.distance_remains) < stop_tolerance_dist_ &&
      fabs(status.angle_remains) < stop_tolerance_ang_)
  {
    v_ = 0;
    w_ = 0;
  }

  cmd_vel.linear.x = v_;
  cmd_vel.angular.z = w_;
  pub_vel_.publish(cmd_vel);
  status.status = trajectory_tracker::TrajectoryTrackerStatus::FOLLOWING;
  if (fabs(status.distance_remains) < goal_tolerance_dist_ &&
      fabs(status.angle_remains) < goal_tolerance_ang_)
  {
    status.status = trajectory_tracker::TrajectoryTrackerStatus::GOAL;
  }
  pub_status_.publish(status);
  geometry_msgs::PoseStamped tracking;
  tracking.header = status.header;
  tracking.header.frame_id = frame_robot_;
  tracking.pose.position = pos_line_;
  tracking.pose.orientation = tf::createQuaternionMsgFromYaw(-angle);
  pub_tracking_.publish(tracking);

  path_step_done_ = iclose;
  if (path_step_done_ < 0)
    path_step_done_ = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_tracker");

  TrackerNode track;
  track.spin();

  return 0;
}
