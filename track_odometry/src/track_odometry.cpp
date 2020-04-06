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

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <track_odometry/kalman_filter1.h>

#include <neonavigation_common/compatibility.h>

Eigen::Vector3d toEigen(const geometry_msgs::Vector3& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Vector3d toEigen(const geometry_msgs::Point& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}
Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& a)
{
  return Eigen::Quaterniond(a.w, a.x, a.y, a.z);
}
geometry_msgs::Point toPoint(const Eigen::Vector3d& a)
{
  geometry_msgs::Point b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}
geometry_msgs::Vector3 toVector3(const Eigen::Vector3d& a)
{
  geometry_msgs::Vector3 b;
  b.x = a.x();
  b.y = a.y();
  b.z = a.z();
  return b;
}

class TrackOdometryNode
{
private:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu>;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_imu_raw_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu>> sub_imu_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  ros::Subscriber sub_reset_z_;
  ros::Publisher pub_odom_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  nav_msgs::Odometry odom_prev_;
  nav_msgs::Odometry odomraw_prev_;

  std::string base_link_id_;
  std::string base_link_id_overwrite_;
  std::string odom_id_;

  sensor_msgs::Imu imu_;
  double gyro_zero_[3];
  double z_filter_timeconst_;
  double tf_tolerance_;

  bool debug_;
  bool use_kf_;
  bool negative_slip_;
  double sigma_predict_;
  double sigma_odom_;
  double predict_filter_tc_;
  double dist_;
  bool without_odom_;

  track_odometry::KalmanFilter1 slip_;

  bool has_imu_;
  bool has_odom_;
  bool publish_tf_;

  void cbResetZ(const std_msgs::Float32::Ptr& msg)
  {
    odom_prev_.pose.pose.position.z = msg->data;
  }
  void cbOdomImu(const nav_msgs::Odometry::ConstPtr& odom_msg, const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    ROS_DEBUG(
        "Synchronized timestamp: odom %0.3f, imu %0.3f",
        odom_msg->header.stamp.toSec(),
        imu_msg->header.stamp.toSec());
    cbImu(imu_msg);
    cbOdom(odom_msg);
  }
  void cbImu(const sensor_msgs::Imu::ConstPtr& msg)
  {
    if (base_link_id_.size() == 0)
    {
      ROS_ERROR("base_link id is not specified.");
      return;
    }

    imu_.header = msg->header;
    try
    {
      geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(
          base_link_id_, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));

      geometry_msgs::Vector3Stamped vin, vout;
      vin.header = imu_.header;
      vin.header.stamp = ros::Time(0);
      vin.vector = msg->linear_acceleration;
      tf2::doTransform(vin, vout, trans);
      imu_.linear_acceleration = vout.vector;

      vin.header = imu_.header;
      vin.header.stamp = ros::Time(0);
      vin.vector = msg->angular_velocity;
      tf2::doTransform(vin, vout, trans);
      imu_.angular_velocity = vout.vector;

      tf2::Stamped<tf2::Quaternion> qin, qout;
      geometry_msgs::QuaternionStamped qmin, qmout;
      qmin.header = imu_.header;
      qmin.quaternion = msg->orientation;
      tf2::fromMsg(qmin, qin);

      auto axis = qin.getAxis();
      auto angle = qin.getAngle();
      geometry_msgs::Vector3Stamped axis2;
      geometry_msgs::Vector3Stamped axis1;
      axis1.vector = tf2::toMsg(axis);
      axis1.header.stamp = ros::Time(0);
      axis1.header.frame_id = qin.frame_id_;
      tf2::doTransform(axis1, axis2, trans);

      tf2::fromMsg(axis2.vector, axis);
      qout.setData(tf2::Quaternion(axis, angle));
      qout.stamp_ = qin.stamp_;
      qout.frame_id_ = base_link_id_;

      qmout = tf2::toMsg(qout);
      imu_.orientation = qmout.quaternion;
      // ROS_INFO("%0.3f %s -> %0.3f %s",
      //   tf2::getYaw(qmin.quaternion), qmin.header.frame_id.c_str(),
      //   tf2::getYaw(qmout.quaternion), qmout.header.frame_id.c_str());

      has_imu_ = true;
    }
    catch (tf2::TransformException& e)
    {
      ROS_ERROR("%s", e.what());
      has_imu_ = false;
      return;
    }
  }
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
  {
    nav_msgs::Odometry odom = *msg;
    if (has_odom_)
    {
      const double dt = (odom.header.stamp - odomraw_prev_.header.stamp).toSec();
      if (base_link_id_overwrite_.size() == 0)
      {
        base_link_id_ = odom.child_frame_id;
      }

      if (!has_imu_)
      {
        ROS_ERROR_THROTTLE(1.0, "IMU data not received");
        return;
      }

      double slip_ratio = 1.0;
      odom.header.stamp += ros::Duration(tf_tolerance_);
      odom.twist.twist.angular = imu_.angular_velocity;
      odom.pose.pose.orientation = imu_.orientation;

      double w_imu = imu_.angular_velocity.z;
      const double w_odom = msg->twist.twist.angular.z;

      if (w_imu * w_odom < 0 && !negative_slip_)
        w_imu = w_odom;

      slip_.predict(-slip_.x_ * dt * predict_filter_tc_, dt * sigma_predict_);
      if (std::abs(w_odom) > sigma_odom_ * 3)
      {
        // non-kf mode: calculate slip_ratio if angular vel < 3*sigma
        slip_ratio = w_imu / w_odom;
      }

      const double slip_ratio_per_angvel =
          (w_odom - w_imu) / (w_odom * std::abs(w_odom));
      double slip_ratio_per_angvel_sigma =
          sigma_odom_ * std::abs(2.0 * w_odom * sigma_odom_ / std::pow(w_odom * w_odom - sigma_odom_ * sigma_odom_, 2));
      if (std::abs(w_odom) < sigma_odom_)
        slip_ratio_per_angvel_sigma = std::numeric_limits<double>::infinity();

      slip_.measure(slip_ratio_per_angvel, slip_ratio_per_angvel_sigma);
      // printf("%0.5f %0.5f %0.5f   %0.5f %0.5f  %0.5f\n",
      //   slip_ratio_per_angvel, slip_ratio_sigma, slip_ratio_per_angvel_sigma,
      //   slip_.x_, slip_.sigma_, msg->twist.twist.angular.z);

      if (debug_)
      {
        printf("%0.3f %0.3f  %0.3f  %0.3f %0.3f  %0.3f  %0.3f\n",
               imu_.angular_velocity.z,
               msg->twist.twist.angular.z,
               slip_ratio,
               slip_.x_, slip_.sigma_,
               odom.twist.twist.linear.x, dist_);
      }
      dist_ += odom.twist.twist.linear.x * dt;

      const Eigen::Vector3d diff = toEigen(msg->pose.pose.position) - toEigen(odomraw_prev_.pose.pose.position);
      Eigen::Vector3d v =
          toEigen(odom.pose.pose.orientation) * toEigen(msg->pose.pose.orientation).inverse() * diff;
      if (use_kf_)
        v *= 1.0 - slip_.x_;
      else
        v *= slip_ratio;

      odom.pose.pose.position = toPoint(toEigen(odom_prev_.pose.pose.position) + v);
      if (z_filter_timeconst_ > 0)
        odom.pose.pose.position.z *= 1.0 - (dt / z_filter_timeconst_);

      odom.child_frame_id = base_link_id_;
      pub_odom_.publish(odom);

      geometry_msgs::TransformStamped odom_trans;

      odom_trans.header = odom.header;
      odom_trans.child_frame_id = base_link_id_;
      odom_trans.transform.translation = toVector3(toEigen(odom.pose.pose.position));
      odom_trans.transform.rotation = odom.pose.pose.orientation;
      if (publish_tf_)
        tf_broadcaster_.sendTransform(odom_trans);
    }
    odomraw_prev_ = *msg;
    odom_prev_ = odom;
    has_odom_ = true;
  }

public:
  TrackOdometryNode()
    : nh_()
    , pnh_("~")
    , tf_listener_(tf_buffer_)
  {
    neonavigation_common::compat::checkCompatMode();

    bool enable_tcp_no_delay;
    pnh_.param("enable_tcp_no_delay", enable_tcp_no_delay, true);
    const ros::TransportHints transport_hints =
        enable_tcp_no_delay ? ros::TransportHints().reliable().tcpNoDelay(true) : ros::TransportHints();

    pnh_.param("without_odom", without_odom_, false);
    if (without_odom_)
    {
      sub_imu_raw_ = neonavigation_common::compat::subscribe(
          nh_, "imu/data",
          nh_, "imu", 64, &TrackOdometryNode::cbImu, this);
      pnh_.param("base_link_id", base_link_id_, std::string("base_link"));
      pnh_.param("odom_id", odom_id_, std::string("odom"));
    }
    else
    {
      sub_odom_.reset(
          new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odom_raw", 50, transport_hints));
      if (neonavigation_common::compat::getCompat() == neonavigation_common::compat::current_level)
      {
        sub_imu_.reset(
            new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "imu/data", 50, transport_hints));
      }
      else
      {
        sub_imu_.reset(
            new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "imu", 50, transport_hints));
      }

      int sync_window;
      pnh_.param("sync_window", sync_window, 50);
      sync_.reset(
          new message_filters::Synchronizer<SyncPolicy>(
              SyncPolicy(sync_window), *sub_odom_, *sub_imu_));
      sync_->registerCallback(boost::bind(&TrackOdometryNode::cbOdomImu, this, _1, _2));

      pnh_.param("base_link_id", base_link_id_overwrite_, std::string(""));
    }

    sub_reset_z_ = neonavigation_common::compat::subscribe(
        nh_, "reset_odometry_z",
        pnh_, "reset_z", 1, &TrackOdometryNode::cbResetZ, this);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 8);

    if (pnh_.hasParam("z_filter"))
    {
      z_filter_timeconst_ = -1.0;
      double z_filter;
      if (pnh_.getParam("z_filter", z_filter))
      {
        const double odom_freq = 100.0;
        if (0.0 < z_filter && z_filter < 1.0)
          z_filter_timeconst_ = (1.0 / odom_freq) / (1.0 - z_filter);
      }
      ROS_ERROR(
          "track_odometry: ~z_filter parameter (exponential filter (1 - alpha) value) is deprecated. "
          "Use ~z_filter_timeconst (in seconds) instead. "
          "Treated as z_filter_timeconst=%0.6f. (negative value means disabled)",
          z_filter_timeconst_);
    }
    else
    {
      pnh_.param("z_filter_timeconst", z_filter_timeconst_, -1.0);
    }
    pnh_.param("tf_tolerance", tf_tolerance_, 0.01);
    pnh_.param("use_kf", use_kf_, true);
    pnh_.param("enable_negative_slip", negative_slip_, false);
    pnh_.param("debug", debug_, false);
    pnh_.param("publish_tf", publish_tf_, true);

    if (base_link_id_overwrite_.size() > 0)
    {
      base_link_id_ = base_link_id_overwrite_;
    }

    // sigma_odom_ [rad/s]: standard deviation of odometry angular vel on straight running
    pnh_.param("sigma_odom", sigma_odom_, 0.005);
    // sigma_predict_ [sigma/second]: prediction sigma of kalman filter
    pnh_.param("sigma_predict", sigma_predict_, 0.5);
    // predict_filter_tc_ [sec.]: LPF time-constant to forget estimated slip_ ratio
    pnh_.param("predict_filter_tc", predict_filter_tc_, 1.0);

    has_imu_ = false;
    has_odom_ = false;

    dist_ = 0;
    slip_.set(0.0, 0.1);
  }
  void cbTimer(const ros::TimerEvent& event)
  {
    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header.stamp = ros::Time::now();
    odom->header.frame_id = odom_id_;
    odom->child_frame_id = base_link_id_;
    odom->pose.pose.orientation.w = 1.0;
    cbOdom(odom);
  }
  void spin()
  {
    if (!without_odom_)
    {
      ros::spin();
    }
    else
    {
      ros::Timer timer = nh_.createTimer(
          ros::Duration(1.0 / 50.0), &TrackOdometryNode::cbTimer, this);
      ros::spin();
    }
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "track_odometry");

  TrackOdometryNode odom;

  odom.spin();

  return 0;
}
