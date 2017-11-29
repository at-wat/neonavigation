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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <limits>
#include <string>

#include <kalman_filter1.h>

static geometry_msgs::Vector3 operator*(const double &a, const geometry_msgs::Vector3 &vin)
{
  geometry_msgs::Vector3 vout;
  vout.x = vin.x * a;
  vout.y = vin.y * a;
  vout.z = vin.z * a;
  return vout;
}
static geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3 &a,
                                        const geometry_msgs::Vector3 &b)
{
  geometry_msgs::Vector3 vout;
  vout.x = a.x + b.x;
  vout.y = a.y + b.y;
  vout.z = a.z + b.z;
  return vout;
}
static geometry_msgs::Point operator+(const geometry_msgs::Point &a,
                                      const geometry_msgs::Vector3 &b)
{
  geometry_msgs::Point vout;
  vout.x = a.x + b.x;
  vout.y = a.y + b.y;
  vout.z = a.z + b.z;
  return vout;
}
geometry_msgs::Vector3 to_vector3(const geometry_msgs::Point &a)
{
  geometry_msgs::Vector3 vout;
  vout.x = a.x;
  vout.y = a.y;
  vout.z = a.z;
  return vout;
}
geometry_msgs::Vector3 cross(const geometry_msgs::Quaternion &q,
                             const geometry_msgs::Vector3 &vin)
{
  geometry_msgs::Vector3 vout;
  vout.x = q.y * vin.z - q.z * vin.y;
  vout.y = q.z * vin.x - q.x * vin.z;
  vout.z = q.x * vin.y - q.y * vin.x;
  return vout;
}

class TrackOdometryNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_reset_z_;
  ros::Publisher pub_odom_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  nav_msgs::Odometry odom_prev_;
  nav_msgs::Odometry odomraw_prev_;

  std::string base_link_id_;
  std::string base_link_id_overwrite_;
  std::string odom_id_;

  sensor_msgs::Imu imu_;
  double gyro_zero_[3];
  double z_filter_;
  double tf_tolerance_;

  bool debug_;
  bool use_kf_;
  bool negative_slip_;
  double sigma_predict_;
  double sigma_odom_;
  double predict_filter_tc_;
  float dist_;
  bool without_odom_;

  KalmanFilter1 slip_;

  bool has_imu_;
  bool has_odom_;
  bool publish_tf_;

  void cb_reset_z(const std_msgs::Float32::Ptr &msg)
  {
    odom_prev_.pose.pose.position.z = msg->data;
  }
  void cb_imu(const sensor_msgs::Imu::Ptr &msg)
  {
    if (base_link_id_.size() == 0)
    {
      ROS_ERROR("base_link id is not specified.");
      return;
    }

    imu_.header = msg->header;
    try
    {
      tf_listener_.waitForTransform(base_link_id_, msg->header.frame_id,
                                    ros::Time(0), ros::Duration(0.1));

      geometry_msgs::Vector3Stamped vin, vout;
      vin.header = imu_.header;
      vin.header.stamp = ros::Time(0);
      vin.vector = msg->linear_acceleration;
      tf_listener_.transformVector(base_link_id_, vin, vout);
      imu_.linear_acceleration = vout.vector;

      vin.header = imu_.header;
      vin.header.stamp = ros::Time(0);
      vin.vector = msg->angular_velocity;
      tf_listener_.transformVector(base_link_id_, vin, vout);
      imu_.angular_velocity = vout.vector;

      tf::Stamped<tf::Quaternion> qin, qout;
      geometry_msgs::QuaternionStamped qmin, qmout;
      qmin.header = imu_.header;
      qmin.quaternion = msg->orientation;
      tf::quaternionStampedMsgToTF(qmin, qin);

      auto axis = qin.getAxis();
      auto angle = qin.getAngle();
      tf::Stamped<tf::Vector3> axis2;
      tf::Stamped<tf::Vector3> axis1;
      axis1.setData(axis);
      axis1.stamp_ = ros::Time(0);
      axis1.frame_id_ = qin.frame_id_;
      tf_listener_.transformVector(base_link_id_, axis1, axis2);

      qout.setData(tf::Quaternion(axis2, angle));
      qout.stamp_ = qin.stamp_;
      qout.frame_id_ = base_link_id_;

      tf::quaternionStampedTFToMsg(qout, qmout);
      imu_.orientation = qmout.quaternion;
      // ROS_INFO("%0.3f %s -> %0.3f %s",
      //   tf::getYaw(qmin.quaternion), qmin.header.frame_id.c_str(),
      //   tf::getYaw(qmout.quaternion), qmout.header.frame_id.c_str());

      has_imu_ = true;
    }
    catch (tf::TransformException &e)
    {
      ROS_ERROR("%s", e.what());
      has_imu_ = false;
      return;
    }
  }
  void cb_odom(const nav_msgs::Odometry::Ptr &msg)
  {
    nav_msgs::Odometry odom = *msg;
    if (has_odom_)
    {
      double dt = (odom.header.stamp - odomraw_prev_.header.stamp).toSec();
      if (base_link_id_overwrite_.size() == 0)
      {
        base_link_id_ = odom.child_frame_id;
      }

      if (!has_imu_)
      {
        ROS_ERROR_THROTTLE(1.0, "IMU data not received");
        return;
      }

      float slip_ratio = 1.0;
      odom.header.stamp += ros::Duration(tf_tolerance_);
      odom.twist.twist.angular = imu_.angular_velocity;
      odom.pose.pose.orientation = imu_.orientation;

      float w_imu = imu_.angular_velocity.z;
      const float w_odom = msg->twist.twist.angular.z;

      if (w_imu * w_odom < 0 && !negative_slip_)
        w_imu = w_odom;

      slip_.predict(-slip_.x_ * dt * predict_filter_tc_, dt * sigma_predict_);
      if (fabs(w_odom) > sigma_odom_ * 3)
      {
        // non-kf mode: calculate slip_ratio if angular vel < 3*sigma
        slip_ratio = w_imu / w_odom;
      }

      const float slip_ratio_per_angvel =
          (w_odom - w_imu) / (w_odom * fabs(w_odom));
      float slip_ratio_per_angvel_sigma =
          sigma_odom_ * fabs(2 * w_odom * sigma_odom_ / powf(w_odom * w_odom - sigma_odom_ * sigma_odom_, 2.0));
      if (fabs(w_odom) < sigma_odom_)
        slip_ratio_per_angvel_sigma = std::numeric_limits<float>::infinity();

      slip_.measure(slip_ratio_per_angvel, slip_ratio_per_angvel_sigma);
      // printf("%0.5f %0.5f %0.5f   %0.5f %0.5f  %0.5f\n",
      //   slip_ratio_per_angvel, slip_ratio_sigma, slip_ratio_per_angvel_sigma,
      //   slip_.x_, slip_.sigma_, msg->twist.twist.angular.z);
      if (use_kf_)
        odom.twist.twist.linear.x *= 1.0 - slip_.x_ * fabs(w_odom);
      else
        odom.twist.twist.linear.x *= slip_ratio;

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

      geometry_msgs::Vector3 v, t;
      t = 2.0 * cross(odom.pose.pose.orientation, odom.twist.twist.linear);
      v = odom.twist.twist.linear + odom.pose.pose.orientation.w * t + cross(odom.pose.pose.orientation, t);

      odom.pose.pose.position = odom_prev_.pose.pose.position + dt * v;
      odom.pose.pose.position.z *= z_filter_;
      pub_odom_.publish(odom);

      geometry_msgs::TransformStamped odom_trans;

      odom_trans.header = odom.header;
      odom_trans.child_frame_id = base_link_id_;
      odom_trans.transform.translation = to_vector3(odom.pose.pose.position);
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
    : nh_("")
    , pnh_("~")
  {
    pnh_.param("without_odom", without_odom_, false);
    if (!without_odom_)
      sub_odom_ = nh_.subscribe("odom_raw", 64, &TrackOdometryNode::cb_odom, this);
    sub_imu_ = nh_.subscribe("imu", 64, &TrackOdometryNode::cb_imu, this);
    sub_reset_z_ = pnh_.subscribe("reset_z", 1, &TrackOdometryNode::cb_reset_z, this);
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 8);

    if (!without_odom_)
    {
      pnh_.param("base_link_id", base_link_id_overwrite_, std::string(""));
    }
    else
    {
      pnh_.param("base_link_id", base_link_id_, std::string("base_link"));
      pnh_.param("odom_id", odom_id_, std::string("odom"));
    }
    pnh_.param("z_filter", z_filter_, 0.99);
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
  void spin()
  {
    if (!without_odom_)
    {
      ros::spin();
    }
    else
    {
      ros::Rate r(50);
      while (ros::ok())
      {
        r.sleep();
        ros::spinOnce();

        nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
        odom->header.stamp = ros::Time::now();
        odom->header.frame_id = odom_id_;
        odom->child_frame_id = base_link_id_;
        odom->pose.pose.orientation.w = 1.0;
        cb_odom(odom);
      }
    }
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "track_odometry");

  TrackOdometryNode odom;

  odom.spin();

  return 0;
}
