/*
 * Copyright (c) 2018-2020, the neonavigation authors
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

#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <gtest/gtest.h>

#define UNDEF_COMPATIBILITY_LEVEL

namespace neonavigation_common
{
namespace compat
{
int current_level;
int supported_level;
int default_level;
}  // namespace compat
}  // namespace neonavigation_common
#include <neonavigation_common/compatibility.h>

TEST(NeonavigationCompat, CompatMode)
{
  neonavigation_common::compat::supported_level = 2;
  neonavigation_common::compat::current_level = 3;
  neonavigation_common::compat::default_level = neonavigation_common::compat::supported_level;

  ros::NodeHandle("/").setParam("neonavigation_compatible", 2);
  ASSERT_NO_THROW(
      {
        neonavigation_common::compat::checkCompatMode();
      });  // NOLINT(whitespace/braces)

  ros::NodeHandle("/").setParam("neonavigation_compatible", 3);
  ASSERT_NO_THROW(
      {
        neonavigation_common::compat::checkCompatMode();
      });  // NOLINT(whitespace/braces)

  ros::NodeHandle("/").setParam("neonavigation_compatible", 4);
  ASSERT_THROW(
      {
        neonavigation_common::compat::checkCompatMode();
      },  // NOLINT(whitespace/braces)
      std::runtime_error);

  ros::NodeHandle("/").setParam("neonavigation_compatible", 1);
  ASSERT_THROW(
      {
        neonavigation_common::compat::checkCompatMode();
      },  // NOLINT(whitespace/braces)
      std::runtime_error);
}

class NeonavigationCompatCallbacks
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std_msgs::Bool::ConstPtr msg_;
  mutable std_msgs::Bool::ConstPtr msg_const_;
  bool srv_called_;

  void cb(const std_msgs::Bool::ConstPtr& msg)
  {
    msg_ = msg;
  }
  void cbConst(const std_msgs::Bool::ConstPtr& msg) const
  {
    msg_const_ = msg;
  }

  bool cbSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    srv_called_ = true;
    return true;
  }

  NeonavigationCompatCallbacks()
    : pnh_("~")
    , srv_called_(false)
  {
  }
};

TEST(NeonavigationCompat, Subscribe)
{
  neonavigation_common::compat::supported_level = 2;
  neonavigation_common::compat::current_level = 3;
  neonavigation_common::compat::default_level = neonavigation_common::compat::supported_level;

  NeonavigationCompatCallbacks cls;

  ros::Publisher pub_old = cls.pnh_.advertise<std_msgs::Bool>("test_old", 1, true);
  ros::Publisher pub_new = cls.nh_.advertise<std_msgs::Bool>("test_new", 1, true);
  std_msgs::Bool msg;
  msg.data = false;
  pub_old.publish(msg);
  msg.data = true;
  pub_new.publish(msg);

  {
    ros::NodeHandle("/").setParam("neonavigation_compatible", 2);
    ros::Subscriber sub = neonavigation_common::compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &NeonavigationCompatCallbacks::cb, &cls);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_));
    ASSERT_EQ(false, static_cast<bool>(cls.msg_->data));
  }

  {
    ros::NodeHandle("/").setParam("neonavigation_compatible", 3);
    cls.msg_ = nullptr;
    ros::Subscriber sub = neonavigation_common::compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &NeonavigationCompatCallbacks::cb, &cls);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_));
    ASSERT_EQ(true, static_cast<bool>(cls.msg_->data));
  }

  {
    ros::NodeHandle("/").setParam("neonavigation_compatible", 2);
    cls.msg_ = nullptr;
    ros::Subscriber sub = neonavigation_common::compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &NeonavigationCompatCallbacks::cbConst, &cls);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_const_));
    ASSERT_EQ(false, static_cast<bool>(cls.msg_const_->data));
  }

  {
    ros::NodeHandle("/").setParam("neonavigation_compatible", 3);
    cls.msg_ = nullptr;
    ros::Subscriber sub = neonavigation_common::compat::subscribe(
        cls.nh_, "test_new",
        cls.pnh_, "test_old",
        1,
        &NeonavigationCompatCallbacks::cbConst, &cls);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ASSERT_TRUE(static_cast<bool>(cls.msg_const_));
    ASSERT_EQ(true, static_cast<bool>(cls.msg_const_->data));
  }
}

TEST(NeonavigationCompat, AdvertiseService)
{
  neonavigation_common::compat::supported_level = 2;
  neonavigation_common::compat::current_level = 3;
  neonavigation_common::compat::default_level = neonavigation_common::compat::supported_level;

  NeonavigationCompatCallbacks cls;
  ros::ServiceClient cli_new = cls.nh_.serviceClient<std_srvs::Empty>("srv_new");
  ros::ServiceClient cli_old = cls.pnh_.serviceClient<std_srvs::Empty>("srv_old");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  {
    ros::NodeHandle("/").setParam("neonavigation_compatible", 2);

    ros::ServiceServer srv = neonavigation_common::compat::advertiseService(
        cls.nh_, "srv_new",
        cls.pnh_, "srv_old",
        &NeonavigationCompatCallbacks::cbSrv, &cls);
    ros::Duration(0.1).sleep();
    std_srvs::Empty empty;
    ASSERT_TRUE(cli_old.call(empty.request, empty.response));
  }

  {
    ros::NodeHandle("/").setParam("neonavigation_compatible", 3);

    ros::ServiceServer srv = neonavigation_common::compat::advertiseService(
        cls.nh_, "srv_new",
        cls.pnh_, "srv_old",
        &NeonavigationCompatCallbacks::cbSrv, &cls);
    ros::Duration(0.1).sleep();
    std_srvs::Empty empty;
    ASSERT_TRUE(cli_new.call(empty.request, empty.response));
  }

  spinner.stop();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_compat");

  return RUN_ALL_TESTS();
}
