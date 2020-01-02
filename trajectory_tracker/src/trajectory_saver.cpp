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
 */

#include <cmath>
#include <fstream>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include <neonavigation_common/compatibility.h>

class SaverNode
{
public:
  SaverNode();
  ~SaverNode();
  void save();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_path_;

  std::string topic_path_;
  std::string filename_;
  bool saved_;
  void cbPath(const nav_msgs::Path::ConstPtr& msg);
};

SaverNode::SaverNode()
  : nh_()
  , pnh_("~")
  , saved_(false)
{
  neonavigation_common::compat::checkCompatMode();
  neonavigation_common::compat::deprecatedParam(pnh_, "path", topic_path_, std::string("recpath"));
  pnh_.param("file", filename_, std::string("a.path"));

  sub_path_ = neonavigation_common::compat::subscribe(
      nh_, "path",
      pnh_, topic_path_, 10, &SaverNode::cbPath, this);
}
SaverNode::~SaverNode()
{
}

void SaverNode::cbPath(const nav_msgs::Path::ConstPtr& msg)
{
  if (saved_)
    return;
  std::ofstream ofs(filename_.c_str());

  if (!ofs)
  {
    ROS_ERROR("Failed to open %s", filename_.c_str());
    return;
  }

  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  ROS_INFO("Size: %d\n", (int)serial_size);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);

  ofs.write(reinterpret_cast<char*>(buffer.get()), serial_size);

  saved_ = true;
}

void SaverNode::save()
{
  ros::Rate loop_rate(5);
  ROS_INFO("Waiting for the path");

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (saved_)
      break;
  }
  ROS_INFO("Path saved");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_saver");

  SaverNode rec;
  rec.save();

  return 0;
}
