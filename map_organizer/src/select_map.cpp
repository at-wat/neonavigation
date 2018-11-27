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

#include <map_organizer_msgs/OccupancyGridArray.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

#include <neonavigation_common/compatibility.h>

map_organizer_msgs::OccupancyGridArray maps;
std::vector<nav_msgs::MapMetaData> orig_mapinfos;
int floor_cur = 0;

void cbMaps(const map_organizer_msgs::OccupancyGridArray::Ptr& msg)
{
  ROS_INFO("Map array received");
  maps = *msg;
  orig_mapinfos.clear();
  for (auto& map : maps.maps)
  {
    orig_mapinfos.push_back(map.info);
    map.info.origin.position.z = 0.0;
  }
}
void cbFloor(const std_msgs::Int32::Ptr& msg)
{
  floor_cur = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "select_map");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh("");

  neonavigation_common::compat::checkCompatMode();
  auto subMaps = neonavigation_common::compat::subscribe(
      nh, "maps",
      nh, "/maps", 1, cbMaps);
  auto subFloor = neonavigation_common::compat::subscribe(
      nh, "floor",
      pnh, "floor", 1, cbFloor);
  auto pubMap = neonavigation_common::compat::advertise<nav_msgs::OccupancyGrid>(
      nh, "map",
      nh, "/map", 1, true);

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped trans;
  trans.header.frame_id = "map_ground";
  trans.child_frame_id = "map";
  trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), 0.0));

  ros::Rate wait(10);
  int floor_prev = -1;
  while (ros::ok())
  {
    wait.sleep();
    ros::spinOnce();

    if (maps.maps.size() == 0)
      continue;

    if (floor_cur != floor_prev)
    {
      if (floor_cur >= 0 && floor_cur < static_cast<int>(maps.maps.size()))
      {
        pubMap.publish(maps.maps[floor_cur]);
        trans.transform.translation.z = orig_mapinfos[floor_cur].origin.position.z;
      }
      else
      {
        ROS_INFO("Floor out of range");
      }
      floor_prev = floor_cur;
    }
    trans.header.stamp = ros::Time::now() + ros::Duration(0.15);
    tfb.sendTransform(trans);
  }

  return 0;
}
