/*
 * Copyright (c) 2019, the neonavigation authors
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

#include <string>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <map_organizer_msgs/OccupancyGridArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <gtest/gtest.h>

void validateMap0(const nav_msgs::OccupancyGrid& map, const double z)
{
  ASSERT_EQ("map_ground", map.header.frame_id);
  ASSERT_EQ(2u, map.info.width);
  ASSERT_EQ(4u, map.data.size());
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.y);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.z);
  ASSERT_FLOAT_EQ(1.0, map.info.origin.orientation.w);
  ASSERT_FLOAT_EQ(0.1, map.info.resolution);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.position.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.position.y);
  ASSERT_FLOAT_EQ(z, map.info.origin.position.z);
  ASSERT_EQ(100, map.data[0]);
  ASSERT_EQ(0, map.data[1]);
  ASSERT_EQ(100, map.data[2]);
  ASSERT_EQ(100, map.data[3]);
}
void validateMap1(const nav_msgs::OccupancyGrid& map, const double z)
{
  ASSERT_EQ("map_ground", map.header.frame_id);
  ASSERT_EQ(2u, map.info.width);
  ASSERT_EQ(4u, map.data.size());
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.x);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.y);
  ASSERT_FLOAT_EQ(0.0, map.info.origin.orientation.z);
  ASSERT_FLOAT_EQ(1.0, map.info.origin.orientation.w);
  ASSERT_FLOAT_EQ(0.2, map.info.resolution);
  ASSERT_FLOAT_EQ(0.1, map.info.origin.position.x);
  ASSERT_FLOAT_EQ(0.1, map.info.origin.position.y);
  ASSERT_FLOAT_EQ(z, map.info.origin.position.z);
  ASSERT_EQ(100, map.data[0]);
  ASSERT_EQ(0, map.data[1]);
  ASSERT_EQ(0, map.data[2]);
  ASSERT_EQ(100, map.data[3]);
}

TEST(MapOrganizer, MapArray)
{
  ros::NodeHandle nh;

  map_organizer_msgs::OccupancyGridArray::ConstPtr maps;
  const boost::function<void(const map_organizer_msgs::OccupancyGridArray::ConstPtr&)>
      cb = [&maps](const map_organizer_msgs::OccupancyGridArray::ConstPtr& msg) -> void
  {
    maps = msg;
  };
  ros::Subscriber sub = nh.subscribe("maps", 1, cb);

  ros::Rate rate(10.0);
  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (maps)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());

  validateMap0(maps->maps[0], 1.0);
  validateMap1(maps->maps[1], 10.0);
}

TEST(MapOrganizer, Maps)
{
  ros::NodeHandle nh;

  nav_msgs::OccupancyGrid::ConstPtr map[2];
  const boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&, int)>
      cb = [&map](const nav_msgs::OccupancyGrid::ConstPtr& msg,
                  const int id) -> void
  {
    map[id] = msg;
  };
  ros::Subscriber sub0 =
      nh.subscribe<nav_msgs::OccupancyGrid>("map0", 1, boost::bind(cb, _1, 0));
  ros::Subscriber sub1 =
      nh.subscribe<nav_msgs::OccupancyGrid>("map1", 1, boost::bind(cb, _1, 1));

  ros::Rate rate(10.0);
  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (map[0] && map[1])
      break;
  }
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_TRUE(static_cast<bool>(map[i]));
  }
  validateMap0(*(map[0]), 1.0);
  validateMap1(*(map[1]), 10.0);
}

TEST(MapOrganizer, SelectMap)
{
  ros::NodeHandle nh;

  nav_msgs::OccupancyGrid::ConstPtr map;
  const boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)>
      cb = [&map](const nav_msgs::OccupancyGrid::ConstPtr& msg) -> void
  {
    map = msg;
  };
  ros::Subscriber sub =
      nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, cb);
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("floor", 1);

  ros::Rate rate(10.0);
  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (pub.getNumSubscribers() > 0 && map)
      break;
  }
  ASSERT_GT(pub.getNumSubscribers(), 0u);
  ASSERT_TRUE(static_cast<bool>(map));
  validateMap0(*map, 0.0);

  std_msgs::Int32 floor;
  floor.data = 2;  // invalid floor must be ignored
  pub.publish(floor);

  map = nullptr;
  for (int i = 0; i < 10 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (map)
      break;
  }
  ASSERT_FALSE(static_cast<bool>(map));

  floor.data = 1;
  pub.publish(floor);

  map = nullptr;
  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (map)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(map));
  validateMap1(*map, 0.0);
}

TEST(MapOrganizer, SavedMapArray)
{
  ros::NodeHandle nh;

  map_organizer_msgs::OccupancyGridArray::ConstPtr maps;
  const boost::function<void(const map_organizer_msgs::OccupancyGridArray::ConstPtr&)>
      cb = [&maps](const map_organizer_msgs::OccupancyGridArray::ConstPtr& msg) -> void
  {
    maps = msg;
  };
  ros::Subscriber sub = nh.subscribe("saved/maps", 1, cb);

  ros::Rate rate(10.0);
  for (int i = 0; i < 100 && ros::ok(); ++i)
  {
    rate.sleep();
    ros::spinOnce();
    if (maps)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(maps));
  ASSERT_EQ(2u, maps->maps.size());

  validateMap0(maps->maps[0], 1.0);
  validateMap1(maps->maps[1], 10.0);

  // clean temporary files
  ros::NodeHandle pnh("~");
  std::string file_prefix;
  if (pnh.getParam("file_prefix", file_prefix))
  {
    ASSERT_EQ(0, system(std::string("rm -f " + file_prefix + "*").c_str()));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_map_organizer");

  return RUN_ALL_TESTS();
}
