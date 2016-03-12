#include "ros/ros.h"
#include <map_organizer/OccupancyGridArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>


map_organizer::OccupancyGridArray maps;
std::vector<nav_msgs::MapMetaData> orig_mapinfos;
int floor_cur = 0;

void cbMaps(const map_organizer::OccupancyGridArray::Ptr &msg)
{
	ROS_INFO("Map array received");
	maps = *msg;
	orig_mapinfos.clear();
	for(auto &map: maps.maps)
	{
		orig_mapinfos.push_back(map.info);
		map.info.origin.position.z = 0.0;
	}
}
void cbFloor(const std_msgs::Int32::Ptr &msg)
{
	floor_cur = msg->data;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "select_map");
  ros::NodeHandle nh("~");

  auto subMaps  = nh.subscribe("/maps", 1, cbMaps);
  auto subFloor = nh.subscribe("floor", 1, cbFloor);
  auto pubMap   = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  tf::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped trans;
  trans.header.frame_id = "map_ground";
  trans.child_frame_id = "map";
  trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

  ros::Rate wait(10);
  int floor_prev = -1;
  while(ros::ok())
  {
	  wait.sleep();
	  ros::spinOnce();

	  if(maps.maps.size() == 0) continue;

	  if(floor_cur != floor_prev)
	  {
		  if(floor_cur >= 0 && floor_cur < (int)maps.maps.size())
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
	  trans.header.stamp = ros::Time::now();
	  tfb.sendTransform(trans);
  }

  return 0;
}

