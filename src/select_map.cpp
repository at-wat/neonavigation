#include "ros/ros.h"
#include <map_organizer/OccupancyGridArray.h>
#include <std_msgs/Int32.h>


map_organizer::OccupancyGridArray maps;
int floor_cur = 0;

void cbMaps(const map_organizer::OccupancyGridArray::Ptr &msg)
{
	ROS_INFO("Map array received");
	maps = *msg;
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
		  }
		  else
		  {
			  ROS_INFO("Floor out of range");
		  }
		  floor_prev = floor_cur;
	  }
  }

  return 0;
}

