#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_organizer/OccupancyGridArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <sstream>

pcl::PointXYZ operator-(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
	auto c = a;
	c.x -= b.x;
	c.y -= b.y;
	c.z -= b.z;
	return c;
}
pcl::PointXYZ operator+(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
	auto c = a;
	c.x += b.x;
	c.y += b.y;
	c.z += b.z;
	return c;
}
pcl::PointXYZ operator*(const pcl::PointXYZ &a, const float &b)
{
	auto c = a;
	c.x *= b;
	c.y *= b;
	c.z *= b;
	return c;
}

class pointcloud_to_maps
{
public:
	pointcloud_to_maps():
		n("~")
	{
		subPoints = n.subscribe("map_cloud", 1, &pointcloud_to_maps::cbPoints, this);
		pubMapArray = n.advertise<map_organizer::OccupancyGridArray>("/maps", 1, true);
	}
	void cbPoints(const sensor_msgs::PointCloud2::Ptr &msg)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(*msg, *pc);

		const float grid = 0.05;
		const int min_points = 500;
		const int robot_width = 4;
		const int robot_height = 10;
		const float min_floorage = 5.0;
		const float floorage_filter = 0.5;

		std::map<int, int> hist;
		int x_min = INT_MAX, x_max = 0;
		int y_min = INT_MAX, y_max = 0;
		int h_min = INT_MAX, h_max = 0;

		for(auto &p: pc->points)
		{
			int h = (p.z / grid);
			int x = (p.x / grid);
			int y = (p.y / grid);
			if(x_min > x) x_min = x;
			if(y_min > y) y_min = y;
			if(h_min > h) h_min = h;
			if(x_max < x) x_max = x;
			if(y_max < y) y_max = y;
			if(h_max < h) h_max = h;
			if(hist.find(h) == hist.end()) hist[h] = 0;
			hist[h] ++;
		}
		int max_height = h_max;
		float floorage[max_height];

		nav_msgs::MapMetaData mmd;
		mmd.resolution = grid;
		mmd.origin.position.x = x_min * grid;
		mmd.origin.position.y = y_min * grid;
		mmd.origin.orientation.w = 1.0;
		mmd.width = x_max - x_min;
		mmd.height = y_max - y_min;
		ROS_INFO("width %d, height %d", mmd.width, mmd.height);
		std::vector<nav_msgs::OccupancyGrid> maps;

		int map_num = 0;
		for(int i = 0; i < max_height; i ++)
		{
			if(hist[i] > min_points)
			{
				std::map<std::pair<int, int>, char> floor;
				for(auto &p: pc->points)
				{
					int x = (p.x / grid);
					int y = (p.y / grid);
					int z = (p.z / grid);
					auto v = std::pair<int, int>(x, y);

					if(i == z)
					{
						if(floor.find(v) == floor.end())
						{
							floor[v] = 0;
						}
					}
					else if(i < z && z <= i + robot_height)
					{
						floor[v] = 1;
					}
				}
				int cnt = 0;
				for(auto &m: floor)
				{
					if(m.second == 0) cnt ++;
				}
				floorage[i] = cnt * (grid*grid);
				if(floorage[i] < floorage_filter)
				{
					floorage[i] = 0;
				}
				else
				{
					nav_msgs::OccupancyGrid map;
					map.info = mmd;
					map.info.origin.position.z = i * grid;
					map.header = msg->header;
					map.data.resize(mmd.width * mmd.height);
					for(auto &c: map.data) c = -1;
					for(auto &m: floor)
					{
						int addr = (m.first.first - x_min) + (m.first.second - y_min) * mmd.width;
						if(m.second == 0)
						{
							map.data[addr] = 0;
						}
						else if(m.second == 1) map.data[addr] = 100;
					}

					maps.push_back(map);
					map_num ++;
				}
			}
			else
			{
				floorage[i] = 0;
			}
		}
		auto it_prev = maps.rbegin();
		for(auto it = maps.rbegin() + 1; it != maps.rend(); it ++)
		{
			if(fabs(it_prev->info.origin.position.z - it->info.origin.position.z) < grid * 1.5)
			{
				// merge slopes
				for(unsigned int i = 0; i < it->data.size(); i ++)
				{
					if(it->data[i] == 100 && it_prev->data[i] == 0)
					{
						it->data[i] = 0;
						it_prev->data[i] = -1;
					}
				}
			}
			it_prev = it;
		}
		float floorage_ext[map_num];
		int num = 0;
		for(auto &map: maps)
		{
			auto map_exp = map.data;
			for(unsigned int i = 0; i < map_exp.size(); i ++)
			{
				if(map.data[i] != 0)
				{
					for(int xp = -robot_width; xp <= robot_width; xp ++)
					{
						for(int yp = -robot_width; yp <= robot_width; yp ++)
						{
							if(xp * xp + yp * yp > robot_width * robot_width) continue;
							unsigned int x = i % mmd.width + xp;
							unsigned int y = i / mmd.width + yp;
							if(x >= mmd.width) continue;
							if(y >= mmd.height) continue;
							int addr = x + y * mmd.width;
							map_exp[addr] = map.data[i];
						}
					}
				}
			}
			int cnt = 0;
			for(auto &c: map_exp) if(c == 0) cnt ++;
			floorage_ext[num] = cnt * grid * grid;
			num ++;
		}
		num = -1;
		int floor_num = 0;
		map_organizer::OccupancyGridArray map_array;
		for(auto &map: maps)
		{
			num ++;
			if(floorage_ext[num] < min_floorage) continue;
			std::string name = "map" + std::to_string(floor_num);
			pubMaps[name] = n.advertise<nav_msgs::OccupancyGrid>(name, 1, true);
			pubMaps[name].publish(map);
			map_array.maps.push_back(map);
			ROS_ERROR("floor %d (%5.2fm^2), h = %0.2fm", 
					floor_num, floorage_ext[num], map.info.origin.position.z);
			floor_num ++;
		}
		pubMapArray.publish(map_array);
	}
private:
	ros::NodeHandle n;
	std::map<std::string, ros::Publisher> pubMaps;
	ros::Publisher pubMapArray;
	ros::Subscriber subPoints;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pointcloud_to_maps");

	pointcloud_to_maps p2m;
	ros::spin();

	return 0;
}

