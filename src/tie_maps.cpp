#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_organizer/OccupancyGridArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <sstream>

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class tie_maps_node
{
private:
    ros::NodeHandle n;
    ros::Publisher pubMapArray;
	std::vector<ros::Publisher> pubMap;
public:
	tie_maps_node():
		n("~")
	{
		pubMapArray = n.advertise<map_organizer::OccupancyGridArray>("/maps", 1, true);

		map_organizer::OccupancyGridArray maps;

		std::string files_str;
		std::string mapfname;
		double res;
		double origin[3], height;
		int negate;
		double occ_th, free_th;
		bool trinary = true;
		std::string frame_id;
		n.param("map_files", files_str, std::string(""));
		n.param("frame_id", frame_id, std::string("map"));

		int i = 0;
		std::string file;
		std::stringstream ss(files_str);
		while(std::getline(ss, file, ','))
		{
			std::ifstream fin(file);
			if(fin.fail())
			{
				ROS_ERROR("Map_server could not open %s.", file.c_str());
				ros::shutdown();
				return;
			}
#ifdef HAVE_NEW_YAMLCPP
			// The document loading process changed in yaml-cpp 0.5.
			YAML::Node doc = YAML::Load(fin);
#else
			YAML::Parser parser(fin);
			YAML::Node doc;
			parser.GetNextDocument(doc);
#endif
			try
			{ 
				doc["resolution"] >> res; 
			}
			catch (YAML::InvalidScalar)
			{ 
				ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
				ros::shutdown();
				return;
			}
			try
			{ 
				doc["negate"] >> negate; 
			}
			catch(YAML::InvalidScalar)
			{ 
				ROS_ERROR("The map does not contain a negate tag or it is invalid.");
				ros::shutdown();
				return;
			}
			try
			{ 
				doc["occupied_thresh"] >> occ_th; 
			}
			catch(YAML::InvalidScalar)
			{ 
				ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
				ros::shutdown();
				return;
			}
			try
			{ 
				doc["free_thresh"] >> free_th; 
			}
			catch(YAML::InvalidScalar)
			{ 
				ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
				ros::shutdown();
				return;
			}
			try
			{ 
				doc["trinary"] >> trinary; 
			}
			catch(YAML::Exception)
			{ 
				ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
				trinary = true;
			}
			try
			{ 
				doc["origin"][0] >> origin[0]; 
				doc["origin"][1] >> origin[1]; 
				doc["origin"][2] >> origin[2]; 
			}
			catch (YAML::InvalidScalar)
			{ 
				ROS_ERROR("The map does not contain an origin tag or it is invalid.");
				ros::shutdown();
				return;
			}
			try
			{ 
				doc["height"] >> height;
			}
			catch (YAML::Exception)
			{
			   height = 0;	
			}
			try
			{ 
				doc["image"] >> mapfname; 
				// TODO: make this path-handling more robust
				if(mapfname.size() == 0)
				{
					ROS_ERROR("The image tag cannot be an empty string.");
					ros::shutdown();
					return;
				}
				if(mapfname[0] != '/')
				{
					// dirname can modify what you pass it
					char* fname_copy = strdup(file.c_str());
					mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
					free(fname_copy);
				}
			}
			catch (YAML::InvalidScalar)
			{ 
				ROS_ERROR("The map does not contain an image tag or it is invalid.");
				ros::shutdown();
				return;
			}

			ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());

			nav_msgs::GetMap::Response map_resp;
			map_server::loadMapFromFile(&map_resp,
					mapfname.c_str(), res, negate, occ_th, free_th, origin, trinary);
			map_resp.map.info.origin.position.z = height;
			map_resp.map.info.map_load_time = ros::Time::now();
			map_resp.map.header.frame_id = frame_id;
			map_resp.map.header.stamp = ros::Time::now();
			ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
					map_resp.map.info.width,
					map_resp.map.info.height,
					map_resp.map.info.resolution);
			maps.maps.push_back(map_resp.map);
			pubMap.push_back(n.advertise<nav_msgs::OccupancyGrid>(
						"/map" + std::to_string(i), 1, true));
			pubMap.back().publish(map_resp.map);
			i ++;
		}
		pubMapArray.publish(maps);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tie_maps");

	tie_maps_node tmn;
	ros::spin();

	return 0;
}
