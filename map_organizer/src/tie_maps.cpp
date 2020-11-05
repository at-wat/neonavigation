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
#include <nav_msgs/OccupancyGrid.h>
#include <map_organizer_msgs/OccupancyGridArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class TieMapNode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Publisher pub_map_array_;
  std::vector<ros::Publisher> pub_map_;

public:
  TieMapNode()
    : pnh_("~")
    , nh_()
  {
    pub_map_array_ = nh_.advertise<map_organizer_msgs::OccupancyGridArray>("maps", 1, true);

    map_organizer_msgs::OccupancyGridArray maps;

    std::string files_str;
    std::string mapfname;
    double res;
    double origin[3], height;
    int negate;
    double occ_th, free_th;
    MapMode mode;
    std::string frame_id;
    pnh_.param("map_files", files_str, std::string(""));
    pnh_.param("frame_id", frame_id, std::string("map"));

    int i = 0;
    std::string file;
    std::stringstream ss(files_str);
    while (std::getline(ss, file, ','))
    {
      std::ifstream fin(file);
      if (fin.fail())
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
      catch (YAML::InvalidScalar& e)
      {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid: %s", e.what());
        ros::shutdown();
        return;
      }
      try
      {
        doc["negate"] >> negate;
      }
      catch (YAML::InvalidScalar& e)
      {
        ROS_ERROR("The map does not contain a negate tag or it is invalid: %s", e.what());
        ros::shutdown();
        return;
      }
      try
      {
        doc["occupied_thresh"] >> occ_th;
      }
      catch (YAML::InvalidScalar& e)
      {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid: %s", e.what());
        ros::shutdown();
        return;
      }
      try
      {
        doc["free_thresh"] >> free_th;
      }
      catch (YAML::InvalidScalar& e)
      {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid: %s", e.what());
        ros::shutdown();
        return;
      }
      try
      {
        std::string modeS = "";
        doc["mode"] >> modeS;

        if (modeS == "trinary")
          mode = TRINARY;
        else if (modeS == "scale")
          mode = SCALE;
        else if (modeS == "raw")
          mode = RAW;
        else
        {
          ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
          exit(-1);
        }
      }
      catch (YAML::Exception& e)
      {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming trinary: %s", e.what());
        mode = TRINARY;
      }
      try
      {
        doc["origin"][0] >> origin[0];
        doc["origin"][1] >> origin[1];
        doc["origin"][2] >> origin[2];
      }
      catch (YAML::InvalidScalar& e)
      {
        ROS_ERROR("The map does not contain an origin tag or it is invalid: %s", e.what());
        ros::shutdown();
        return;
      }
      try
      {
        doc["height"] >> height;
      }
      catch (YAML::Exception& e)
      {
        height = 0;
      }
      try
      {
        doc["image"] >> mapfname;
        // TODO(at-wat): make this path-handling more robust
        if (mapfname.size() == 0)
        {
          ROS_ERROR("The image tag cannot be an empty string.");
          ros::shutdown();
          return;
        }
        if (mapfname[0] != '/')
        {
          // dirname can modify what you pass it
          char* fname_copy = strdup(file.c_str());
          mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
          free(fname_copy);
        }
      }
      catch (YAML::InvalidScalar& e)
      {
        ROS_ERROR("The map does not contain an image tag or it is invalid: e.what()");
        ros::shutdown();
        return;
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());

      nav_msgs::GetMap::Response map_resp;
      map_server::loadMapFromFile(&map_resp,
                                  mapfname.c_str(), res, negate, occ_th, free_th, origin, mode);
      map_resp.map.info.origin.position.z = height;
      map_resp.map.info.map_load_time = ros::Time::now();
      map_resp.map.header.frame_id = frame_id;
      map_resp.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp.map.info.width,
               map_resp.map.info.height,
               map_resp.map.info.resolution);
      maps.maps.push_back(map_resp.map);
      pub_map_.push_back(nh_.advertise<nav_msgs::OccupancyGrid>(
          "map" + std::to_string(i), 1, true));
      pub_map_.back().publish(map_resp.map);
      i++;
    }
    pub_map_array_.publish(maps);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tie_maps");

  TieMapNode tmn;
  ros::spin();

  return 0;
}
