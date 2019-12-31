/*
 * Copyright (c) 2014-2018, the neonavigation authors
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
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include <string>
#include <utility>
#include <vector>

#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <costmap_cspace/costmap_3d.h>
#include <neonavigation_common/compatibility.h>

class Costmap3DOFNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_map_;
  std::vector<ros::Subscriber> sub_map_overlay_;
  ros::Publisher pub_costmap_;
  ros::Publisher pub_costmap_update_;
  ros::Publisher pub_footprint_;
  ros::Publisher pub_debug_;
  ros::Timer timer_footprint_;

  costmap_cspace::Costmap3d::Ptr costmap_;
  std::vector<
      std::pair<nav_msgs::OccupancyGrid::ConstPtr,
                costmap_cspace::Costmap3dLayerBase::Ptr>> map_buffer_;

  void cbMap(
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const costmap_cspace::Costmap3dLayerBase::Ptr map)
  {
    if (map->getAngularGrid() <= 0)
    {
      ROS_ERROR("ang_resolution is not set.");
      std::runtime_error("ang_resolution is not set.");
    }
    ROS_INFO("2D costmap received");

    map->setBaseMap(msg);
    ROS_DEBUG("C-Space costmap generated");

    if (map_buffer_.size() > 0)
    {
      for (auto& map : map_buffer_)
        cbMapOverlay(map.first, map.second);
      ROS_INFO("%ld buffered costmaps processed", map_buffer_.size());
      map_buffer_.clear();
    }
  }
  void cbMapOverlay(
      const nav_msgs::OccupancyGrid::ConstPtr& msg,
      const costmap_cspace::Costmap3dLayerBase::Ptr map)
  {
    ROS_DEBUG("Overlay 2D costmap received");

    auto map_msg = map->getMap();
    if (map_msg->info.width < 1 ||
        map_msg->info.height < 1)
    {
      map_buffer_.push_back(
          std::pair<nav_msgs::OccupancyGrid::ConstPtr,
                    costmap_cspace::Costmap3dLayerBase::Ptr>(msg, map));
      return;
    }

    map->processMapOverlay(msg);
    ROS_DEBUG("C-Space costmap updated");
  }
  bool cbUpdateStatic(
      const costmap_cspace::CSpace3DMsg::Ptr map,
      const costmap_cspace_msgs::CSpace3DUpdate::Ptr update)
  {
    publishDebug(*map);
    pub_costmap_.publish<costmap_cspace_msgs::CSpace3D>(*map);
    return true;
  }
  bool cbUpdate(
      const costmap_cspace::CSpace3DMsg::Ptr map,
      const costmap_cspace_msgs::CSpace3DUpdate::Ptr update)
  {
    if (update)
    {
      publishDebug(*map);
      pub_costmap_update_.publish(*update);
    }
    else
    {
      ROS_WARN("Updated region of the costmap is empty. The position may be out-of-boundary, or input map is wrong.");
    }
    return true;
  }
  void publishDebug(const costmap_cspace_msgs::CSpace3D& map)
  {
    if (pub_debug_.getNumSubscribers() == 0)
      return;
    sensor_msgs::PointCloud pc;
    pc.header = map.header;
    pc.header.stamp = ros::Time::now();
    for (size_t yaw = 0; yaw < map.info.angle; yaw++)
    {
      for (unsigned int i = 0; i < map.info.width * map.info.height; i++)
      {
        int gx = i % map.info.width;
        int gy = i / map.info.width;
        if (map.data[i + yaw * map.info.width * map.info.height] < 100)
          continue;
        geometry_msgs::Point32 p;
        p.x = gx * map.info.linear_resolution + map.info.origin.position.x;
        p.y = gy * map.info.linear_resolution + map.info.origin.position.y;
        p.z = yaw * 0.1;
        pc.points.push_back(p);
      }
    }
    pub_debug_.publish(pc);
  }
  void cbPublishFootprint(const ros::TimerEvent& event, const geometry_msgs::PolygonStamped msg)
  {
    auto footprint = msg;
    footprint.header.stamp = ros::Time::now();
    pub_footprint_.publish(footprint);
  }

  static costmap_cspace::MapOverlayMode getMapOverlayModeFromString(
      const std::string overlay_mode_str)
  {
    if (overlay_mode_str == "overwrite")
    {
      return costmap_cspace::MapOverlayMode::OVERWRITE;
    }
    else if (overlay_mode_str == "max")
    {
      return costmap_cspace::MapOverlayMode::MAX;
    }
    ROS_FATAL("Unknown overlay_mode \"%s\"", overlay_mode_str.c_str());
    throw std::runtime_error("Unknown overlay_mode.");
  };

public:
  Costmap3DOFNode()
    : nh_()
    , pnh_("~")
  {
    neonavigation_common::compat::checkCompatMode();
    pub_costmap_ = neonavigation_common::compat::advertise<costmap_cspace_msgs::CSpace3D>(
        nh_, "costmap",
        pnh_, "costmap", 1, true);
    pub_costmap_update_ = neonavigation_common::compat::advertise<costmap_cspace_msgs::CSpace3DUpdate>(
        nh_, "costmap_update",
        pnh_, "costmap_update", 1, true);
    pub_footprint_ = pnh_.advertise<geometry_msgs::PolygonStamped>("footprint", 2, true);
    pub_debug_ = pnh_.advertise<sensor_msgs::PointCloud>("debug", 1, true);

    int ang_resolution;
    pnh_.param("ang_resolution", ang_resolution, 16);

    XmlRpc::XmlRpcValue footprint_xml;
    if (!pnh_.hasParam("footprint"))
    {
      ROS_FATAL("Footprint doesn't specified");
      throw std::runtime_error("Footprint doesn't specified.");
    }
    pnh_.getParam("footprint", footprint_xml);
    costmap_cspace::Polygon footprint;
    try
    {
      footprint = costmap_cspace::Polygon(footprint_xml);
    }
    catch (const std::exception& e)
    {
      ROS_FATAL("Invalid footprint");
      throw e;
    }

    costmap_.reset(new costmap_cspace::Costmap3d(ang_resolution));

    auto root_layer = costmap_->addRootLayer<costmap_cspace::Costmap3dLayerFootprint>();
    float linear_expand;
    float linear_spread;
    pnh_.param("linear_expand", linear_expand, 0.2f);
    pnh_.param("linear_spread", linear_spread, 0.5f);
    root_layer->setExpansion(linear_expand, linear_spread);
    root_layer->setFootprint(footprint);

    if (pnh_.hasParam("static_layers"))
    {
      XmlRpc::XmlRpcValue layers_xml;
      pnh_.getParam("static_layers", layers_xml);

      if (layers_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || layers_xml.size() < 1)
      {
        ROS_FATAL("static_layers parameter must contain at least one layer config.");
        ROS_ERROR(
            "Migration from old version:\n"
            "---  # Old\n"
            "static_layers:\n"
            "  YOUR_LAYER_NAME:\n"
            "    type: LAYER_TYPE\n"
            "    parameters: values\n"
            "---  # New\n"
            "static_layers:\n"
            "  - name: YOUR_LAYER_NAME\n"
            "    type: LAYER_TYPE\n"
            "    parameters: values\n"
            "---\n");
        throw std::runtime_error("layers parameter must contain at least one layer config.");
      }
      for (int i = 0; i < layers_xml.size(); ++i)
      {
        auto layer_xml = std::pair<std::string, XmlRpc::XmlRpcValue>(
            layers_xml[i]["name"], layers_xml[i]);
        ROS_INFO("New static layer: %s", layer_xml.first.c_str());

        costmap_cspace::MapOverlayMode overlay_mode(costmap_cspace::MapOverlayMode::MAX);
        if (layer_xml.second["overlay_mode"].getType() == XmlRpc::XmlRpcValue::TypeString)
          overlay_mode = getMapOverlayModeFromString(
              layer_xml.second["overlay_mode"]);
        else
          ROS_WARN("overlay_mode of the static layer is not specified. Using MAX mode.");

        std::string type;
        if (layer_xml.second["type"].getType() == XmlRpc::XmlRpcValue::TypeString)
          type = std::string(layer_xml.second["type"]);
        else
        {
          ROS_FATAL("Layer type is not specified.");
          throw std::runtime_error("Layer type is not specified.");
        }

        if (!layer_xml.second.hasMember("footprint"))
          layer_xml.second["footprint"] = footprint_xml;

        costmap_cspace::Costmap3dLayerBase::Ptr layer =
            costmap_cspace::Costmap3dLayerClassLoader::loadClass(type);
        costmap_->addLayer(layer, overlay_mode);
        layer->loadConfig(layer_xml.second);

        sub_map_overlay_.push_back(nh_.subscribe<nav_msgs::OccupancyGrid>(
            layer_xml.first, 1,
            boost::bind(&Costmap3DOFNode::cbMapOverlay, this, _1, layer)));
      }
    }

    auto static_output_layer = costmap_->addLayer<costmap_cspace::Costmap3dLayerOutput>();
    static_output_layer->setHandler(boost::bind(&Costmap3DOFNode::cbUpdateStatic, this, _1, _2));

    sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "map", 1,
        boost::bind(&Costmap3DOFNode::cbMap, this, _1, root_layer));

    if (pnh_.hasParam("layers"))
    {
      XmlRpc::XmlRpcValue layers_xml;
      pnh_.getParam("layers", layers_xml);

      if (layers_xml.getType() != XmlRpc::XmlRpcValue::TypeArray || layers_xml.size() < 1)
      {
        ROS_FATAL("layers parameter must contain at least one layer config.");
        ROS_ERROR(
            "Migration from old version:\n"
            "---  # Old\n"
            "layers:\n"
            "  YOUR_LAYER_NAME:\n"
            "    type: LAYER_TYPE\n"
            "    parameters: values\n"
            "---  # New\n"
            "layers:\n"
            "  - name: YOUR_LAYER_NAME\n"
            "    type: LAYER_TYPE\n"
            "    parameters: values\n"
            "---\n");
        throw std::runtime_error("layers parameter must contain at least one layer config.");
      }
      for (int i = 0; i < layers_xml.size(); ++i)
      {
        auto layer_xml = std::pair<std::string, XmlRpc::XmlRpcValue>(
            layers_xml[i]["name"], layers_xml[i]);
        ROS_INFO("New layer: %s", layer_xml.first.c_str());

        costmap_cspace::MapOverlayMode overlay_mode(costmap_cspace::MapOverlayMode::MAX);
        if (layer_xml.second["overlay_mode"].getType() == XmlRpc::XmlRpcValue::TypeString)
          overlay_mode = getMapOverlayModeFromString(
              layer_xml.second["overlay_mode"]);
        else
          ROS_WARN("overlay_mode of the layer is not specified. Using MAX mode.");

        std::string type;
        if (layer_xml.second["type"].getType() == XmlRpc::XmlRpcValue::TypeString)
          type = std::string(layer_xml.second["type"]);
        else
        {
          ROS_FATAL("Layer type is not specified.");
          throw std::runtime_error("Layer type is not specified.");
        }

        if (!layer_xml.second.hasMember("footprint"))
          layer_xml.second["footprint"] = footprint_xml;

        costmap_cspace::Costmap3dLayerBase::Ptr layer =
            costmap_cspace::Costmap3dLayerClassLoader::loadClass(type);
        costmap_->addLayer(layer, overlay_mode);
        layer->loadConfig(layer_xml.second);

        sub_map_overlay_.push_back(nh_.subscribe<nav_msgs::OccupancyGrid>(
            layer_xml.first, 1,
            boost::bind(&Costmap3DOFNode::cbMapOverlay, this, _1, layer)));
      }
    }
    else
    {
      // Single layer mode for backward-compatibility
      costmap_cspace::MapOverlayMode overlay_mode;
      std::string overlay_mode_str;
      pnh_.param("overlay_mode", overlay_mode_str, std::string("max"));
      if (overlay_mode_str.compare("overwrite") == 0)
        overlay_mode = costmap_cspace::MapOverlayMode::OVERWRITE;
      else if (overlay_mode_str.compare("max") == 0)
        overlay_mode = costmap_cspace::MapOverlayMode::MAX;
      else
      {
        ROS_FATAL("Unknown overlay_mode \"%s\"", overlay_mode_str.c_str());
        throw std::runtime_error("Unknown overlay_mode.");
      }
      ROS_INFO("costmap_3d: %s mode", overlay_mode_str.c_str());

      XmlRpc::XmlRpcValue layer_xml;
      layer_xml["footprint"] = footprint_xml;
      layer_xml["linear_expand"] = linear_expand;
      layer_xml["linear_spread"] = linear_spread;

      auto layer = costmap_->addLayer<costmap_cspace::Costmap3dLayerFootprint>(overlay_mode);
      layer->loadConfig(layer_xml);
      sub_map_overlay_.push_back(nh_.subscribe<nav_msgs::OccupancyGrid>(
          "map_overlay", 1,
          boost::bind(&Costmap3DOFNode::cbMapOverlay, this, _1, layer)));
    }

    auto update_output_layer = costmap_->addLayer<costmap_cspace::Costmap3dLayerOutput>();
    update_output_layer->setHandler(boost::bind(&Costmap3DOFNode::cbUpdate, this, _1, _2));

    const geometry_msgs::PolygonStamped footprint_msg = footprint.toMsg();
    timer_footprint_ = nh_.createTimer(
        ros::Duration(1.0),
        boost::bind(&Costmap3DOFNode::cbPublishFootprint, this, _1, footprint_msg));
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "costmap_3d");

  Costmap3DOFNode cm;
  ros::spin();

  return 0;
}
