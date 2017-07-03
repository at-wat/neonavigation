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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
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

#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

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

std::vector<std::string> split(const std::string &input, char delimiter)
{
  std::istringstream stream(input);

  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter))
  {
    result.push_back(field);
  }
  return result;
}

class obj_to_pointcloud
{
public:
  obj_to_pointcloud()
    : n("~"), engine(seed_gen())
  {
    pubCloud = n.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);

    n.param("frame_id", frame_id, std::string("map"));
    n.param("objs", file, std::string(""));
    if (file.compare("") == 0)
    {
      ROS_ERROR("OBJ file not specified");
      ros::shutdown();
      return;
    }
    n.param("points_per_meter_sq", ppmsq, 600.0);
    n.param("downsample_grid", downsample_grid, 0.05);
    n.param("offset_x", offset_x, 0.0);
    n.param("offset_y", offset_y, 0.0);
    n.param("offset_z", offset_z, 0.0);
    n.param("scale", scale, 1.0);

    auto pc = convert_obj(split(file, ','));
    pubCloud.publish(pc);
  }

private:
  ros::NodeHandle n;
  ros::Publisher pubCloud;

  std::string file;
  std::string frame_id;
  double ppmsq;
  double downsample_grid;
  double offset_x;
  double offset_y;
  double offset_z;
  double scale;

  std::random_device seed_gen;
  std::default_random_engine engine;

  sensor_msgs::PointCloud2 convert_obj(const std::vector<std::string> &files)
  {
    sensor_msgs::PointCloud2 pc_msg;
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_rs(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointXYZ offset(static_cast<float>(offset_x), static_cast<float>(offset_y), static_cast<float>(offset_z));

    pc_rs->points.reserve(500000);
    for (auto &file : files)
    {
      auto ext = file.substr(file.find_last_of(".") + 1);

      if (ext == "pcd")
      {
        if (pcl::io::loadPCDFile(file, *pc) == -1)
        {
          ROS_ERROR("Failed to load PCD file");
          ros::shutdown();
          return pc_msg;
        }
        for (auto &p : pc->points)
        {
          p.x *= scale;
          p.y *= scale;
          p.z *= scale;
          p = p + offset;
          pc_rs->points.push_back(p);
        }
      }
      else if (ext == "obj")
      {
        if (pcl::io::loadPolygonFileOBJ(file, *mesh) == -1)
        {
          ROS_ERROR("Failed to load OBJ file");
          ros::shutdown();
          return pc_msg;
        }

        pcl::fromPCLPointCloud2(mesh->cloud, *pc);
        pc_rs->header = pc->header;
        for (auto &p : pc->points)
        {
          p.x *= scale;
          p.y *= scale;
          p.z *= scale;
        }

        std::uniform_real_distribution<float> ud(0.0, 1.0);
        for (auto &poly : mesh->polygons)
        {
          if (poly.vertices.size() != 3)
          {
            ROS_ERROR("Input mesh mush be triangle");
            ros::shutdown();
            return pc_msg;
          }
          auto &p0 = pc->points[poly.vertices[0]];
          auto &p1 = pc->points[poly.vertices[1]];
          auto &p2 = pc->points[poly.vertices[2]];

          auto a = p1 - p0;
          auto b = p2 - p0;

          float s = 0.5 * sqrtf(
                              powf(a.y * b.z - a.z * b.y, 2.0) +
                              powf(a.z * b.x - a.x * b.z, 2.0) +
                              powf(a.x * b.y - a.y * b.x, 2.0));
          float numf = ppmsq * s;
          int num = numf;
          if (numf - num > ud(engine))
            num++;
          for (int i = 0; i < num; i++)
          {
            float r0 = ud(engine);
            float r1 = ud(engine);
            if (r0 + r1 > 1.0)
            {
              r0 = 1.0 - r0;
              r1 = 1.0 - r1;
            }
            pcl::PointXYZ p = p0 + (a * r0 + b * r1) + offset;
            pc_rs->points.push_back(p);
          }
        }
      }
    }
    pc_rs->width = pc_rs->points.size();
    pc_rs->height = 1;
    pc_rs->is_dense = true;

    // Down-sample
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds;
    ds.setInputCloud(pc_rs);
    ds.setLeafSize(downsample_grid, downsample_grid, downsample_grid);
    ds.filter(*pc_ds);

    pcl::toROSMsg(*pc_ds, pc_msg);
    pc_msg.header.frame_id = frame_id;
    pc_msg.header.stamp = ros::Time::now();
    ROS_INFO("pointcloud (%d points) has been generated from %d verticles",
             (int)pc_ds->size(),
             (int)pc->size());
    return pc_msg;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obj_to_pointcloud");

  obj_to_pointcloud m2p;
  ros::spin();

  return 0;
}
