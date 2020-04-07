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

#ifndef COSTMAP_CSPACE_COSTMAP_3D_LAYER_CLASS_LOADER_H
#define COSTMAP_CSPACE_COSTMAP_3D_LAYER_CLASS_LOADER_H

#include <map>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_cspace_msgs/CSpace3D.h>
#include <costmap_cspace_msgs/CSpace3DUpdate.h>

#include <costmap_cspace/cspace3_cache.h>
#include <costmap_cspace/polygon.h>

namespace costmap_cspace
{
class Costmap3dLayerSpawnerBase
{
public:
  using Ptr = std::shared_ptr<Costmap3dLayerSpawnerBase>;
  virtual Costmap3dLayerBase::Ptr spawn() const = 0;
};
template <typename T>
class Costmap3dLayerSpawner : public Costmap3dLayerSpawnerBase
{
public:
  Costmap3dLayerBase::Ptr spawn() const
  {
    return Costmap3dLayerBase::Ptr(new T);
  }
};
class Costmap3dLayerClassLoader
{
protected:
  using ClassList = std::map<std::string, Costmap3dLayerSpawnerBase::Ptr>;
  static ClassList classes_;

public:
  static Costmap3dLayerBase::Ptr loadClass(const std::string& name)
  {
    if (classes_.find(name) == classes_.end())
    {
      throw std::runtime_error("Costmap3dLayerSpawner: class not found");
    }
    return classes_[name]->spawn();
  };
  static void registerClass(const std::string& name, Costmap3dLayerSpawnerBase::Ptr spawner)
  {
    classes_[name] = spawner;
  };
};
#define COSTMAP_3D_LAYER_CLASS_LOADER_ENABLE()         \
  costmap_cspace::Costmap3dLayerClassLoader::ClassList \
      costmap_cspace::Costmap3dLayerClassLoader::classes_;

#define COSTMAP_3D_LAYER_CLASS_LOADER_REGISTER(name, klass, id)   \
  namespace                                                       \
  {                                                               \
  struct ClassLoaderRegister##id                                  \
  {                                                               \
    ClassLoaderRegister##id()                                     \
    {                                                             \
      costmap_cspace::Costmap3dLayerClassLoader::registerClass(   \
          name,                                                   \
          costmap_cspace::Costmap3dLayerSpawnerBase::Ptr(         \
              new costmap_cspace::Costmap3dLayerSpawner<klass>)); \
    } /* NOLINT(whitespace/braces)*/                              \
  };  /* NOLINT(whitespace/braces)*/                              \
  static ClassLoaderRegister##id g_register_class_##id;           \
  } /* NOLINT(readability/namespace) */  // namespace

}  // namespace costmap_cspace

#endif  // COSTMAP_CSPACE_COSTMAP_3D_LAYER_CLASS_LOADER_H
