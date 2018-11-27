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

#ifndef COSTMAP_CSPACE_NODE_HANDLE_FLOAT_H
#define COSTMAP_CSPACE_NODE_HANDLE_FLOAT_H

#include <ros/ros.h>

#include <string>

namespace ros
{
class NodeHandle_f : public NodeHandle
{
public:
  void param_cast(const std::string& param_name,
                  float& param_val, const float& default_val) const
  {
    double _default_val_d = default_val;
    double _param_val_d;

    param<double>(param_name, _param_val_d, _default_val_d);
    param_val = _param_val_d;
  }
  void param_cast(const std::string& param_name,
                  size_t& param_val, const size_t& default_val) const
  {
    int _default_val_d = default_val;
    int _param_val_d;

    param<int>(param_name, _param_val_d, _default_val_d);
    param_val = _param_val_d;
  }
  NodeHandle_f(const std::string& ns = std::string(),
               const M_string& remappings = M_string())
    : NodeHandle(ns, remappings)
  {
  }
};

}  // namespace ros

#endif  // COSTMAP_CSPACE_NODE_HANDLE_FLOAT_H
