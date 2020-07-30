/*
 * Copyright (c) 2020, the neonavigation authors
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

#ifndef NEONAVIGATION_COMMON_KILL_NODES_H
#define NEONAVIGATION_COMMON_KILL_NODES_H

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpcClient.h>

#include <string>
#include <vector>

namespace neonavigation_common
{
namespace kill_nodes
{
inline void killAll()
{
  std::vector<std::string> nodes;
  ros::master::getNodes(nodes);
  for (const auto& node : nodes)
  {
    std::cerr << node << std::endl;
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = "test_navigate";
    args[1] = node;
    ros::master::execute("lookupNode", args, result, payload, true);
    if (result.size() > 2)
    {
      std::cerr << result[0] << " " << result[1] << " " << result[2] << std::endl;
      const std::string uri = result[2];
      const size_t pos_colon = uri.find(":", 7);
      const size_t pos_slash = uri.find("/", pos_colon);
      const std::string host(uri, 7, pos_colon - 7);
      const std::string port(uri, pos_colon + 1, pos_slash - pos_colon - 1);
      std::cerr << host << " " << port << std::endl;

      XmlRpc::XmlRpcClient* c =
          ros::XMLRPCManager::instance()->getXMLRPCClient(host, std::stoi(port), uri);
      XmlRpc::XmlRpcValue args, result;
      args[0] = "test_navigate";
      args[1] = "test is end";
      c->execute("shutdown", args, result);
    }
  }
}
}  // namespace kill_nodes
}  // namespace neonavigation_common

#endif  // NEONAVIGATION_COMMON_KILL_NODES_H
