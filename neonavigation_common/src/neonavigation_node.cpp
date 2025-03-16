/*
 * Copyright (c) 2024, the neonavigation authors
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

#include <string>
#include <functional>
#include <vector>
#include <unordered_map>
#include <variant>
#include <memory>

#include <neonavigation_common/neonavigation_node.hpp>

namespace neonavigation_common
{
NeonavigationNode::NeonavigationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  , parameter_handler_disabled_(false)
{
  create_logger_services();
  parameter_handler_ =
      add_on_set_parameters_callback(std::bind(&NeonavigationNode::cbParameter, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
NeonavigationNode::cbParameter(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  if (parameter_handler_disabled_)
  {
    return result;
  }
  bool parameter_updated = false;
  for (const auto& param : parameters)
  {
    if (params_.find(param.get_name()) != params_.end())
    {
      std::visit(
          [&](auto&& arg)
          {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, int64_t*>)
              *std::get<int64_t*>(params_[param.get_name()]) = param.as_int();
            else if constexpr (std::is_same_v<T, bool*>)
              *std::get<bool*>(params_[param.get_name()]) = param.as_bool();
            else if constexpr (std::is_same_v<T, double*>)
              *std::get<double*>(params_[param.get_name()]) = param.as_double();
            else if constexpr (std::is_same_v<T, std::string*>)
              *std::get<std::string*>(params_[param.get_name()]) = param.as_string();
          },
          params_[param.get_name()]);
      parameter_updated = true;
    }
  }
  if (parameter_updated)
  {
    onDynamicParameterUpdated(parameters);
  }
  return result;
}

// This function is copied from the "jazzy" branch of rclcpp/src/rclcpp/node_interfaces/node_logging.cpp
void NeonavigationNode::create_logger_services()
{
  const std::string node_name = get_name();
  get_loggers_service_ = create_service<neonavigation_common_msgs::srv::GetLoggerLevels>(
      node_name + "/get_logger_levels",
      [](const std::shared_ptr<rmw_request_id_t>,
         const std::shared_ptr<neonavigation_common_msgs::srv::GetLoggerLevels::Request> request,
         std::shared_ptr<neonavigation_common_msgs::srv::GetLoggerLevels::Response> response)
      {
        for (auto& name : request->names)
        {
          neonavigation_common_msgs::msg::LoggerLevel logger_level;
          logger_level.name = name;
          auto ret = rcutils_logging_get_logger_level(name.c_str());
          if (ret < 0)
          {
            logger_level.level = 0;
          }
          else
          {
            logger_level.level = static_cast<uint8_t>(ret);
          }
          response->levels.push_back(std::move(logger_level));
        }
      });

  set_loggers_service_ = create_service<neonavigation_common_msgs::srv::SetLoggerLevels>(
      node_name + "/set_logger_levels",
      [](const std::shared_ptr<rmw_request_id_t>,
         const std::shared_ptr<neonavigation_common_msgs::srv::SetLoggerLevels::Request> request,
         std::shared_ptr<neonavigation_common_msgs::srv::SetLoggerLevels::Response> response)
      {
        neonavigation_common_msgs::msg::SetLoggerLevelsResult result;
        for (auto& level : request->levels)
        {
          auto ret = rcutils_logging_set_logger_level(level.name.c_str(), level.level);
          if (ret != RCUTILS_RET_OK)
          {
            result.successful = false;
            result.reason = rcutils_get_error_string().str;
          }
          else
          {
            result.successful = true;
          }
          response->results.push_back(std::move(result));
        }
      });
}

}  // namespace neonavigation_common
