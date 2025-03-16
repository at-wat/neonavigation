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

#ifndef NEONAVIGATION_COMMON__NEONAVIGATION_NODE_HPP_
#define NEONAVIGATION_COMMON__NEONAVIGATION_NODE_HPP_

#include <string>
#include <variant>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <neonavigation_common_msgs/srv/get_logger_levels.hpp>
#include <neonavigation_common_msgs/srv/set_logger_levels.hpp>

namespace neonavigation_common
{
class NeonavigationNode : public rclcpp::Node
{
public:
  explicit NeonavigationNode(const std::string& node_name, const rclcpp::NodeOptions& options);

  // Disable copy and move. declare_dynamic_parameter() will not work correctly if copied.
  NeonavigationNode& operator=(const NeonavigationNode&) = delete;
  NeonavigationNode(const NeonavigationNode&) = delete;

protected:
  OnSetParametersCallbackHandle::SharedPtr parameter_handler_;
  bool parameter_handler_disabled_;
  using ParamType = std::variant<int64_t*, double*, bool*, std::string*>;
  std::unordered_map<std::string, ParamType> params_;
  rclcpp::Service<neonavigation_common_msgs::srv::GetLoggerLevels>::SharedPtr get_loggers_service_;
  rclcpp::Service<neonavigation_common_msgs::srv::SetLoggerLevels>::SharedPtr set_loggers_service_;

  template <typename T>
  void declare_dynamic_parameter(const std::string& name, T* const param, const T& default_value)
  {
    // Disable cbParameter while declaring parameters
    parameter_handler_disabled_ = true;
    *param = declare_parameter(name, default_value);
    params_[name] = param;
    parameter_handler_disabled_ = false;
  }

  void create_logger_services();
  rcl_interfaces::msg::SetParametersResult cbParameter(const std::vector<rclcpp::Parameter>& parameters);

  // This function is called after a parameter is updated.
  virtual void onDynamicParameterUpdated(const std::vector<rclcpp::Parameter>&)
  {
  }
};

}  // namespace neonavigation_common

#endif  // NEONAVIGATION_COMMON__NEONAVIGATION_NODE_HPP_
