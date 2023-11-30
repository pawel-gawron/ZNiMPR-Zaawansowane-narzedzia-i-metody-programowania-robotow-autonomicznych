// Copyright 2023 Pawel_Gawron
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gawron_node_service/gawron_node_service_node.hpp"

void service_msg(const std::shared_ptr<gawron_service::srv::CustomService::Request> request,
                  const std::shared_ptr<gawron_service::srv::CustomService::Response> response)
{
  response->response = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f",
                request->position.x, request->position.y, request->position.z);
}

namespace gawron_node_service
{

GawronNodeServiceNode::GawronNodeServiceNode(const rclcpp::NodeOptions & options)
:  Node("gawron_node_service", options)
{
  gawron_node_service_ = std::make_unique<gawron_node_service::GawronNodeService>();
  param_name_ = this->declare_parameter("param_name", 456);
  gawron_node_service_->foo(param_name_);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gawron_node_service_node");

  rclcpp::Service<gawron_service::srv::CustomService>::SharedPtr service_ =
    node->create_service<gawron_service::srv::CustomService>("custom_service", &service_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();

}

}  // namespace gawron_node_service

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gawron_node_service::GawronNodeServiceNode)
