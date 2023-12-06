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

#ifndef GAWRON_NODE_SERVICE__GAWRON_NODE_SERVICE_NODE_HPP_
#define GAWRON_NODE_SERVICE__GAWRON_NODE_SERVICE_NODE_HPP_

#include <memory>

#include "gawron_node_service/gawron_node_service.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace gawron_node_service
{
using GawronNodeServicePtr = std::unique_ptr<gawron_node_service::GawronNodeService>;

class GAWRON_NODE_SERVICE_PUBLIC GawronNodeServiceNode : public rclcpp::Node
{
public:
  explicit GawronNodeServiceNode(const rclcpp::NodeOptions & options);
  rclcpp::Service<gawron_service::srv::CustomService>::SharedPtr service_;

private:
  GawronNodeServicePtr gawron_node_service_{nullptr};
  std::vector<std::string> planner_id_;
  int planner_id_selection;
  std::string planner{"RRTConnectConfigDefault"};
  rclcpp::Service<gawron_service::srv::CustomService>::SharedPtr service;
  void service_msg(const std::shared_ptr<gawron_service::srv::CustomService::Request> request,
                  const std::shared_ptr<gawron_service::srv::CustomService::Response> response);
  void plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                        const std::shared_ptr<gawron_service::srv::CustomService::Response>);
  void pose_target_and_planner(const std::shared_ptr<gawron_service::srv::CustomService::Request> request,
                                moveit::planning_interface::MoveGroupInterface &move_group_interface);
};
}  // namespace gawron_node_service

#endif  // GAWRON_NODE_SERVICE__GAWRON_NODE_SERVICE_NODE_HPP_
