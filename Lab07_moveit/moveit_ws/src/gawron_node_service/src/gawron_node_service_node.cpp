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

namespace gawron_node_service
{

void GawronNodeServiceNode::pose_target_and_planner(const std::shared_ptr<gawron_service::srv::CustomService::Request> request,
                                            moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  geometry_msgs::msg::Point position = request->position;
  geometry_msgs::msg::Vector3 orientation = request->orientation;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f"
                "\nroll: %f" " pitch: %f" " yaw: %f",
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z);

  tf2::Quaternion quaternion;
  quaternion.setRPY(orientation.x, orientation.y, orientation.z);

  // Set a target Pose
  auto const target_pose = [&quaternion, &position]{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = quaternion.w();
    msg.position.x = position.x;
    msg.position.y = position.y;
    msg.position.z = position.z;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  move_group_interface.setPlannerId(planner);

  RCLCPP_INFO(this->get_logger(), "Planner: %s", planner.c_str());
  RCLCPP_INFO(this->get_logger(), "Selected planner: %s", move_group_interface.getPlannerId().c_str());
}

void GawronNodeServiceNode::plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                                            const std::shared_ptr<gawron_service::srv::CustomService::Response> response)
{
    auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
    }();
    if(success) {
            move_group_interface.execute(plan);
            response->response = true;
    } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
            response->response = false;
    }
}

void GawronNodeServiceNode::service_msg(const std::shared_ptr<gawron_service::srv::CustomService::Request> request,
                  const std::shared_ptr<gawron_service::srv::CustomService::Response> response)
{
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(shared_from_this(), "panda_arm");

  this->pose_target_and_planner(request, move_group_interface);

  this->plan_and_execute(move_group_interface, response);
}

GawronNodeServiceNode::GawronNodeServiceNode(const rclcpp::NodeOptions & options)
:  Node("gawron_node_service", options)
{
  rclcpp::NodeOptions nonconst_options(options);
    nonconst_options.parameter_overrides(
      {{"planner_id_selection", this->declare_parameter("planner_id_selection", 0)}});

  planner_id_selection = this->get_parameter("planner_id_selection").get_value<int>();
  planner_id_ = this->declare_parameter("planner_id", std::vector<std::string>{"RRTConnectConfigDefault"});

  gawron_node_service_ = std::make_unique<gawron_node_service::GawronNodeService>();

  planner = gawron_node_service_->planner_selector(planner_id_selection, planner_id_);

  service_ = this->create_service<gawron_service::srv::CustomService>("custom_service",
    std::bind(&GawronNodeServiceNode::service_msg, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "ID planner: %i", planner_id_selection);
}

}  // namespace gawron_node_service

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gawron_node_service::GawronNodeServiceNode)
