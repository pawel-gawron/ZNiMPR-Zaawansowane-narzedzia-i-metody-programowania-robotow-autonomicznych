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

#ifndef GAWRON_NODE_SERVICE__GAWRON_NODE_SERVICE_HPP_
#define GAWRON_NODE_SERVICE__GAWRON_NODE_SERVICE_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "gawron_node_service/visibility_control.hpp"
#include "gawron_service/srv/custom_service.hpp"

#include <moveit/move_group_interface/move_group_interface.h>


namespace gawron_node_service
{

class GAWRON_NODE_SERVICE_PUBLIC GawronNodeService
{
public:
  GawronNodeService();
  std::string planner_selector(int planner_id_selection, std::vector<std::string> planner_id);
};

}  // namespace gawron_node_service

#endif  // GAWRON_NODE_SERVICE__GAWRON_NODE_SERVICE_HPP_
