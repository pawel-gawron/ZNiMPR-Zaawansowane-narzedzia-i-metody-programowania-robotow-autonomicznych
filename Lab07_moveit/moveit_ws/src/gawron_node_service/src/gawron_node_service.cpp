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

#include "gawron_node_service/gawron_node_service.hpp"

#include <iostream>

namespace gawron_node_service
{

GawronNodeService::GawronNodeService()
{
}

int64_t GawronNodeService::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

// void GawronNodeService::service_msg(const std::shared_ptr<gawron_service::srv::CustomService::Request> request)
// {
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f",
//                 request->position.x, request->position.y, request->position.z);
// }

}  // namespace gawron_node_service