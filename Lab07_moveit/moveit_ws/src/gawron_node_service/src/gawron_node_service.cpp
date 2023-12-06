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

std::string GawronNodeService::planner_selector(int planner_id_selection, std::vector<std::string> planner_id)
{
        return planner_id[planner_id_selection];
}

}  // namespace gawron_node_service
