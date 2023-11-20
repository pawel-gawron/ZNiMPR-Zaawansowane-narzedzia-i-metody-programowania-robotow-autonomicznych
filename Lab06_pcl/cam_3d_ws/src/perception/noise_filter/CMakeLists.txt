# Copyright 2022 Amadeusz Szymko
# Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(noise_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

ament_auto_find_build_dependencies(REQUIRED
    ${${PROJECT_NAME}_BUILD_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(NOISE_FILTER_SRC
    src/noise_filter.cpp
)

set(NOISE_FILTER_HEADERS
    include/noise_filter/noise_filter.hpp
    include/noise_filter/visibility_control.hpp
)

ament_auto_add_library(noise_filter SHARED
    ${NOISE_FILTER_SRC}
    ${NOISE_FILTER_HEADERS}
)

set(NOISE_FILTER_NODE_SRC
    src/noise_filter_node.cpp
)

set(NOISE_FILTER_NODE_HEADERS
    include/noise_filter/noise_filter_node.hpp
    include/noise_filter/visibility_control.hpp
)

ament_auto_add_library(noise_filter_node SHARED
    ${NOISE_FILTER_NODE_SRC}
    ${NOISE_FILTER_NODE_HEADERS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_node(noise_filter_node
    PLUGIN "perception::noise_filter::NoiseFilterNode"
    EXECUTABLE noise_filter_node_exe
)

target_link_libraries(noise_filter
    ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
